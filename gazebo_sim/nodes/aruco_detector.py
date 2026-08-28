"""真 ArUco 影像定位節點。

核心 `ArucoWorldLocalizer` 只依賴 OpenCV / numpy 與 phase1 偵測器設定；
ROS2 訂閱、時間戳與訊息轉換集中在 `main()` 的薄包裝。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from gazebo_sim.generators.generate_robot_sdf import CAMERA_FRONT_X_M, CAMERA_HEIGHT_M
from vgr_driver.vision import ArucoDetector, detect_markers
from vgr_core.safety import Pose


DEFAULT_CAMERA_POSE_ON_ROBOT = (CAMERA_FRONT_X_M, 0.0, CAMERA_HEIGHT_M, 0.0)
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MARKER_MAP_PATH = REPO_ROOT / "gazebo_sim" / "models" / "markers" / "marker_map.json"
DEFAULT_CAMERA_INFO_PATH = (
    REPO_ROOT / "gazebo_sim" / "models" / "vgr_diff_drive" / "camera_info.json"
)
BODY_FROM_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)
DETECTABLE_ARUCO_SIZE_FRACTION = 0.8
MARKER_AREA_WEIGHT_POWER = 3.0
# Planar PnP can produce two similarly plausible poses for a front-facing marker.
# Reject ambiguous solutions when reprojection error cannot distinguish them.
# Physical consistency gates on camera height/tilt reject flipped solutions;
# reprojection and edge-completeness checks handle malformed detections.
EDGE_MARGIN_PX = 4.0
REPROJ_MAX_PX = 2.0
AMBIGUITY_RATIO = 0.6
CAM_Z_TOL_M = 0.20
CAM_TILT_TOL_RAD = math.radians(15.0)


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ArucoWorldLocalizer:
    """由 BGR 影像估測 chassis 原點的世界座標位姿。"""

    def __init__(
        self,
        marker_map: dict,
        camera_info: dict,
        camera_pose_on_robot: tuple[float, float, float, float],
        *,
        edge_margin_px: float = EDGE_MARGIN_PX,
        reproj_max_px: float = REPROJ_MAX_PX,
        ambiguity_ratio: float = AMBIGUITY_RATIO,
    ) -> None:
        self._edge_margin_px = float(edge_margin_px)
        self._reproj_max_px = float(reproj_max_px)
        self._ambiguity_ratio = float(ambiguity_ratio)
        # Accumulated rejection diagnostics for periodic node reporting.
        self.stats = {"used": 0, "rej_edge": 0, "rej_reproj": 0,
                      "rej_implausible": 0, "rej_ambiguous": 0}
        self.last_used_ids: list[int] = []
        # 每個 marker 可用 "dictionary" 覆寫地圖預設字典（實地混印 5x5/6x6）。
        # id 需跨字典唯一——偵測結果只以 id 對回地圖，撞號會混淆。
        default_dict = marker_map.get("dictionary", "DICT_6X6_250")
        self.marker_map = {int(marker["id"]): marker for marker in marker_map["markers"]}
        if len(self.marker_map) != len(marker_map["markers"]):
            raise ValueError("marker ids must be unique across the marker map")
        self._marker_dict_names = {
            marker_id: marker.get("dictionary", default_dict)
            for marker_id, marker in self.marker_map.items()
        }
        self.camera_pose_on_robot = camera_pose_on_robot
        self._detectors = {
            name: self._make_detector(name)
            for name in set(self._marker_dict_names.values())
        }
        # 向下相容：舊測試/呼叫端取 self._detector（地圖預設字典那顆）。
        self._detector = self._detectors.get(default_dict) or self._make_detector(default_dict)
        self._camera_matrix = np.array(
            [
                [float(camera_info["fx"]), 0.0, float(camera_info["cx"])],
                [0.0, float(camera_info["fy"]), float(camera_info["cy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        # 實體相機經 ChArUco 校正有畸變係數；Gazebo 相機無畸變（缺鍵時為零）。
        self._dist_coeffs = np.asarray(
            camera_info.get("dist_coeffs", np.zeros(5)), dtype=np.float64
        ).ravel()

    @staticmethod
    def _make_detector(dict_name: str) -> ArucoDetector:
        detector = ArucoDetector(dict_name)
        # 我們的 marker 貼圖 quiet zone 佔 10%（黑方塊佔 0.8），白框外緣與
        # 黑方塊外緣的角點距離約為黑方塊周長的 0.044，低於 OpenCV 預設
        # minMarkerDistanceRate=0.05：近距離時白框對灰牆也成為候選四邊形，
        # 兩候選被視為重複、保留周長較大的白框 → 解碼失敗 → 整格偵測不到
        # （實測 spawn x=3.0、距東牆 1.0m 可重現）。調低門檻讓兩候選並存，
        # 黑方塊正常解碼。
        detector.parameters.minMarkerDistanceRate = 0.03
        return detector

    def locate(self, image_bgr: np.ndarray) -> Pose | None:
        """從單張 BGR 影像定位；沒有可用 marker 時回傳 None。"""

        estimates: list[tuple[Pose, float]] = []
        used_ids: list[int] = []
        for marker_id, corner in self._detect_map_markers(image_bgr):
            marker = self.marker_map[marker_id]
            camera_pose = self._solve_camera_pose(marker, corner[0].astype(np.float64))
            if camera_pose is None:
                continue
            estimates.append(
                (
                    self._camera_pose_to_chassis_pose(camera_pose),
                    self._marker_weight(corner[0]),
                )
            )
            used_ids.append(marker_id)

        self.last_used_ids = used_ids
        if not estimates:
            return None
        self.stats["used"] += len(estimates)
        return self._average_poses(estimates)

    def _detect_map_markers(
        self, image_bgr: np.ndarray
    ) -> list[tuple[int, np.ndarray]]:
        """跑地圖用到的每本字典，回傳 (marker_id, corners)；id 需屬於該字典。"""
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            50,
            0.001,
        )
        found: list[tuple[int, np.ndarray]] = []
        for dict_name, detector in self._detectors.items():
            corners, ids, _rejected = detect_markers(
                gray, detector.dictionary, detector.parameters
            )
            if ids is None:
                continue
            for corner, marker_id_array in zip(corners, ids):
                marker_id = int(marker_id_array[0])
                if self._marker_dict_names.get(marker_id) != dict_name:
                    continue
                if self._corners_near_border(corner[0], gray.shape):
                    self.stats["rej_edge"] += 1
                    continue
                cv2.cornerSubPix(gray, corner[0], (3, 3), (-1, -1), criteria)
                found.append((marker_id, corner))
        return found

    def _corners_near_border(
        self, corners: np.ndarray, shape: tuple[int, ...]
    ) -> bool:
        """殘缺 marker 防禦：任何角點貼近畫面邊緣就整顆拒收。"""
        h, w = shape[0], shape[1]
        m = self._edge_margin_px
        xs, ys = corners[:, 0], corners[:, 1]
        return bool(
            (xs < m).any() or (ys < m).any()
            or (xs > w - 1 - m).any() or (ys > h - 1 - m).any()
        )

    def _detect_marker_corners(
        self, image_bgr: np.ndarray
    ) -> tuple[tuple[np.ndarray, ...] | list[np.ndarray], np.ndarray | None]:
        """向下相容的單字典偵測（預設字典）；新程式請用 _detect_map_markers。"""
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = detect_markers(
            gray, self._detector.dictionary, self._detector.parameters
        )
        if ids is not None:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                50,
                0.001,
            )
            for corner in corners:
                cv2.cornerSubPix(gray, corner[0], (3, 3), (-1, -1), criteria)
        return corners, ids

    def _solve_camera_pose(
        self, marker: dict, image_corners: np.ndarray
    ) -> tuple[np.ndarray, float] | None:
        object_points = self._marker_corners_world(marker)
        try:
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                object_points,
                image_corners,
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error:
            n = 0
        if n < 1:
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_corners,
                self._camera_matrix,
                self._dist_coeffs,
            )
            if not ok:
                return None
            rvecs, tvecs = [rvec], [tvec]
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec, self._camera_matrix, self._dist_coeffs
            )
            errs = np.array([np.sqrt(np.mean(np.sum(
                (projected.reshape(-1, 2) - image_corners) ** 2, axis=1)))])

        errs = np.asarray(errs, dtype=np.float64).ravel()
        # 每個候選解過「重投影＋物理」門檻：相機剛性固定在車上，
        # z 高度已知、俯仰/翻滾 ≈ 0。翻錯的解在這裡現形。
        plausible: list[tuple[float, tuple[np.ndarray, float]]] = []
        any_reproj_ok = False
        expected_z = float(self.camera_pose_on_robot[2])
        for i in range(len(errs)):
            err = float(errs[i])
            if err > self._reproj_max_px:
                continue
            any_reproj_ok = True
            world_to_optical, _ = cv2.Rodrigues(rvecs[i])
            optical_to_world = world_to_optical.T
            camera_world = (-optical_to_world @ tvecs[i]).reshape(3)
            body_to_world = optical_to_world @ BODY_FROM_OPTICAL.T
            if abs(float(camera_world[2]) - expected_z) > CAM_Z_TOL_M:
                continue
            pitch = -math.asin(max(-1.0, min(1.0, float(body_to_world[2, 0]))))
            roll = math.atan2(float(body_to_world[2, 1]), float(body_to_world[2, 2]))
            if abs(pitch) > CAM_TILT_TOL_RAD or abs(roll) > CAM_TILT_TOL_RAD:
                continue
            forward_world = body_to_world[:, 0]
            yaw = math.atan2(float(forward_world[1]), float(forward_world[0]))
            plausible.append((err, (camera_world, yaw)))

        if not plausible:
            self.stats["rej_reproj" if not any_reproj_ok else "rej_implausible"] += 1
            return None
        plausible.sort(key=lambda item: item[0])
        if len(plausible) >= 2:
            e1, e2 = plausible[0][0], plausible[1][0]
            # 兩個解都物理可信且誤差比接近 → 真歧義，寧缺勿錯。
            if e2 <= 0.0 or e1 / e2 > self._ambiguity_ratio:
                self.stats["rej_ambiguous"] += 1
                return None
        return plausible[0][1]

    @staticmethod
    def _marker_corners_world(marker: dict) -> np.ndarray:
        # Gazebo 貼圖的 size_m 含 10% quiet zone(黑方塊佔 0.8)；實體印刷的
        # marker 直接量黑框邊長，用 black_size_m 給值時不再乘比例。
        if "black_size_m" in marker:
            size = float(marker["black_size_m"])
        else:
            size = float(marker["size_m"]) * DETECTABLE_ARUCO_SIZE_FRACTION
        half = size / 2.0
        center = np.array(
            [float(marker["x"]), float(marker["y"]), float(marker["z"])],
            dtype=np.float64,
        )
        yaw = float(marker["yaw"])
        right = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=np.float64)
        down = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        return np.array(
            [
                center - right * half - down * half,
                center + right * half - down * half,
                center + right * half + down * half,
                center - right * half + down * half,
            ],
            dtype=np.float64,
        )

    def _camera_pose_to_chassis_pose(self, camera_pose: tuple[np.ndarray, float]) -> Pose:
        camera_world, camera_yaw = camera_pose
        cam_x, cam_y, _cam_z, cam_yaw_on_robot = self.camera_pose_on_robot
        theta = _wrap_pi(camera_yaw - cam_yaw_on_robot)
        chassis_x = (
            float(camera_world[0])
            - math.cos(theta) * cam_x
            + math.sin(theta) * cam_y
        )
        chassis_y = (
            float(camera_world[1])
            - math.sin(theta) * cam_x
            - math.cos(theta) * cam_y
        )
        return Pose(chassis_x, chassis_y, theta)

    @staticmethod
    def _marker_weight(image_corners: np.ndarray) -> float:
        area_px = abs(float(cv2.contourArea(image_corners.astype(np.float32))))
        return max(area_px, 1.0) ** MARKER_AREA_WEIGHT_POWER

    @staticmethod
    def _average_poses(weighted_poses: Iterable[tuple[Pose, float]]) -> Pose:
        pose_list = list(weighted_poses)
        weight_sum = sum(weight for _pose, weight in pose_list)
        x = sum(pose.x * weight for pose, weight in pose_list) / weight_sum
        y = sum(pose.y * weight for pose, weight in pose_list) / weight_sum
        sin_sum = sum(math.sin(pose.theta) * weight for pose, weight in pose_list)
        cos_sum = sum(math.cos(pose.theta) * weight for pose, weight in pose_list)
        theta = math.atan2(sin_sum, cos_sum)
        return Pose(x, y, theta)


@dataclass(frozen=True)
class ArucoFrameResult:
    stamp_s: float
    pose: Pose | None
    marker_ids: tuple[int, ...]

    @property
    def marker_ids_json(self) -> str:
        return json.dumps(
            {"stamp_s": self.stamp_s, "ids": list(self.marker_ids)},
            sort_keys=True,
        )


def process_frame(localizer: ArucoWorldLocalizer, image_bgr: np.ndarray,
                  *, stamp_s: float) -> ArucoFrameResult:
    pose = localizer.locate(image_bgr)
    return ArucoFrameResult(
        stamp_s=float(stamp_s), pose=pose,
        marker_ids=tuple(sorted(int(value) for value in localizer.last_used_ids)),
    )


def load_json(path: Path) -> dict:
    """讀取 JSON 設定檔；供 main 與測試重用。"""

    return json.loads(path.read_text(encoding="utf-8"))


def _quaternion_from_yaw(theta: float) -> tuple[float, float, float, float]:
    half = theta / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main() -> None:
    """啟動 ROS2 節點；ROS import 僅限這層薄包裝。"""

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    class ArucoDetectorNode(Node):
        """ROS topic 包裝：Image(rgb8) 進，每幀出一筆 marker-id 觀察，有定位才出 PoseStamped。"""

        def __init__(self) -> None:
            super().__init__("aruco_detector")
            self.declare_parameter("marker_map_path", str(DEFAULT_MARKER_MAP_PATH))
            self.declare_parameter("camera_info_path", str(DEFAULT_CAMERA_INFO_PATH))
            self.declare_parameter("pose_topic", "/aruco/pose")
            self.declare_parameter("marker_ids_topic", "/aruco/marker_ids")
            self._localizer = ArucoWorldLocalizer(
                load_json(Path(str(self.get_parameter("marker_map_path").value))),
                load_json(Path(str(self.get_parameter("camera_info_path").value))),
                DEFAULT_CAMERA_POSE_ON_ROBOT,
            )
            self._pub = self.create_publisher(
                PoseStamped, str(self.get_parameter("pose_topic").value), 10)
            self._ids_pub = self.create_publisher(
                String, str(self.get_parameter("marker_ids_topic").value), 10)
            # ros_gz bridge 的影像是 best-effort；預設 reliable 訂閱會
            # 因 QoS 不相容而完全收不到 callback（實測無任何錯誤訊息）。
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(Image, "/camera/image_raw", self._on_image,
                                     qos_profile_sensor_data)

        def _on_image(self, msg: Image) -> None:
            if msg.encoding != "rgb8":
                self.get_logger().warning(f"unsupported image encoding: {msg.encoding}")
                return
            image_rgb = self._image_msg_to_rgb_array(msg)
            image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            result = process_frame(self._localizer, image_bgr, stamp_s=stamp_s)
            # 每幀都出一筆 accepted-ID 觀察（即使無定位），供下游取捨。
            self._ids_pub.publish(String(data=result.marker_ids_json))
            if result.pose is None:
                return

            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = "map"
            out.pose.position.x = result.pose.x
            out.pose.position.y = result.pose.y
            qx, qy, qz, qw = _quaternion_from_yaw(result.pose.theta)
            out.pose.orientation.x = qx
            out.pose.orientation.y = qy
            out.pose.orientation.z = qz
            out.pose.orientation.w = qw
            self._pub.publish(out)

        @staticmethod
        def _image_msg_to_rgb_array(msg: Image) -> np.ndarray:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            rows = data.reshape((msg.height, msg.step))
            tight = rows[:, : msg.width * 3]
            return tight.reshape((msg.height, msg.width, 3))

    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
