"""ArUco camera localization node: V4L2 image → `/aruco/pose`.

The node optionally rotates a mounted camera image, uses the tested
`ArucoWorldLocalizer` solvePnP implementation with a configurable synthetic or
deployment marker map, and publishes the chassis pose in the map frame.
Measurement timestamps are captured when `cap.read()` returns so processing
latency remains visible through `pose_age`. The `/aruco/set_dropout` service is
kept for controlled disconnect drills. `black_size_m` explicitly describes the
detected black marker edge, while camera intrinsics come from a ChArUco
calibration file.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path


def charuco_to_camera_info(data: dict) -> dict:
    """把 ChArUco 校正 JSON 轉成 ArucoWorldLocalizer 的 camera_info dict。"""
    m = data["camera_matrix"]
    return {
        "fx": float(m[0][0]),
        "fy": float(m[1][1]),
        "cx": float(m[0][2]),
        "cy": float(m[1][2]),
        "width": int(data["image_size"]["width"]),
        "height": int(data["image_size"]["height"]),
        "dist_coeffs": [float(v) for row in data["dist_coeffs"] for v in (row if isinstance(row, list) else [row])],
    }


def main() -> None:
    import cv2
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from std_srvs.srv import SetBool

    from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer, _quaternion_from_yaw
    from vgr_driver.vision.camera_orientation import upright

    class ArucoCameraPoseNode(Node):
        def __init__(self) -> None:
            super().__init__("aruco_camera_pose")
            self.declare_parameter("camera_index", 0)
            self.declare_parameter("intrinsics_path", "ChArUco/camera_intrinsics_640x480.json")
            self.declare_parameter("marker_map_path", "config/room_marker_map.json")
            # base_link 到相機的 (x, y, z, yaw)；z 不影響 2D 位姿，x 是鏡頭
            # 到輪軸中心的前向距離（實測後由 launch 覆寫）。
            self.declare_parameter("camera_pose_on_robot", [0.10, 0.0, 0.10, 0.0])
            self.declare_parameter("pose_topic", "/aruco/pose")
            self.declare_parameter("log_period_s", 5.0)
            # 錄影用影格傾印（預設關）：把轉正後的影格存 JPEG，檔名帶
            # 擷取時戳，供事後與 /aruco/pose 對齊回播。
            self.declare_parameter("frame_dump_dir", "")
            self.declare_parameter("frame_dump_hz", 5.0)

            camera_index = int(self.get_parameter("camera_index").value)
            intrinsics_path = Path(str(self.get_parameter("intrinsics_path").value))
            marker_map_path = Path(str(self.get_parameter("marker_map_path").value))
            cam_pose = tuple(float(v) for v in self.get_parameter("camera_pose_on_robot").value)
            pose_topic = str(self.get_parameter("pose_topic").value)
            self._log_period_s = float(self.get_parameter("log_period_s").value)

            with intrinsics_path.open("r", encoding="utf-8") as f:
                camera_info = charuco_to_camera_info(json.load(f))
            with marker_map_path.open("r", encoding="utf-8") as f:
                marker_map = json.load(f)

            self._localizer = ArucoWorldLocalizer(marker_map, camera_info, cam_pose)

            dump_dir = str(self.get_parameter("frame_dump_dir").value)
            self._dump_dir = Path(dump_dir) if dump_dir else None
            self._dump_interval_s = 1.0 / max(float(self.get_parameter("frame_dump_hz").value), 0.1)
            self._last_dump_mono = 0.0
            if self._dump_dir is not None:
                self._dump_dir.mkdir(parents=True, exist_ok=True)

            # Pi 上 OpenCV 預設 GStreamer backend 會卡住，必須明給 V4L2。
            self._cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_info["width"])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_info["height"])
            if not self._cap.isOpened():
                raise SystemExit(f"打不開相機 index {camera_index}")

            self._dropout = False
            self._frames = 0
            self._published = 0
            self._pub = self.create_publisher(PoseStamped, pose_topic, 10)
            self.create_service(SetBool, "/aruco/set_dropout", self._on_dropout)
            self.create_timer(self._log_period_s, self._log_stats)

            self._stop = threading.Event()
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
            self.get_logger().info(
                f"aruco_camera_pose up: camera={camera_index} map={marker_map_path} "
                f"markers={sorted(self._localizer.marker_map)} cam_pose={cam_pose}"
            )

        def _capture_loop(self) -> None:
            while not self._stop.is_set() and rclpy.ok():
                ok, frame = self._cap.read()
                stamp = self.get_clock().now().to_msg()  # 擷取當下的時戳
                if not ok:
                    self.get_logger().warning("讀取影格失敗")
                    continue
                self._frames += 1
                up = upright(frame)
                if self._dump_dir is not None:
                    now_mono = time.monotonic()
                    if now_mono - self._last_dump_mono >= self._dump_interval_s:
                        self._last_dump_mono = now_mono
                        name = f"{stamp.sec}.{stamp.nanosec:09d}.jpg"
                        cv2.imwrite(str(self._dump_dir / name), up)
                if self._dropout:
                    continue
                pose = self._localizer.locate(up)
                if pose is None:
                    continue
                out = PoseStamped()
                out.header.stamp = stamp
                out.header.frame_id = "map"
                out.pose.position.x = pose.x
                out.pose.position.y = pose.y
                qx, qy, qz, qw = _quaternion_from_yaw(pose.theta)
                out.pose.orientation.x = qx
                out.pose.orientation.y = qy
                out.pose.orientation.z = qz
                out.pose.orientation.w = qw
                try:
                    self._pub.publish(out)
                except Exception:
                    # context 已在關閉中（Ctrl-C/SIGTERM 與擷取執行緒的競態）。
                    if self._stop.is_set() or not rclpy.ok():
                        return
                    raise
                self._published += 1

        def _on_dropout(self, request, response):
            self._dropout = bool(request.data)
            self.get_logger().warn(f"aruco pose dropout={self._dropout}")
            response.success = True
            response.message = f"dropout={self._dropout}"
            return response

        def _log_stats(self) -> None:
            hz_cap = self._frames / self._log_period_s
            hz_pub = self._published / self._log_period_s
            self._frames = 0
            self._published = 0
            stats = getattr(self._localizer, "stats", {})
            used_ids = getattr(self._localizer, "last_used_ids", [])
            self.get_logger().info(
                f"capture {hz_cap:.1f} Hz, pose {hz_pub:.1f} Hz, "
                f"ids={used_ids} gates={stats}"
            )

        def shutdown(self) -> None:
            self._stop.set()
            self._thread.join(timeout=2.0)
            self._cap.release()

    rclpy.init()
    node = ArucoCameraPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
