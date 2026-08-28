"""SG-C 真 ArUco 定位節點的純函式測試（不需 rclpy/相機）。"""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer
from ros2_ws.src.vgr_safety_gate.vgr_safety_gate.aruco_camera_pose import charuco_to_camera_info

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_charuco_to_camera_info_real_file():
    data = json.loads((REPO_ROOT / "ChArUco/camera_intrinsics_640x480.json").read_text())
    info = charuco_to_camera_info(data)
    assert 600 < info["fx"] < 700
    assert 600 < info["fy"] < 700
    assert 280 < info["cx"] < 360
    assert 200 < info["cy"] < 280
    assert info["width"] == 640 and info["height"] == 480
    assert len(info["dist_coeffs"]) == 5


def test_black_size_m_overrides_texture_fraction():
    base = {"id": 1, "x": 2.0, "y": 0.0, "z": 0.3, "yaw": math.pi, "size_m": 0.17}
    with_black = dict(base, black_size_m=0.17)
    corners_frac = ArucoWorldLocalizer._marker_corners_world(base)
    corners_black = ArucoWorldLocalizer._marker_corners_world(with_black)
    # 貼圖比例路徑：邊長 0.17*0.8；black_size_m 路徑：邊長 0.17
    assert np.linalg.norm(corners_frac[1] - corners_frac[0]) == pytest.approx(0.136)
    assert np.linalg.norm(corners_black[1] - corners_black[0]) == pytest.approx(0.17)
    # 中心不變
    assert np.allclose(corners_frac.mean(axis=0), corners_black.mean(axis=0))


def _room_localizer(cam_pose=(0.10, 0.0, 0.10, 0.0)):
    marker_map = json.loads((REPO_ROOT / "config/room_marker_map.json").read_text())
    intrinsics = json.loads((REPO_ROOT / "ChArUco/camera_intrinsics_640x480.json").read_text())
    return ArucoWorldLocalizer(marker_map, charuco_to_camera_info(intrinsics), cam_pose)


def test_solve_camera_pose_roundtrip_synthetic_projection():
    """已知相機位姿投影 marker 角點，localizer 應解回同一位姿（閉環驗證）。"""
    import cv2

    loc = _room_localizer()
    marker = loc.marker_map[1]
    world_corners = ArucoWorldLocalizer._marker_corners_world(marker)

    # 相機位於 (1.0, 0.05, 0.10)、朝 +x 偏 3°，optical frame: x右(=-y_w)、y下(=-z_w)、z前(=+x_w)
    # z 必須與 _room_localizer 宣告的安裝高 0.10 一致：P0 之後解算器會用
    # 安裝高做物理門檻，z 不符的解一律拒收（翻解防禦）。
    yaw = math.radians(3.0)
    cam_pos = np.array([1.0, 0.05, 0.10])
    fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    down = np.array([0.0, 0.0, -1.0])
    r_world_to_optical = np.stack([right, down, fwd])  # rows = optical axes in world
    rvec, _ = cv2.Rodrigues(r_world_to_optical)
    tvec = -r_world_to_optical @ cam_pos

    image_corners, _ = cv2.projectPoints(
        world_corners, rvec, tvec, loc._camera_matrix, loc._dist_coeffs
    )
    camera_pose = loc._solve_camera_pose(marker, image_corners.reshape(4, 2))
    assert camera_pose is not None
    camera_world, camera_yaw = camera_pose
    assert camera_world[0] == pytest.approx(1.0, abs=0.01)
    assert camera_world[1] == pytest.approx(0.05, abs=0.01)
    assert camera_yaw == pytest.approx(yaw, abs=math.radians(0.5))

    # 底盤位姿 = 相機位姿往後退 camera_pose_on_robot.x
    chassis = loc._camera_pose_to_chassis_pose(camera_pose)
    assert chassis.x == pytest.approx(1.0 - 0.10 * math.cos(yaw), abs=0.01)
    assert chassis.theta == pytest.approx(yaw, abs=math.radians(0.5))
