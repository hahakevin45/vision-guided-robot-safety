"""survey_marker 測繪數學的合成投影閉環測試。"""
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer
from tools.survey_marker import _local_corners, survey_once

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_survey_math_recovers_known_transform(monkeypatch):
    """已知兩張 marker 的世界位姿，從合成角點反解新 marker 位姿應一致。"""
    known = {"id": 1, "x": 2.0, "y": 0.0, "z": 0.30, "yaw": math.pi,
             "size_m": 0.17, "black_size_m": 0.17}
    # 新 marker：右牆（−y 側）、面朝 +y、中心 (1.5, -0.6, 0.10)
    new_truth = {"x": 1.5, "y": -0.6, "z": 0.10, "yaw": 0.0,
                 "size_m": 0.13, "black_size_m": 0.13}

    intr = json.loads((REPO_ROOT / "ChArUco/camera_intrinsics_640x480.json").read_text())
    cam_mtx = np.array(intr["camera_matrix"], dtype=np.float64)
    dist = np.array(intr["dist_coeffs"], dtype=np.float64).ravel()

    # 相機在 (0.8, 0.05, 0.25)、朝 +x 偏右 15°（兩張都在視野）
    yaw_c = math.radians(-15.0)
    cam_pos = np.array([0.8, 0.05, 0.25])
    fwd = np.array([math.cos(yaw_c), math.sin(yaw_c), 0.0])
    right = np.array([math.sin(yaw_c), -math.cos(yaw_c), 0.0])
    down = np.array([0.0, 0.0, -1.0])
    r_wo = np.stack([right, down, fwd])
    rvec, _ = cv2.Rodrigues(r_wo)
    tvec = -r_wo @ cam_pos

    known_world = ArucoWorldLocalizer._marker_corners_world(known)
    new_world = ArucoWorldLocalizer._marker_corners_world(
        {"id": 4, **new_truth}
    )
    img_known, _ = cv2.projectPoints(known_world, rvec, tvec, cam_mtx, dist)
    img_new, _ = cv2.projectPoints(new_world, rvec, tvec, cam_mtx, dist)

    # 打樁 _detect：直接回合成角點
    corners = {(known["id"]): img_known.reshape(4, 2), 4: img_new.reshape(4, 2)}
    monkeypatch.setattr(
        "tools.survey_marker._detect",
        lambda gray, dict_name, wanted_id: corners.get(wanted_id),
    )
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    result = survey_once(frame, known, "DICT_5X5_50", 4, 0.13 / 2.0, cam_mtx, dist)
    assert result is not None
    x, y, z, yaw = result
    assert x == pytest.approx(new_truth["x"], abs=0.01)
    assert y == pytest.approx(new_truth["y"], abs=0.01)
    assert z == pytest.approx(new_truth["z"], abs=0.01)
    assert yaw == pytest.approx(new_truth["yaw"], abs=math.radians(1.0))


def test_local_corners_edge_length():
    c = _local_corners(0.065)
    assert np.linalg.norm(c[1] - c[0]) == pytest.approx(0.13)
