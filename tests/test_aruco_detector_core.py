import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from gazebo_sim.generators.generate_robot_sdf import CAMERA_FRONT_X_M, CAMERA_HEIGHT_M


REPO_ROOT = Path(__file__).resolve().parents[1]
MARKER_MAP = json.loads(
    (REPO_ROOT / "gazebo_sim" / "models" / "markers" / "marker_map.json").read_text(
        encoding="utf-8"
    )
)
CAMERA_INFO = json.loads(
    (REPO_ROOT / "gazebo_sim" / "models" / "vgr_diff_drive" / "camera_info.json").read_text(
        encoding="utf-8"
    )
)
CAMERA_POSE_ON_ROBOT = (CAMERA_FRONT_X_M, 0.0, CAMERA_HEIGHT_M, 0.0)


def _aruco_dictionary():
    dictionary_id = cv2.aruco.DICT_6X6_250
    if hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(dictionary_id)
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def _generate_marker(marker_id: int, px: int = 220) -> np.ndarray:
    module_px = px // 10
    marker_px = module_px * 8
    margin_px = (px - marker_px) // 2
    marker = np.full((px, px), 255, dtype=np.uint8)
    aruco = np.zeros((marker_px, marker_px), dtype=np.uint8)
    if hasattr(cv2.aruco, "drawMarker"):
        cv2.aruco.drawMarker(_aruco_dictionary(), marker_id, marker_px, aruco, 1)
    else:
        cv2.aruco.generateImageMarker(
            _aruco_dictionary(), marker_id, marker_px, aruco, 1
        )
    marker[margin_px:margin_px + marker_px, margin_px:margin_px + marker_px] = aruco
    return cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)


def _camera_matrix() -> np.ndarray:
    return np.array(
        [
            [CAMERA_INFO["fx"], 0.0, CAMERA_INFO["cx"]],
            [0.0, CAMERA_INFO["fy"], CAMERA_INFO["cy"]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _marker_corners_world(marker: dict) -> np.ndarray:
    size = marker["size_m"]
    half = size / 2.0
    center = np.array([marker["x"], marker["y"], marker["z"]], dtype=np.float64)
    yaw = marker["yaw"]
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


def _world_to_cv_camera(robot_pose: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    x, y, theta = robot_pose
    cam_x, cam_y, cam_z, cam_yaw = CAMERA_POSE_ON_ROBOT
    yaw = theta + cam_yaw
    camera_world = np.array(
        [
            x + math.cos(theta) * cam_x - math.sin(theta) * cam_y,
            y + math.sin(theta) * cam_x + math.cos(theta) * cam_y,
            cam_z,
        ],
        dtype=np.float64,
    )
    robot_to_world = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    optical_to_robot = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=np.float64,
    )
    camera_to_world = robot_to_world @ optical_to_robot
    world_to_camera = camera_to_world.T
    tvec = -world_to_camera @ camera_world
    return world_to_camera, tvec.reshape(3, 1)


def _render_scene(
    robot_pose: tuple[float, float, float],
    marker_ids: tuple[int, ...],
    *,
    marker_map: dict = MARKER_MAP,
    image_marker_ids: dict[int, int] | None = None,
) -> np.ndarray:
    canvas = np.full(
        (CAMERA_INFO["height"], CAMERA_INFO["width"], 3), 255, dtype=np.uint8
    )
    markers = {marker["id"]: marker for marker in marker_map["markers"]}
    rmat, tvec = _world_to_cv_camera(robot_pose)
    rvec, _ = cv2.Rodrigues(rmat)
    for marker_id in marker_ids:
        marker = markers[marker_id]
        world_corners = _marker_corners_world(marker)
        projected, _ = cv2.projectPoints(
            world_corners, rvec, tvec, _camera_matrix(), np.zeros(5)
        )
        dst = projected.reshape(4, 2).astype(np.float32)
        src_marker_id = image_marker_ids.get(marker_id, marker_id) if image_marker_ids else marker_id
        marker_image = _generate_marker(src_marker_id)
        h, w = marker_image.shape[:2]
        src = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.float32)
        homography = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(
            marker_image,
            homography,
            (CAMERA_INFO["width"], CAMERA_INFO["height"]),
            borderValue=(255, 255, 255),
        )
        mask = cv2.warpPerspective(
            np.full((h, w), 255, dtype=np.uint8),
            homography,
            (CAMERA_INFO["width"], CAMERA_INFO["height"]),
        )
        canvas[mask > 0] = warped[mask > 0]
    return canvas


def _assert_pose_close(actual, expected: tuple[float, float, float]) -> None:
    assert actual is not None
    assert actual.x == pytest.approx(expected[0], abs=0.05)
    assert actual.y == pytest.approx(expected[1], abs=0.05)
    assert _wrap_pi(actual.theta - expected[2]) == pytest.approx(0.0, abs=math.radians(3.0))


def test_locate_recovers_robot_pose_facing_marker():
    from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer

    pose = (0.80, -1.0 / 3.0, math.pi)
    image = _render_scene(pose, (2,))
    localizer = ArucoWorldLocalizer(MARKER_MAP, CAMERA_INFO, CAMERA_POSE_ON_ROBOT)

    _assert_pose_close(localizer.locate(image), pose)


def test_locate_recovers_robot_pose_from_30_degree_oblique_view():
    from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer

    pose = (0.85, -0.15, math.radians(210.0))
    image = _render_scene(pose, (2,))
    localizer = ArucoWorldLocalizer(MARKER_MAP, CAMERA_INFO, CAMERA_POSE_ON_ROBOT)

    _assert_pose_close(localizer.locate(image), pose)


def test_locate_returns_none_for_blank_image():
    from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer

    image = np.full((CAMERA_INFO["height"], CAMERA_INFO["width"], 3), 255, dtype=np.uint8)
    localizer = ArucoWorldLocalizer(MARKER_MAP, CAMERA_INFO, CAMERA_POSE_ON_ROBOT)

    assert localizer.locate(image) is None


def test_locate_returns_none_when_detected_marker_id_is_not_in_map():
    from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer

    pose = (0.80, -1.0 / 3.0, math.pi)
    image = _render_scene(pose, (2,), image_marker_ids={2: 42})
    localizer = ArucoWorldLocalizer(MARKER_MAP, CAMERA_INFO, CAMERA_POSE_ON_ROBOT)

    assert localizer.locate(image) is None


def test_locate_averages_two_visible_markers_without_losing_pose_accuracy():
    from gazebo_sim.nodes.aruco_detector import ArucoWorldLocalizer

    pose = (0.80, 0.0, math.pi)
    image = _render_scene(pose, (2, 3))
    localizer = ArucoWorldLocalizer(MARKER_MAP, CAMERA_INFO, CAMERA_POSE_ON_ROBOT)

    _assert_pose_close(localizer.locate(image), pose)
