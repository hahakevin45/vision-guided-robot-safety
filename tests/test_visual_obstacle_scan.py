"""Visual marker detection representations shared by SAPF and Nav2."""
from __future__ import annotations

import json
import math

import pytest

from gazebo_sim.nodes.visual_obstacle_scan import (
    box_boundary_points,
    box_ray_intersection,
    boxes_to_json,
    box_to_laser_scan,
    detection_latched,
    laser_scan_hits_world,
)
from vgr_core.geometry.arena_geometry import Box2D

BOX = Box2D(x=2.0, y=0.0, size_x=0.40, size_y=0.60)


def test_ray_hits_box_front_face():
    # 車在 (1.0, 0) 朝 +x：射線應擊中箱前表面 x=1.8
    dist = box_ray_intersection((1.0, 0.0), 0.0, BOX)
    assert dist is not None
    assert dist == pytest.approx(0.8)


def test_ray_misses_box():
    # 車在 (1.0, 0) 朝 +y（平行箱側）……實際朝斜上方不穿箱
    dist = box_ray_intersection((1.0, 0.0), math.pi / 2.0, BOX)
    assert dist is None


def test_ray_hits_corner_diagonal():
    # 車在 (1.0, -1.0) 朝 (1,1) 方向（45°）：擊中箱右下角附近
    dist = box_ray_intersection((1.0, -1.0), math.pi / 4.0, BOX)
    assert dist is not None
    assert dist > 0.0


def test_ray_from_inside_box_is_none():
    dist = box_ray_intersection((2.0, 0.0), 0.0, BOX)
    assert dist is None


def test_laser_scan_front_cone_sees_box():
    # 車在 (1.0, 0) 朝 +x；前方 ±60° 錐，1°/ray，max_range 3.0
    ranges, angle_min, angle_inc = box_to_laser_scan(
        (1.0, 0.0, 0.0), BOX,
        angle_min=-math.pi / 3, angle_max=math.pi / 3,
        angle_increment=math.radians(1.0), max_range=3.0,
    )
    assert angle_min == pytest.approx(-math.pi / 3)
    assert angle_inc == pytest.approx(math.radians(1.0))
    assert len(ranges) == 121
    # 正前方（index 60）距離 ≈ 0.8（前表面）
    center = ranges[60]
    assert center == pytest.approx(0.8, abs=0.05)
    # 邊緣角度應為 inf（箱不在 ±60° 邊緣）
    assert math.isinf(ranges[0])
    assert math.isinf(ranges[-1])


def test_laser_scan_robot_heading_rotation():
    # 車在 (2.0, 1.0) 朝 -y（θ=-π/2）：箱在車的「前方」（-y 方向）
    ranges, _, _ = box_to_laser_scan(
        (2.0, 1.0, -math.pi / 2.0), BOX,
        angle_min=-math.pi / 3, angle_max=math.pi / 3,
        angle_increment=math.radians(1.0), max_range=3.0,
    )
    assert len(ranges) == 121
    # 正前方擊中箱下表面（y=0.3 → 距車 0.7）
    assert ranges[60] == pytest.approx(0.7, abs=0.05)


def test_laser_scan_hits_transform_to_odom():
    points = laser_scan_hits_world(
        (1.0, 2.0, math.pi / 2.0),
        [1.0, math.inf, 2.0],
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
        min_range=0.05,
        max_range=3.0,
    )

    assert points[0] == pytest.approx((1.0, 3.0, 0.0))
    assert points[1] == pytest.approx((1.0, 0.0, 0.0))


def test_marker_detection_is_retained_after_box_leaves_fov():
    assert detection_latched(False, [math.inf, math.inf]) is False
    assert detection_latched(False, [math.inf, 1.0]) is True
    assert detection_latched(True, [math.inf, math.inf]) is True


def test_box_boundary_points_expose_full_marker_geometry():
    points = box_boundary_points(BOX, spacing_m=0.1)
    xy = {(round(x, 6), round(y, 6)) for x, y, _ in points}

    for corner in ((1.8, -0.3), (1.8, 0.3), (2.2, -0.3), (2.2, 0.3)):
        assert corner in xy
    assert all(x in (1.8, 2.2) or y in (-0.3, 0.3) for x, y in xy)


def test_box_json_preserves_marker_pose_and_size():
    assert json.loads(boxes_to_json([BOX])) == [{
        "type": "box",
        "x": 2.0,
        "y": 0.0,
        "size_x": 0.4,
        "size_y": 0.6,
    }]
