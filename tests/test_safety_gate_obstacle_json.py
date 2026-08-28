"""Gazebo safety-gate wrapper plumbing: obstacle JSON parsing and /plan frames.

These helpers are the only ROS-free surface of the wrapper; the node itself
delegates to them so the parse/reject behavior is testable without rclpy.
"""
import math

import pytest

from gazebo_sim.nodes.safety_gate import parse_obstacles_json, plan_points_from_path
from vgr_core.safety import Circle


def test_empty_json_yields_no_obstacles():
    assert parse_obstacles_json("") == ()
    assert parse_obstacles_json("   ") == ()


def test_valid_list_parses_circles():
    circles = parse_obstacles_json(
        '[{"x": 2.0, "y": 0.0, "radius": 0.2}, {"x": -1.0, "y": 0.5, "radius": 0.1}]'
    )
    assert circles == (Circle(2.0, 0.0, 0.20), Circle(-1.0, 0.5, 0.10))


def test_non_list_rejected():
    with pytest.raises(ValueError):
        parse_obstacles_json('{"x": 1.0}')


def test_missing_or_non_finite_fields_rejected():
    for bad in ('[{"x": 1.0, "y": 0.0}]',
                '[{"x": 1.0, "y": 0.0, "radius": "big"}]',
                '[{"x": 1.0, "y": 0.0, "radius": null}]',
                '[{"x": 1.0, "y": 0.0, "radius": NaN}]',
                '[{"x": 1.0, "y": 0.0, "radius": 0.2}, 42]'):
        with pytest.raises(ValueError):
            parse_obstacles_json(bad)


def test_non_positive_radius_rejected():
    with pytest.raises(ValueError):
        parse_obstacles_json('[{"x": 1.0, "y": 0.0, "radius": 0.0}]')
    with pytest.raises(ValueError):
        parse_obstacles_json('[{"x": 1.0, "y": 0.0, "radius": -0.2}]')


def test_invalid_json_rejected():
    with pytest.raises(ValueError):
        parse_obstacles_json("not json")


def test_map_frame_plan_accepted():
    points = plan_points_from_path("map", ((0.1, 0.0), (0.2, 0.5)))
    assert points == ((0.1, 0.0), (0.2, 0.5))


def test_non_map_frame_plan_rejected():
    assert plan_points_from_path("odom", ((0.1, 0.0),)) is None
    assert plan_points_from_path("", ((0.1, 0.0),)) is None


def test_empty_plan_in_map_frame_is_empty_tuple():
    assert plan_points_from_path("map", ()) == ()


def test_box_json_parses_to_box2d():
    from vgr_core.geometry.arena_geometry import Box2D

    boxes = parse_obstacles_json(
        '[{"type": "box", "x": 2.0, "y": 0.0, "size_x": 0.4, "size_y": 0.6}]')
    assert boxes == (Box2D(2.0, 0.0, 0.40, 0.60),)


def test_box_missing_or_invalid_size_rejected():
    for bad in ('[{"type": "box", "x": 2.0, "y": 0.0}]',
                '[{"type": "box", "x": 2.0, "y": 0.0, "size_x": 0.0, "size_y": 0.6}]',
                '[{"type": "box", "x": 2.0, "y": 0.0, "size_x": -0.4, "size_y": 0.6}]'):
        with pytest.raises(ValueError):
            parse_obstacles_json(bad)


def test_unknown_type_rejected():
    with pytest.raises(ValueError):
        parse_obstacles_json('[{"type": "triangle", "x": 1.0, "y": 0.0}]')


def test_circle_without_type_still_parses():
    # 舊格式相容：無 type → circle
    circles = parse_obstacles_json('[{"x": 2.0, "y": 0.0, "radius": 0.2}]')
    assert circles == (Circle(2.0, 0.0, 0.20),)
