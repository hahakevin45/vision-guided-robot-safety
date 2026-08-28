"""Box2D 障礙支援測試（矩形障礙：SAPF/CBF/world/場景/產生器）。

動機：圓柱接觸面只有切點，「一滑就過」；矩形有平直邊與角點，
繞行需沿邊走、過角切換——更嚴格的繞行測試。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from safety_sim.filters.cbf import CbfFilter
from safety_sim.filters.safe_apf_new import SafeApfNewFilter
from safety_sim.scenarios.sapf import SAPF_OBSTACLE, S8_GOAL
from safety_sim.types import Observation, Pose, StaticInfo, Twist
from safety_sim.world import World
from vgr_core.geometry.arena_geometry import Box2D, box_edges
from vgr_core.motion import DiffDriveParams

BOX = Box2D(x=2.0, y=0.0, size_x=0.40, size_y=0.40)


def _static() -> StaticInfo:
    return StaticInfo(
        params=DiffDriveParams(),
        robot_radius_m=0.23,
        geofence=((0, -1), (4, -1), (4, 1), (0, 1)),
        max_v_mps=0.15,
        max_omega_rad_s=1.5,
    )


def _obs(pose: Pose, obstacles=(BOX,), goal=S8_GOAL, drift=0.0) -> Observation:
    return Observation(
        pose=pose, pose_age_s=0.05,
        wheel_feedback=(0.0, 0.0),
        obstacles=obstacles,
        pose_drift_m=drift,
        goal=goal, goal_age_s=0.05,
    )


# --- box_edges：4 條邊、外向法線 ---

def test_box_edges_four_unit_normal_edges():
    edges = box_edges(BOX)
    assert len(edges) == 4
    for x1, y1, x2, y2, nx, ny in edges:
        assert math.isclose(math.hypot(nx, ny), 1.0)


def test_box_edges_right_edge_normal_points_plus_x():
    edges = box_edges(BOX)
    right = next(e for e in edges if abs(e[0] - 2.2) < 1e-9)
    assert right[4] > 0.9  # nx ≈ +1


def test_box_edges_top_edge_normal_points_plus_y():
    edges = box_edges(BOX)
    top = next(e for e in edges if abs(e[1] - 0.2) < 1e-9)
    assert top[5] > 0.9  # ny ≈ +1


# --- SAPF：Box2D → 4 個障礙樣本；箱內 STOP ---

def test_sapf_box_obstacle_blocks_straight_path():
    filt = SafeApfNewFilter()
    filt.reset(_static())
    # 車在箱前 (1.45, 0)：距箱面 0.35 < 0.52（vortex 生效區）→ 有切向轉向
    decision = filt.filter(Twist(0.15, 0.0), _obs(Pose(1.45, 0.0, 0.0)), 0.0, 0.05)
    assert decision.mode == "MODIFIED"
    assert abs(decision.cmd.omega) > 0.01  # 有轉向（沿邊滑）


def test_sapf_box_field_depends_on_nearest_surface_not_opposite_face():
    filt = SafeApfNewFilter()
    filt.reset(_static())
    pose = Pose(1.45, 0.0, 0.0)
    shallow = Box2D(x=2.0, y=0.0, size_x=0.40, size_y=0.60)
    deep = Box2D(x=2.8, y=0.0, size_x=2.00, size_y=0.60)

    shallow_decision = filt.filter(
        Twist(0.15, 0.0), _obs(pose, obstacles=(shallow,)), 0.0, 0.05)
    deep_decision = filt.filter(
        Twist(0.15, 0.0), _obs(pose, obstacles=(deep,)), 0.0, 0.05)

    assert shallow_decision.debug["gradient_x"] == pytest.approx(
        deep_decision.debug["gradient_x"])
    assert shallow_decision.debug["gradient_y"] == pytest.approx(
        deep_decision.debug["gradient_y"])

def test_sapf_box_inside_obstacle_stops():
    filt = SafeApfNewFilter()
    filt.reset(_static())
    decision = filt.filter(Twist(0.15, 0.0),
                           _obs(Pose(2.0, 0.0, 0.0)), 0.0, 0.05)
    assert decision.mode == "STOP"


def test_sapf_box_far_away_no_effect():
    filt = SafeApfNewFilter()
    filt.reset(_static())
    # 距箱 > Q*：斥力為零，命令 = 吸引場（直行）
    decision = filt.filter(Twist(0.15, 0.0), _obs(Pose(0.7, 0.0, 0.0)), 0.0, 0.05)
    assert decision.mode == "MODIFIED"
    assert abs(decision.cmd.omega) < 1e-9


# --- CBF：Box2D → 4 邊 barrier；箱內不可行 ---

def test_cbf_box_barrier_slows_approach():
    filt = CbfFilter(buffer_m=0.05)
    filt.reset(_static())
    decision = filt.filter(Twist(0.15, 0.0), _obs(Pose(1.5, 0.0, 0.0)), 0.0, 0.05)
    assert decision.mode == "MODIFIED"
    assert decision.cmd.v < 0.15  # 降速


def test_cbf_box_stops_when_inside():
    filt = CbfFilter(buffer_m=0.05)
    filt.reset(_static())
    decision = filt.filter(Twist(0.15, 0.0), _obs(Pose(2.0, 0.0, 0.0)), 0.0, 0.05)
    assert decision.mode == "STOP"


# --- World：box 距離 ---

def test_world_min_clearance_box_front():
    w = World(geofence=((0, -1), (4, -1), (4, 1), (0, 1)),
              obstacles=(BOX,), robot_radius_m=0.23)
    # 車心 (1.7, 0)：距箱面 2.0−0.2−1.7 = 0.1 → footprint 0.1−0.23 = −0.13
    assert w.min_clearance(Pose(1.7, 0.0, 0.0)) == pytest.approx(-0.13)


def test_world_min_clearance_box_side():
    w = World(geofence=((0, -1), (4, -1), (4, 1), (0, 1)),
              obstacles=(BOX,), robot_radius_m=0.23)
    # 車心 (2.0, 0.55)：距箱面 0.55−0.2 = 0.35 → 0.12
    assert w.min_clearance(Pose(2.0, 0.55, 0.0)) == pytest.approx(0.12)


def test_world_collides_inside_box():
    w = World(geofence=((0, -1), (4, -1), (4, 1), (0, 1)),
              obstacles=(BOX,), robot_radius_m=0.23)
    assert w.collided(Pose(2.0, 0.0, 0.0))


# --- S8 場景：SAPF_OBSTACLE 為 Box2D ---

def test_s8_obstacle_is_box2d():
    assert isinstance(SAPF_OBSTACLE, Box2D)
    assert SAPF_OBSTACLE.x == 2.0
    assert SAPF_OBSTACLE.size_x == pytest.approx(0.40)
    assert SAPF_OBSTACLE.size_y == pytest.approx(0.40)
