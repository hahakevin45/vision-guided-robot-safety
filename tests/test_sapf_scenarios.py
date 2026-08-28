"""S8 / GS3 asset contracts: scenario registry, geometry, and world generator.

The Gazebo cylinder and the deterministic World must both read the same
`SAPF_OBSTACLE` constant, and the straight start-goal line must intersect the
inflated obstacle so reaching the goal proves a detour happened.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from gazebo_sim.generators.generate_sapf_world import (
    CYLINDER_CENTER_Z_M,
    build_sapf_world,
    write_sapf_world,
)
from safety_sim.scenarios import get_scenario
from safety_sim.scenarios.sapf import SAPF_OBSTACLE, S8_GOAL
from safety_sim.scenarios.basic import ARENA


def test_s8_registered():
    scenario = get_scenario("S8")
    assert scenario.name == "S8"
    # 矩形角落 vortex 衝突使過角需 ~50s（圓柱 ~20s）
    assert scenario.duration_s == pytest.approx(120.0)


def test_s8_straight_path_necessarily_intersects_obstacle():
    # start (0.5, 0) -> goal (3.2, 0) along y=0; inflated box (robot 0.23 m
    # body) must cover y=0 so the straight segment intersects the obstacle.
    inflated = 0.23
    lo, hi = SAPF_OBSTACLE.x - SAPF_OBSTACLE.size_x / 2.0 - inflated, \
              SAPF_OBSTACLE.x + SAPF_OBSTACLE.size_x / 2.0 + inflated
    assert lo < 3.2 and hi > 0.5  # x-range overlap with [0.5, 3.2]
    assert abs(SAPF_OBSTACLE.y - 0.0) < SAPF_OBSTACLE.size_y / 2.0 + inflated


def test_s8_world_contains_goal_and_obstacle():
    world = get_scenario("S8").make_world()
    assert world.goal == S8_GOAL
    assert world.obstacles == (SAPF_OBSTACLE,)
    assert world.geofence == ARENA


def test_s1_and_s2_keep_original_contracts_with_goals():
    s1 = get_scenario("S1")
    s2 = get_scenario("S2")
    # nav / duration / expectation unchanged from the original definitions
    assert s1.duration_s == pytest.approx(30.0)
    assert s2.duration_s == pytest.approx(25.0)
    assert s2.fault_t0 == pytest.approx(3.0)
    assert s1.make_world().goal == (4.0, 0.0)
    assert s2.make_world().goal == (3.0, 0.0)
    assert s1.make_world().obstacles == ()
    assert s2.make_world().obstacles == ()


def test_generated_world_box_matches_obstacle_constant():
    root = ET.fromstring(build_sapf_world())
    model = root.find("./world/model[@name='sapf_obstacle']")
    assert model is not None
    pose = model.find("./link/pose").text.split()
    assert float(pose[0]) == pytest.approx(SAPF_OBSTACLE.x)
    assert float(pose[1]) == pytest.approx(SAPF_OBSTACLE.y)
    assert float(pose[2]) == pytest.approx(CYLINDER_CENTER_Z_M)
    box = model.find("./link/collision/geometry/box")
    assert box is not None
    sx, sy = (float(v) for v in box.find("size").text.split()[:2])
    assert sx == pytest.approx(SAPF_OBSTACLE.size_x)
    assert sy == pytest.approx(SAPF_OBSTACLE.size_y)


def test_checked_in_sapf_world_matches_obstacle_constant():
    root = ET.parse(Path("gazebo_sim/worlds/vgr_sapf.world")).getroot()
    size = root.find(
        "./world/model[@name='sapf_obstacle']/link/collision/geometry/box/size")
    assert size is not None
    sx, sy = (float(value) for value in size.text.split()[:2])
    assert sx == pytest.approx(SAPF_OBSTACLE.size_x)
    assert sy == pytest.approx(SAPF_OBSTACLE.size_y)


def test_generated_world_is_valid_sdf():
    root = ET.fromstring(build_sapf_world())
    assert root.tag == "sdf"
    assert root.find("./world") is not None
    assert root.find("./world/model[@name='floor']") is not None


def test_write_sapf_world_outputs_parseable_file(tmp_path):
    out = tmp_path / "vgr_sapf.world"
    write_sapf_world(out)
    text = out.read_text(encoding="utf-8")
    assert "sapf_obstacle" in text
    assert ET.fromstring(text) is not None
