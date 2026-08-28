from pathlib import Path
import xml.etree.ElementTree as ET

from gazebo_sim.generators.generate_nav2_assets import build_nav2_world
from gazebo_sim.generators.generate_nav2_assets import build_nav2_marker_map
from vgr_core.geometry import (
    MAP_PADDING_M,
    MAP_RESOLUTION_M,
    NAV_OBSTACLE,
    build_occupancy_grid,
)


def test_obstacle_is_present_in_world_and_map() -> None:
    world = ET.fromstring(build_nav2_world())
    model = world.find(".//model[@name='nav_obstacle']")
    assert model is not None

    grid = build_occupancy_grid(
        resolution_m=MAP_RESOLUTION_M,
        padding_m=MAP_PADDING_M,
    )
    assert grid.is_occupied(NAV_OBSTACLE.x, NAV_OBSTACLE.y)
    assert not grid.is_occupied(0.5, 0.0)


def test_navigation_obstacle_forces_a_north_or_south_detour() -> None:
    min_x, max_x, min_y, max_y = NAV_OBSTACLE.bounds
    assert min_x < 2.0 < max_x
    assert min_y < 0.0 < max_y
    assert max_y < 0.75
    assert min_y > -0.75


def test_each_detour_corridor_fits_robot_envelope_and_clearance() -> None:
    _min_x, _max_x, min_y, max_y = NAV_OBSTACLE.bounds
    north_corridor = 1.0 - max_y
    south_corridor = min_y - (-1.0)
    required = 2.0 * (0.23 + 0.05)
    assert north_corridor >= required
    assert south_corridor >= required


def test_generated_map_yaml_points_at_sibling_pgm() -> None:
    path = Path("ros2_ws/src/vgr_nav2_bringup/maps/vgr_nav2.yaml")
    text = path.read_text(encoding="utf-8")
    assert "image: vgr_nav2.pgm" in text
    assert "resolution: 0.05" in text
    assert "negate: 0" in text


def test_navigation_world_and_marker_map_include_obstacle_landmarks() -> None:
    world = ET.fromstring(build_nav2_world())
    uris = [item.text for item in world.findall(".//include/uri")]
    marker_map = build_nav2_marker_map()
    ids = {item["id"] for item in marker_map["markers"]}
    assert "model://marker_8" in uris
    assert set(range(8, 14)).issubset(ids)
