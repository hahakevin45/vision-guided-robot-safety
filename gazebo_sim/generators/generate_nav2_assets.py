"""Generate the obstacle navigation world and matching Nav2 occupancy map."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from gazebo_sim.generators.generate_arena_world import (
    MARKER_CENTER_Z_M,
    MARKER_SIZE_M,
    MARKER_THICKNESS_M,
    build_arena_world,
    build_marker_map,
)
from gazebo_sim.generators.generate_marker_assets import generate_marker_assets
from vgr_core.geometry import (
    MAP_PADDING_M,
    MAP_RESOLUTION_M,
    NAV_OBSTACLE,
    build_occupancy_grid,
)


DEFAULT_WORLD = Path("gazebo_sim/worlds/vgr_nav2.world")
DEFAULT_MAP_DIR = Path("ros2_ws/src/vgr_nav2_bringup/maps")
DEFAULT_MARKER_MAP = Path("gazebo_sim/models/markers/nav2_marker_map.json")


def _add_text(parent: ET.Element, tag: str, text: object) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = str(text)
    return child


def build_nav2_world() -> str:
    root = ET.fromstring(build_arena_world("vgr_nav2", with_sensors=True))
    world = root.find("world")
    if world is None:
        raise ValueError("generated arena has no world element")

    model = ET.SubElement(world, "model", {"name": "nav_obstacle"})
    _add_text(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": "nav_obstacle_link"})
    _add_text(link, "pose", f"{NAV_OBSTACLE.x} {NAV_OBSTACLE.y} 0.25 0 0 0")
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, {"name": f"nav_obstacle_{kind}"})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        _add_text(box, "size", f"{NAV_OBSTACLE.size_x} {NAV_OBSTACLE.size_y} 0.5")
        if kind == "visual":
            material = ET.SubElement(element, "material")
            _add_text(material, "ambient", "0.15 0.25 0.75 1")
            _add_text(material, "diffuse", "0.15 0.25 0.75 1")
    for marker in build_nav2_marker_map()["markers"]:
        if int(marker["id"]) < 8:
            continue
        include = ET.SubElement(world, "include")
        _add_text(include, "uri", f"model://marker_{marker['id']}")
        _add_text(
            include,
            "pose",
            f"{marker['x']} {marker['y']} {marker['z']} 0 0 {marker['yaw']}",
        )
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def build_nav2_marker_map() -> dict[str, object]:
    result = build_marker_map()
    min_x, max_x, min_y, max_y = NAV_OBSTACLE.bounds
    offset = MARKER_THICKNESS_M / 2.0
    z = max(MARKER_CENTER_Z_M, 0.18)
    obstacle_markers = [
        {"id": 8, "x": min_x - offset, "y": -0.18, "z": z, "yaw": math.pi, "size_m": MARKER_SIZE_M},
        {"id": 9, "x": min_x - offset, "y": 0.18, "z": z, "yaw": math.pi, "size_m": MARKER_SIZE_M},
        {"id": 10, "x": max_x + offset, "y": -0.18, "z": z, "yaw": 0.0, "size_m": MARKER_SIZE_M},
        {"id": 11, "x": max_x + offset, "y": 0.18, "z": z, "yaw": 0.0, "size_m": MARKER_SIZE_M},
        {"id": 12, "x": NAV_OBSTACLE.x, "y": max_y + offset, "z": z, "yaw": math.pi / 2.0, "size_m": MARKER_SIZE_M},
        {"id": 13, "x": NAV_OBSTACLE.x, "y": min_y - offset, "z": z, "yaw": -math.pi / 2.0, "size_m": MARKER_SIZE_M},
    ]
    result["markers"] = [*result["markers"], *obstacle_markers]
    return result


def write_assets(
    world_path: Path = DEFAULT_WORLD,
    map_dir: Path = DEFAULT_MAP_DIR,
    marker_map_path: Path = DEFAULT_MARKER_MAP,
) -> tuple[Path, Path, Path, Path]:
    grid = build_occupancy_grid()
    world_path.parent.mkdir(parents=True, exist_ok=True)
    map_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = map_dir / "vgr_nav2.pgm"
    yaml_path = map_dir / "vgr_nav2.yaml"
    marker_map_path.parent.mkdir(parents=True, exist_ok=True)
    generate_marker_assets(tuple(range(8, 14)))
    world_path.write_text(build_nav2_world() + "\n", encoding="utf-8")
    pgm_path.write_text(grid.to_pgm(), encoding="ascii")
    yaml_path.write_text(
        "image: vgr_nav2.pgm\n"
        f"resolution: {MAP_RESOLUTION_M}\n"
        f"origin: [{grid.origin_x}, {grid.origin_y}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n",
        encoding="utf-8",
    )
    marker_map_path.write_text(
        json.dumps(build_nav2_marker_map(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return world_path, pgm_path, yaml_path, marker_map_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_MAP_DIR)
    args = parser.parse_args()
    for path in write_assets(args.world, args.map_dir):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
