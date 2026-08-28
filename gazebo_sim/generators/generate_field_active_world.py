"""Generate the synthetic public ArUco field world."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from safety_sim.experiments.field_scenarios import ARENA
from gazebo_sim.generators.generate_arena_world import (
    SDF_VERSION, _add_box_model, _add_sun_light, _add_text,
    _add_world_system_plugins, _box_size, _pose,
)

DEFAULT_OUTPUT = "gazebo_sim/worlds/vgr_field_active.world"
DEFAULT_MARKER_MAP = "config/field_marker_map.json"
WALL_THICKNESS_M = 0.05
WALL_HEIGHT_M = 0.40
FLOOR_THICKNESS_M = 0.01
FLOOR_PADDING_M = 0.30


def build_field_active_world(marker_map: dict) -> str:
    if marker_map.get("dictionary") != "DICT_5X5_50":
        raise ValueError("field marker map must use DICT_5X5_50")
    ids = [int(marker["id"]) for marker in marker_map["markers"]]
    if ids != list(range(6)):
        raise ValueError("field marker ids must be exactly 0..5")
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    world = ET.SubElement(sdf, "world", {"name": "vgr_field_active"})
    _add_world_system_plugins(world, with_sensors=True)
    _add_sun_light(world)
    xs, ys = [p[0] for p in ARENA], [p[1] for p in ARENA]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    _add_box_model(
        world, "field_floor", _pose(cx, cy, -FLOOR_THICKNESS_M / 2.0),
        _box_size(max(xs) - min(xs) + 2 * FLOOR_PADDING_M,
                  max(ys) - min(ys) + 2 * FLOOR_PADDING_M,
                  FLOOR_THICKNESS_M),
        rgba="0.85 0.85 0.85 1",
    )
    for index, (start, end) in enumerate(zip(ARENA, (*ARENA[1:], ARENA[0]))):
        dx, dy = end[0] - start[0], end[1] - start[1]
        _add_box_model(
            world, f"field_wall_{index}",
            _pose((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0,
                  WALL_HEIGHT_M / 2.0, yaw=math.atan2(dy, dx)),
            _box_size(math.hypot(dx, dy), WALL_THICKNESS_M, WALL_HEIGHT_M),
            rgba="0.62 0.26 0.2 0.55",
        )
    for marker in marker_map["markers"]:
        include = ET.SubElement(world, "include")
        _add_text(include, "uri", f"model://field_marker_{int(marker['id'])}")
        _add_text(include, "pose", _pose(
            float(marker["x"]), float(marker["y"]), float(marker["z"]),
            yaw=float(marker["yaw"])))
    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode")


def write_field_active_world(output: str, marker_map_path: str) -> None:
    marker_map = json.loads(Path(marker_map_path).read_text(encoding="utf-8"))
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(build_field_active_world(marker_map) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate the synthetic public ArUco field world")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--marker-map", default=DEFAULT_MARKER_MAP)
    args = parser.parse_args(argv)
    write_field_active_world(args.output, args.marker_map)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
