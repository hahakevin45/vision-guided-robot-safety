"""Generate the GS3 single-obstacle Gazebo world.

The cylinder geometry comes from the single source of truth
`safety_sim.scenarios.sapf.SAPF_OBSTACLE`, the same constant that feeds the
deterministic safety_sim World and the filter's static obstacle JSON. The two
simulation layers therefore cannot drift apart.
"""
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

from safety_sim.scenarios.sapf import SAPF_OBSTACLE

from gazebo_sim.generators.generate_arena_world import build_arena_world

DEFAULT_OUTPUT = Path("gazebo_sim/worlds/vgr_sapf.world")
CYLINDER_LENGTH_M = 0.5
CYLINDER_CENTER_Z_M = 0.25


def _add_text(parent: ET.Element, tag: str, text: object) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = str(text)
    return child


def build_sapf_world() -> str:
    """Arena world plus the SAPF obstacle box; same geofence as GS1/GS2.

    Obstacle geometry comes from the single source of truth `SAPF_OBSTACLE`
    (Box2D), matching the safety_sim World and the gate's obstacles_json.
    """
    root = ET.fromstring(build_arena_world("vgr_sapf", with_sensors=False))
    world = root.find("world")
    if world is None:
        raise ValueError("generated arena has no world element")

    model = ET.SubElement(world, "model", {"name": "sapf_obstacle"})
    _add_text(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": "sapf_obstacle_link"})
    _add_text(link, "pose",
              f"{SAPF_OBSTACLE.x} {SAPF_OBSTACLE.y} {CYLINDER_CENTER_Z_M} 0 0 0")
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, {"name": f"sapf_obstacle_{kind}"})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        _add_text(box, "size",
                  f"{SAPF_OBSTACLE.size_x} {SAPF_OBSTACLE.size_y} {CYLINDER_LENGTH_M}")
        if kind == "visual":
            material = ET.SubElement(element, "material")
            _add_text(material, "ambient", "0.15 0.55 0.35 1")
            _add_text(material, "diffuse", "0.15 0.55 0.35 1")
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def write_sapf_world(output: Path = DEFAULT_OUTPUT) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(build_sapf_world() + "\n", encoding="utf-8")
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    print(write_sapf_world(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
