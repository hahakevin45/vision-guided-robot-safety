"""產生對齊 safety_sim ARENA 的 Gazebo Fortress world。"""
from __future__ import annotations

import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

from safety_sim.scenarios.basic import ARENA


DEFAULT_OUTPUT = "gazebo_sim/worlds/vgr_arena.world"
DEFAULT_VISION_OUTPUT = "gazebo_sim/worlds/vgr_arena_vision.world"
DEFAULT_MARKER_MAP_OUTPUT = "gazebo_sim/models/markers/marker_map.json"
SDF_VERSION = "1.8"
MARKER_SIZE_M = 0.17   # synthetic public marker edge length
MARKER_CENTER_Z_M = 0.10
MARKER_THICKNESS_M = 0.002

# Gazebo Fortress / ignition-gazebo6 的 world system plugin 命名。
# Garden / gz-sim7+ 會改用 gz-sim-* 與 gz::sim::*；升級時集中改這裡。
WORLD_SYSTEM_PLUGINS = (
    ("libignition-gazebo-physics-system.so", "ignition::gazebo::systems::Physics"),
    ("libignition-gazebo-user-commands-system.so", "ignition::gazebo::systems::UserCommands"),
    ("libignition-gazebo-scene-broadcaster-system.so", "ignition::gazebo::systems::SceneBroadcaster"),
)
SENSORS_SYSTEM_PLUGIN = (
    ("libignition-gazebo-sensors-system.so", "ignition::gazebo::systems::Sensors"),
)


def _add_text(parent: ET.Element, tag: str, text: object) -> ET.Element:
    elem = ET.SubElement(parent, tag)
    elem.text = str(text)
    return elem


def _fmt(value: float) -> str:
    return f"{value:.12g}"


def _pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> str:
    return " ".join(_fmt(part) for part in (x, y, z, roll, pitch, yaw))


def _box_size(x: float, y: float, z: float) -> str:
    return " ".join(_fmt(part) for part in (x, y, z))


def _arena_bounds() -> tuple[float, float, float, float]:
    xs = [point[0] for point in ARENA]
    ys = [point[1] for point in ARENA]
    return min(xs), max(xs), min(ys), max(ys)


def _add_box_model(world: ET.Element, name: str, pose: str, size: str,
                   rgba: str = "0.75 0.72 0.68 1") -> None:
    model = ET.SubElement(world, "model", {"name": name})
    _add_text(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": f"{name}_link"})
    _add_text(link, "pose", pose)
    collision = ET.SubElement(link, "collision", {"name": f"{name}_collision"})
    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    _add_text(box, "size", size)
    visual = ET.SubElement(link, "visual", {"name": f"{name}_visual"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    _add_text(box, "size", size)
    material = ET.SubElement(visual, "material")
    _add_text(material, "ambient", rgba)
    _add_text(material, "diffuse", rgba)


def _add_marker_model(world: ET.Element, marker: dict[str, float | int]) -> None:
    """以標準 Gazebo model include 加入牆內側 marker。"""
    marker_id = int(marker["id"])
    include = ET.SubElement(world, "include")
    _add_text(include, "uri", f"model://marker_{marker_id}")
    _add_text(
        include,
        "pose",
        _pose(
            float(marker["x"]),
            float(marker["y"]),
            float(marker["z"]),
            0.0,
            0.0,
            float(marker["yaw"]),
        ),
    )


def _add_sun_light(world: ET.Element) -> None:
    """定向太陽光。沒有光源時 Fortress 只剩環境光，場景完全沒有立體感。"""
    light = ET.SubElement(world, "light", {"type": "directional", "name": "sun"})
    _add_text(light, "cast_shadows", "true")
    _add_text(light, "pose", "0 0 10 0 0 0")
    _add_text(light, "diffuse", "0.9 0.9 0.9 1")
    _add_text(light, "specular", "0.2 0.2 0.2 1")
    _add_text(light, "direction", "-0.4 0.2 -0.9")


def _add_world_system_plugins(world: ET.Element, *, with_sensors: bool) -> None:
    plugins = WORLD_SYSTEM_PLUGINS + (SENSORS_SYSTEM_PLUGIN if with_sensors else ())
    for filename, name in plugins:
        plugin = ET.SubElement(world, "plugin", {"filename": filename, "name": name})
        if name == "ignition::gazebo::systems::Sensors":
            _add_text(plugin, "render_engine", "ogre2")


def build_marker_map() -> dict[str, object]:
    """Return the synthetic marker ground-truth poses."""
    min_x, max_x, min_y, max_y = _arena_bounds()
    thickness_offset = MARKER_THICKNESS_M / 2.0
    x_quarters = (
        min_x + (max_x - min_x) / 3.0,
        min_x + 2.0 * (max_x - min_x) / 3.0,
    )
    y_thirds = (
        min_y + (max_y - min_y) / 3.0,
        min_y + 2.0 * (max_y - min_y) / 3.0,
    )
    markers = [
        {"id": 0, "x": max_x - thickness_offset, "y": y_thirds[0], "z": MARKER_CENTER_Z_M, "yaw": math.pi, "size_m": MARKER_SIZE_M},
        {"id": 1, "x": max_x - thickness_offset, "y": y_thirds[1], "z": MARKER_CENTER_Z_M, "yaw": math.pi, "size_m": MARKER_SIZE_M},
        {"id": 2, "x": min_x + thickness_offset, "y": y_thirds[0], "z": MARKER_CENTER_Z_M, "yaw": 0.0, "size_m": MARKER_SIZE_M},
        {"id": 3, "x": min_x + thickness_offset, "y": y_thirds[1], "z": MARKER_CENTER_Z_M, "yaw": 0.0, "size_m": MARKER_SIZE_M},
        {"id": 4, "x": x_quarters[0], "y": max_y - thickness_offset, "z": MARKER_CENTER_Z_M, "yaw": -math.pi / 2.0, "size_m": MARKER_SIZE_M},
        {"id": 5, "x": x_quarters[1], "y": max_y - thickness_offset, "z": MARKER_CENTER_Z_M, "yaw": -math.pi / 2.0, "size_m": MARKER_SIZE_M},
        {"id": 6, "x": x_quarters[0], "y": min_y + thickness_offset, "z": MARKER_CENTER_Z_M, "yaw": math.pi / 2.0, "size_m": MARKER_SIZE_M},
        {"id": 7, "x": x_quarters[1], "y": min_y + thickness_offset, "z": MARKER_CENTER_Z_M, "yaw": math.pi / 2.0, "size_m": MARKER_SIZE_M},
    ]
    return {
        "dictionary": "DICT_6X6_250",
        "frame": "world",
        "markers": markers,
    }


def build_arena_world(world_name: str = "vgr_arena", *, with_sensors: bool = False) -> str:
    """回傳 4m x 2m 實體圍牆場地的 world XML 字串。

    內空間直接取 safety_sim.scenarios.basic.ARENA。牆厚、高度與地板厚度
    由 ARENA 尺寸比例推導，避免 Gazebo world 另有一份手填幾何。
    """
    min_x, max_x, min_y, max_y = _arena_bounds()
    width_m = max_x - min_x
    depth_m = max_y - min_y
    scale_m = min(width_m, depth_m)
    wall_thickness_m = scale_m / 40.0
    wall_height_m = scale_m / 4.0
    floor_thickness_m = scale_m / 200.0
    center_x_m = (min_x + max_x) / 2.0
    center_y_m = (min_y + max_y) / 2.0

    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    world = ET.SubElement(sdf, "world", {"name": world_name})
    _add_world_system_plugins(world, with_sensors=with_sensors)
    _add_sun_light(world)

    # 地板淺灰、牆半透明磚紅：牆地對比 + 可透視，立體視角下車不會被牆擋住。
    floor_rgba = "0.85 0.85 0.85 1"
    wall_rgba = "0.62 0.26 0.2 0.55"

    _add_box_model(
        world,
        "floor",
        _pose(center_x_m, center_y_m, -floor_thickness_m / 2.0),
        _box_size(width_m, depth_m, floor_thickness_m),
        rgba=floor_rgba,
    )
    _add_box_model(
        world,
        "wall_east",
        _pose(max_x + wall_thickness_m / 2.0, center_y_m, wall_height_m / 2.0),
        _box_size(wall_thickness_m, depth_m, wall_height_m),
        rgba=wall_rgba,
    )
    _add_box_model(
        world,
        "wall_west",
        _pose(min_x - wall_thickness_m / 2.0, center_y_m, wall_height_m / 2.0),
        _box_size(wall_thickness_m, depth_m, wall_height_m),
        rgba=wall_rgba,
    )
    _add_box_model(
        world,
        "wall_north",
        _pose(center_x_m, max_y + wall_thickness_m / 2.0, wall_height_m / 2.0),
        _box_size(width_m + 2.0 * wall_thickness_m, wall_thickness_m, wall_height_m),
        rgba=wall_rgba,
    )
    _add_box_model(
        world,
        "wall_south",
        _pose(center_x_m, min_y - wall_thickness_m / 2.0, wall_height_m / 2.0),
        _box_size(width_m + 2.0 * wall_thickness_m, wall_thickness_m, wall_height_m),
        rgba=wall_rgba,
    )
    for marker in build_marker_map()["markers"]:
        _add_marker_model(world, marker)

    ET.indent(sdf, space="  ")
    return ET.tostring(sdf, encoding="unicode")


def write_marker_map(output: str = DEFAULT_MARKER_MAP_OUTPUT) -> None:
    """寫出 ArUco marker 世界座標真值 JSON。"""
    directory = os.path.dirname(output)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        json.dump(build_marker_map(), f, ensure_ascii=False, indent=2)
        f.write("\n")


def write_arena_world(
    output: str = DEFAULT_OUTPUT,
    marker_map_output: str = DEFAULT_MARKER_MAP_OUTPUT,
    *,
    with_sensors: bool = False,
) -> None:
    """寫出 world 檔與同源 marker map，必要時建立輸出目錄。"""
    directory = os.path.dirname(output)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(output, "w", encoding="utf-8") as f:
        f.write(build_arena_world(with_sensors=with_sensors))
        f.write("\n")
    write_marker_map(marker_map_output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="產生 VGR Gazebo 圍牆場地")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="輸出 world 路徑")
    parser.add_argument("--marker-map-output", default=DEFAULT_MARKER_MAP_OUTPUT, help="輸出 marker map JSON 路徑")
    parser.add_argument("--with-sensors", action="store_true", help="加入 ogre2 Sensors plugin（vision 模式用）")
    args = parser.parse_args(argv)
    write_arena_world(args.output, args.marker_map_output, with_sensors=args.with_sensors)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
