import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from gazebo_sim.generators.generate_field_active_world import (
    WALL_HEIGHT_M, WALL_THICKNESS_M, build_field_active_world,
)
from safety_sim.experiments.field_scenarios import ARENA


MARKER_MAP = json.loads(Path("config/field_marker_map.json").read_text())


def test_active_world_has_sensor_plugin_collisions_and_no_robot():
    root = ET.fromstring(build_field_active_world(MARKER_MAP))
    names = {p.attrib.get("name") for p in root.iter("plugin")}
    assert "ignition::gazebo::systems::Sensors" in names
    assert root.findtext(".//plugin[@name='ignition::gazebo::systems::Sensors']/render_engine") == "ogre2"
    models = {m.attrib["name"]: m for m in root.iter("model")}
    assert "vgr_diff_drive" not in models
    for index in range(4):
        wall = models[f"field_wall_{index}"]
        assert wall.find(".//collision") is not None
        size = [float(v) for v in wall.findtext(".//box/size").split()]
        assert size[1] == pytest.approx(WALL_THICKNESS_M)
        assert size[2] == pytest.approx(WALL_HEIGHT_M)


def test_active_world_uses_synthetic_walls_and_field_marker_prefix():
    root = ET.fromstring(build_field_active_world(MARKER_MAP))
    includes = [node.findtext("uri") for node in root.iter("include")]
    assert includes == [f"model://field_marker_{i}" for i in range(6)]
    for index, (start, end) in enumerate(zip(ARENA, (*ARENA[1:], ARENA[0]))):
        pose = [float(v) for v in root.find(
            f".//model[@name='field_wall_{index}']/link/pose").text.split()]
        assert pose[0] == pytest.approx((start[0] + end[0]) / 2.0)
        assert pose[1] == pytest.approx((start[1] + end[1]) / 2.0)
