import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from gazebo_sim.generators.generate_arena_world import build_arena_world
from gazebo_sim.generators.generate_robot_sdf import (
    BODY_SIZE_M,
    CAMERA_FRONT_X_M,
    CAMERA_HEIGHT_M,
    CASTER_OFFSET_FROM_REAR_AXLE_M,
    CASTER_Y_OFFSET_M,
    build_robot_sdf,
)
from vgr_core.motion import DiffDriveParams
from safety_sim.scenarios.basic import ARENA


PARAMS = DiffDriveParams()
REPO_ROOT = Path(__file__).resolve().parents[1]


def _texts(root: ET.Element, tag: str) -> list[str]:
    return [elem.text or "" for elem in root.iter(tag)]


def _first_plugin(root: ET.Element, name: str) -> ET.Element:
    for plugin in root.iter("plugin"):
        if plugin.attrib.get("name") == name:
            return plugin
    raise AssertionError(f"找不到 plugin: {name}")


def _plugin_names(root: ET.Element) -> set[str]:
    return {plugin.attrib.get("name", "") for plugin in root.iter("plugin")}


def _model_by_name(root: ET.Element, name: str) -> ET.Element:
    for model in root.iter("model"):
        if model.attrib.get("name") == name:
            return model
    raise AssertionError(f"找不到 model: {name}")


def _link_size(model: ET.Element, link_name: str) -> tuple[float, float, float]:
    link = next(link for link in model.iter("link") if link.attrib.get("name") == link_name)
    size_text = next(link.iter("size")).text
    assert size_text is not None
    return tuple(float(part) for part in size_text.split())


def _link_pose(model: ET.Element, link_name: str) -> tuple[float, float, float, float, float, float]:
    link = next(link for link in model.iter("link") if link.attrib.get("name") == link_name)
    pose_text = next(link.iter("pose")).text
    assert pose_text is not None
    return tuple(float(part) for part in pose_text.split())


def test_robot_sdf_uses_diff_drive_params_for_wheel_geometry():
    root = ET.fromstring(build_robot_sdf(PARAMS))
    plugin = _first_plugin(root, "ignition::gazebo::systems::DiffDrive")

    assert float(plugin.findtext("wheel_separation", "")) == pytest.approx(PARAMS.wheel_base_m)
    assert float(plugin.findtext("wheel_radius", "")) == pytest.approx(PARAMS.wheel_diameter_m / 2.0)


def test_robot_sdf_uses_public_body_camera_and_two_front_casters():
    root = ET.fromstring(build_robot_sdf(PARAMS))
    model = _model_by_name(root, "vgr_diff_drive")

    assert _link_size(model, "chassis") == pytest.approx(BODY_SIZE_M)
    camera_pose = tuple(float(part) for part in model.findtext(".//sensor[@name='front_camera']/pose", "").split())
    assert camera_pose[:3] == pytest.approx((CAMERA_FRONT_X_M, 0.0, CAMERA_HEIGHT_M))

    caster_names = [link.attrib["name"] for link in model.iter("link") if "caster" in link.attrib.get("name", "")]
    assert sorted(caster_names) == ["front_caster_left", "front_caster_right"]
    rear_x_m = -PARAMS.wheel_base_m / 2.0
    caster_x_m = rear_x_m + CASTER_OFFSET_FROM_REAR_AXLE_M
    assert _link_pose(model, "front_caster_left")[:3] == pytest.approx(
        (caster_x_m, CASTER_Y_OFFSET_M, PARAMS.wheel_diameter_m / 4.0)
    )
    assert _link_pose(model, "front_caster_right")[:3] == pytest.approx(
        (caster_x_m, -CASTER_Y_OFFSET_M, PARAMS.wheel_diameter_m / 4.0)
    )


def test_robot_sdf_max_linear_velocity_comes_from_count_limit_formula():
    root = ET.fromstring(build_robot_sdf(PARAMS))
    plugin = _first_plugin(root, "ignition::gazebo::systems::DiffDrive")

    expected = (
        PARAMS.max_counts_per_s
        / max(PARAMS.left_counts_per_rev, PARAMS.right_counts_per_rev)
        * math.pi
        * PARAMS.wheel_diameter_m
    )
    assert float(plugin.findtext("max_linear_velocity", "")) == pytest.approx(expected)


def test_robot_sdf_is_fortress_compatible():
    root = ET.fromstring(build_robot_sdf(PARAMS))

    assert root.attrib["version"] == "1.8"
    plugin = _first_plugin(root, "ignition::gazebo::systems::DiffDrive")
    assert plugin.attrib["filename"] == "libignition-gazebo-diff-drive-system.so"


def test_arena_world_is_fortress_compatible():
    root = ET.fromstring(build_arena_world())

    assert root.attrib["version"] == "1.8"


def test_arena_world_default_omits_sensors_plugin_for_headless_pseudo_ci():
    root = ET.fromstring(build_arena_world())

    assert "ignition::gazebo::systems::Sensors" not in _plugin_names(root)


def test_arena_world_with_sensors_adds_ogre2_sensors_plugin_for_vision():
    root = ET.fromstring(build_arena_world(with_sensors=True))
    sensors = _first_plugin(root, "ignition::gazebo::systems::Sensors")

    assert sensors.attrib["filename"] == "libignition-gazebo-sensors-system.so"
    assert sensors.findtext("render_engine") == "ogre2"


def test_robot_sdf_subscribes_to_safe_command_topic_and_publishes_ground_truth_odometry():
    root = ET.fromstring(build_robot_sdf(PARAMS))
    diff_drive = _first_plugin(root, "ignition::gazebo::systems::DiffDrive")
    odometry = _first_plugin(root, "ignition::gazebo::systems::OdometryPublisher")

    assert diff_drive.findtext("topic") == "/cmd_vel_safe"
    assert diff_drive.findtext("odom_topic") == "/odom"
    assert odometry.attrib["filename"] == "libignition-gazebo-odometry-publisher-system.so"
    assert odometry.findtext("odom_topic") == "/sim/true_pose"
    assert odometry.findtext("robot_base_frame") == "chassis"
    assert float(odometry.findtext("odom_publish_frequency", "0")) >= 50.0
    assert root.find(".//pose_topic") is None


def test_arena_world_walls_enclose_the_safety_sim_arena_inner_space():
    root = ET.fromstring(build_arena_world())
    world = root.find("world")
    assert world is not None

    min_x = min(point[0] for point in ARENA)
    max_x = max(point[0] for point in ARENA)
    min_y = min(point[1] for point in ARENA)
    max_y = max(point[1] for point in ARENA)
    arena_width = max_x - min_x
    arena_depth = max_y - min_y
    wall_thickness = min(arena_width, arena_depth) / 40.0

    models = {model.attrib["name"]: model for model in world.iter("model")}

    east_pose = _link_pose(models["wall_east"], "wall_east_link")
    east_size = _link_size(models["wall_east"], "wall_east_link")
    west_pose = _link_pose(models["wall_west"], "wall_west_link")
    north_pose = _link_pose(models["wall_north"], "wall_north_link")
    north_size = _link_size(models["wall_north"], "wall_north_link")
    south_pose = _link_pose(models["wall_south"], "wall_south_link")

    assert east_pose[0] - east_size[0] / 2.0 == pytest.approx(max_x)
    assert west_pose[0] + east_size[0] / 2.0 == pytest.approx(min_x)
    assert north_pose[1] - north_size[1] / 2.0 == pytest.approx(max_y)
    assert south_pose[1] + north_size[1] / 2.0 == pytest.approx(min_y)
    assert east_size[1] == pytest.approx(arena_depth)
    assert north_size[0] == pytest.approx(arena_width + 2.0 * wall_thickness)


def test_robot_generator_cli_writes_output(tmp_path):
    output = tmp_path / "model.sdf"

    subprocess.run(
        [sys.executable, "-m", "gazebo_sim.generators.generate_robot_sdf", "--output", str(output)],
        check=True,
    )

    root = ET.parse(output).getroot()
    assert _model_by_name(root, "vgr_diff_drive").attrib["name"] == "vgr_diff_drive"


def test_robot_generator_cli_writes_model_config_manifest(tmp_path):
    output = tmp_path / "model.sdf"

    subprocess.run(
        [sys.executable, "-m", "gazebo_sim.generators.generate_robot_sdf", "--output", str(output)],
        check=True,
    )

    config = ET.parse(tmp_path / "model.config").getroot()
    assert config.tag == "model"
    assert config.findtext("name") == "vgr_diff_drive"
    assert config.findtext("version") == "1.0"
    sdf = config.find("sdf")
    assert sdf is not None
    assert sdf.attrib["version"] == "1.8"
    assert sdf.text == "model.sdf"
    assert config.findtext("author/name") == "Vision Guided Robot"
    assert config.findtext("description") == "Gazebo Fortress diff-drive model for VGR."


def test_arena_generator_cli_writes_output(tmp_path):
    output = tmp_path / "vgr_arena.world"

    subprocess.run(
        [sys.executable, "-m", "gazebo_sim.generators.generate_arena_world", "--output", str(output)],
        check=True,
    )

    root = ET.parse(output).getroot()
    assert root.find("world").attrib["name"] == "vgr_arena"
    assert "ignition::gazebo::systems::Sensors" not in _plugin_names(root)


def test_arena_generator_cli_with_sensors_writes_vision_world(tmp_path):
    output = tmp_path / "vgr_arena_vision.world"

    subprocess.run(
        [
            sys.executable,
            "-m",
            "gazebo_sim.generators.generate_arena_world",
            "--with-sensors",
            "--output",
            str(output),
        ],
        check=True,
    )

    root = ET.parse(output).getroot()
    sensors = _first_plugin(root, "ignition::gazebo::systems::Sensors")
    assert sensors.findtext("render_engine") == "ogre2"


def test_g1_straight_line_script_has_expected_contract():
    script = REPO_ROOT / "gazebo_sim" / "scripts" / "run_g1_straight_line.sh"

    text = script.read_text(encoding="utf-8")

    assert "set -euo pipefail" in text
    assert "ign gazebo -s -r" in text
    assert "/cmd_vel_safe" in text
    assert 'publish_twist "0.15" "7.0"' in text
    assert "/sim/true_pose" in text
    assert "G1_OK" in text
    assert "G1_FAIL" in text


def _wheel_friction(root: ET.Element, link_name: str) -> tuple[float, float] | None:
    """回傳 (mu, mu2) 或 None（無 friction 設定）。"""
    for link in root.iter("link"):
        if link.attrib.get("name") != link_name:
            continue
        collision = next(c for c in link.iter("collision"))
        ode = collision.find(".//ode")
        if ode is None:
            return None
        mu = ode.findtext("mu")
        mu2 = ode.findtext("mu2")
        if mu is None:
            return None
        return float(mu), float(mu2) if mu2 is not None else None
    raise AssertionError(f"no link {link_name}")


def test_robot_sdf_default_wheels_have_no_friction_block():
    """預設維持現狀：驅動輪無 surface/friction（ODE 預設高摩擦）。"""
    root = ET.fromstring(build_robot_sdf(PARAMS))
    assert _wheel_friction(root, "left_wheel") is None
    assert _wheel_friction(root, "right_wheel") is None


def test_robot_sdf_wheel_friction_parametrized_per_wheel():
    root = ET.fromstring(build_robot_sdf(
        PARAMS, left_wheel_mu=0.4, right_wheel_mu=1.0, wheel_mu2=0.5))
    left = _wheel_friction(root, "left_wheel")
    right = _wheel_friction(root, "right_wheel")
    assert left == (0.4, 0.5)
    assert right == (1.0, 0.5)


def test_robot_sdf_wheel_friction_left_only_keeps_right_default():
    root = ET.fromstring(build_robot_sdf(PARAMS, left_wheel_mu=0.3))
    assert _wheel_friction(root, "left_wheel") == (0.3, None)
    assert _wheel_friction(root, "right_wheel") is None


def test_robot_sdf_rejects_out_of_range_mu():
    with pytest.raises(ValueError):
        build_robot_sdf(PARAMS, left_wheel_mu=0.0)
    with pytest.raises(ValueError):
        build_robot_sdf(PARAMS, right_wheel_mu=1.5)
    with pytest.raises(ValueError):
        build_robot_sdf(PARAMS, wheel_mu2=-0.1)


def test_robot_sdf_physical_radius_override():
    root = ET.fromstring(build_robot_sdf(PARAMS, left_wheel_radius_m=0.028))
    left = next(l for l in root.iter("link") if l.attrib.get("name") == "left_wheel")
    radius_text = next(c for c in left.iter("radius")).text
    assert float(radius_text) == pytest.approx(0.028)
    # plugin 宣告輪徑維持 params（odom 使用）→ encoder-invisible 誤差來源
    plugin = _first_plugin(root, "ignition::gazebo::systems::DiffDrive")
    assert float(plugin.findtext("wheel_radius", "")) == pytest.approx(
        PARAMS.wheel_diameter_m / 2.0)
    # 輪心 z pose 跟隨物理半徑
    z = next(p for p in left.iter("pose")).text.split()[2]
    assert float(z) == pytest.approx(0.028)


def test_robot_sdf_radius_override_rejects_invalid():
    with pytest.raises(ValueError):
        build_robot_sdf(PARAMS, left_wheel_radius_m=0.0)
    with pytest.raises(ValueError):
        build_robot_sdf(PARAMS, right_wheel_radius_m=1.5)


def test_sapf_world_obstacle_is_box_not_cylinder():
    from gazebo_sim.generators.generate_sapf_world import build_sapf_world

    root = ET.fromstring(build_sapf_world())
    model = _model_by_name(root, "sapf_obstacle")
    size_text = next(model.iter("size")).text
    assert size_text is not None
    sx, sy = (float(v) for v in size_text.split()[:2])
    # box 產生 <size>（無 <radius>）
    assert not [t for t in _texts(model, "radius")], "no cylinder radius allowed"
    # 尺寸 = SAPF_OBSTACLE（0.40 × 0.60）
    assert sx == pytest.approx(0.40)
    assert sy == pytest.approx(0.40)
