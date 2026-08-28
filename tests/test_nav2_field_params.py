from pathlib import Path

import yaml


PACKAGE = Path("ros2_ws/src/vgr_nav2_bringup")
FIELD_PARAMS = PACKAGE / "config/nav2_field_params.yaml"
REAL_PARAMS = PACKAGE / "config/nav2_real_params.yaml"
EXPECTED_FOOTPRINT = "[[0.20, 0.11], [0.20, -0.11], [-0.20, -0.11], [-0.20, 0.11]]"


def _load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_field_params_use_low_speed_envelope_and_map_global_costmap() -> None:
    config = _load(FIELD_PARAMS)
    controller = config["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    assert follow["desired_linear_vel"] <= 0.08
    assert follow["rotate_to_heading_angular_vel"] <= 0.25
    assert config["global_costmap"]["global_costmap"]["ros__parameters"]["global_frame"] == "map"

    def assert_speed_bounds(value, key: str) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return
        angular_keys = {
            "rotate_to_heading_angular_vel",
            "max_angular_accel",
            "max_rotational_vel",
            "min_rotational_vel",
            "rotational_acc_lim",
        }
        linear_keys = {
            "desired_linear_vel",
            "min_approach_linear_velocity",
            "regulated_linear_scaling_min_speed",
            "min_x_velocity_threshold",
            "min_y_velocity_threshold",
            "min_theta_velocity_threshold",
        }
        if key in angular_keys:
            assert value <= 0.25, (key, value)
        elif key in linear_keys:
            assert value <= 0.08, (key, value)

    def walk(node) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                assert_speed_bounds(value, key)
                walk(value)
        elif isinstance(node, list):
            for value in node:
                walk(value)

    walk(config)


def test_field_params_preserve_real_footprint() -> None:
    field = _load(FIELD_PARAMS)
    real = _load(REAL_PARAMS)
    for name in ("local_costmap", "global_costmap"):
        field_costmap = field[name][name]["ros__parameters"]
        real_costmap = real[name][name]["ros__parameters"]
        assert field_costmap["footprint"] == EXPECTED_FOOTPRINT
        assert field_costmap["footprint"] == real_costmap["footprint"]
