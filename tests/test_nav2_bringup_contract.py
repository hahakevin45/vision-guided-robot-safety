from pathlib import Path

import yaml


PACKAGE = Path("ros2_ws/src/vgr_nav2_bringup")
PARAMS = PACKAGE / "config/nav2_params.yaml"
LAUNCH = PACKAGE / "launch/navigation.launch.py"


def params() -> dict:
    return yaml.safe_load(PARAMS.read_text(encoding="utf-8"))


def test_nav2_velocity_is_remapped_to_safety_input() -> None:
    launch = LAUNCH.read_text(encoding="utf-8")
    assert '("cmd_vel", "/cmd_vel_nav")' in launch
    assert '"/cmd_vel_safe"' not in launch
    assert launch.count("remappings=velocity_remaps") == 2


def test_visual_obstacle_is_local_only_with_aligned_geometry() -> None:
    config = params()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = config["global_costmap"]["global_costmap"]["ros__parameters"]

    assert local["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]
    assert global_["plugins"] == ["static_layer", "inflation_layer"]
    assert global_["static_layer"]["enabled"] is True
    assert "obstacle_layer" not in global_

    visual = local["obstacle_layer"]["visual"]
    assert visual["topic"] == "/visual_obstacle_points"
    assert visual["data_type"] == "PointCloud2"
    assert visual["sensor_frame"] == ""
    assert visual["min_obstacle_height"] <= 0.0
    assert visual["max_obstacle_height"] >= 0.1
    assert local["obstacle_layer"]["footprint_clearing_enabled"] is False
    for costmap in (local, global_):
        assert costmap["robot_radius"] == 0.23
        assert costmap["footprint_padding"] == 0.0
        assert costmap["inflation_layer"]["inflation_radius"] == 0.28


def test_local_costmap_uses_comparison_footprint_radius() -> None:
    config = params()
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "footprint" not in local
    assert local["robot_radius"] == 0.23
    assert local["footprint_padding"] == 0.0
    follow = config["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["plugin"] == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert follow["desired_linear_vel"] == 0.12
    assert follow["use_collision_detection"] is True
    assert follow["max_allowed_time_to_collision_up_to_carrot"] <= 0.5


def test_dwb_velocity_window_has_signed_deceleration_limits() -> None:
    follow = params()["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["acc_lim_x"] > 0.0
    assert follow["acc_lim_theta"] > 0.0
    assert follow["decel_lim_x"] < 0.0
    assert follow["decel_lim_y"] < 0.0
    assert follow["decel_lim_theta"] < 0.0


def test_navfn_frames_sim_time_and_lifecycle_are_explicit() -> None:
    config = params()
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert config["bt_navigator"]["ros__parameters"]["global_frame"] == "map"
    assert config["bt_navigator"]["ros__parameters"]["robot_base_frame"] == "base_link"
    assert config["bt_navigator"]["ros__parameters"]["odom_topic"] == "/odom"
    for node in ("map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"):
        assert config[node]["ros__parameters"]["use_sim_time"] is True
    launch = LAUNCH.read_text(encoding="utf-8")
    assert '"autostart": True' in launch
    assert '"map_server"' in launch
    bt_plugins = config["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    assert "nav2_remove_passed_goals_action_bt_node" in bt_plugins
    assert "nav2_navigate_through_poses_action_bt_node" in bt_plugins


def test_full_navigation_can_select_dwb_controller() -> None:
    launch = LAUNCH.read_text(encoding="utf-8")
    assert '"controller_plugin"' in launch
    assert '"FollowPath.plugin"' in launch
    assert "dwb_core::DWBLocalPlanner" in launch


def test_launch_selects_exactly_one_odometry_owner() -> None:
    launch = LAUNCH.read_text(encoding="utf-8")
    assert "nav2_integration.odom_adapter" in launch
    assert "nav2_integration.wheel_odom_node" in launch
    assert "nav2_integration.landmark_localizer" in launch
    assert "odom_mode" in launch
    assert "static_transform_publisher" in launch


def test_controller_only_launch_exists_and_has_no_planner() -> None:
    launch = (PACKAGE / "launch/controller_only.launch.py").read_text(encoding="utf-8")
    assert "controller_server" in launch
    assert "planner_server" not in launch
    # obstacle layer observes the odom-frame copy of the visual ray hits.
    assert "/visual_obstacle_points" in PARAMS.read_text(encoding="utf-8")
    # inflation 對齊（公平性）= SAPF d_safe 0.28。
    assert 'inflation_radius": 0.28' in launch
    # This comparison defines footprint radius 0.23 m.
    assert '"local_costmap.robot_radius": 0.23' in launch



def test_full_comparison_runner_defaults_to_source_nav2_params() -> None:
    runner = Path("gazebo_sim/scripts/run_controller_compare.sh").read_text(
        encoding="utf-8")
    assert (
        'NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$REPO_ROOT/ros2_ws/src/'
        'vgr_nav2_bringup/config/nav2_params.yaml}"'
    ) in runner
    assert 'params_file:="$NAV2_PARAMS_FILE"' in runner

def test_full_comparison_runner_isolates_bridge_process_group() -> None:
    runner = Path("gazebo_sim/scripts/run_controller_compare.sh").read_text(
        encoding="utf-8")
    assert "setsid ros2 run ros_gz_bridge parameter_bridge" in runner


def test_full_comparison_runner_uses_box2d_trace_evaluator() -> None:
    runner = Path("gazebo_sim/scripts/run_controller_compare.sh").read_text(
        encoding="utf-8")
    assert (
        "from gazebo_sim.evaluate_local_detour import evaluate_detour_trace"
        in runner
    )


def test_full_comparison_runner_exposes_visual_sensor_range() -> None:
    runner = Path("gazebo_sim/scripts/run_controller_compare.sh").read_text(
        encoding="utf-8")
    assert 'VISUAL_MAX_RANGE="${VISUAL_MAX_RANGE:-3.0}"' in runner
    assert '-p max_range:="$VISUAL_MAX_RANGE"' in runner
    assert 'NAV2_TIMEOUT_S="${NAV2_TIMEOUT_S:-60}"' in runner
    assert '--timeout-s "$NAV2_TIMEOUT_S"' in runner