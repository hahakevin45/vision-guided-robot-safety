"""Launch the static-map VGR Nav2 stack without AMCL or range sensors."""
from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("vgr_nav2_bringup")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    odom_mode = LaunchConfiguration("odom_mode")
    controller_plugin = LaunchConfiguration("controller_plugin")
    common_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    velocity_remaps = common_remaps + [("cmd_vel", "/cmd_vel_nav")]

    managed = [
        "map_server", "planner_server", "behavior_server", "bt_navigator",
    ]
    # SAPF 比較 arm（use_controller=false）不啟動 controller_server；
    # lifecycle manager 的 node 列表必須同步，否則 activation 卡住。
    if os.environ.get("VGR_NAV2_USE_CONTROLLER", "1") == "1":
        managed.insert(2, "controller_server")
    actions = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(share, "maps", "vgr_nav2.yaml")
        ),
        DeclareLaunchArgument(
            "odom_mode", default_value="ground_truth",
            description="ground_truth or wheel_odom",
        ),
        DeclareLaunchArgument(
            "use_controller", default_value="true",
            description="false = planner-only（SAPF 取代 Nav2 controller 的比較 arm）",
        ),
        DeclareLaunchArgument(
            "controller_plugin", default_value="rpp",
            description="rpp or dwb",
        ),
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen", parameters=[params_file, {"yaml_filename": map_file}],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_planner", executable="planner_server", name="planner_server",
            output="screen", parameters=[params_file], remappings=common_remaps,
        ),
        Node(
            package="nav2_controller", executable="controller_server", name="controller_server",
            output="screen", parameters=[params_file, {
                "FollowPath.plugin": PythonExpression([
                    "'dwb_core::DWBLocalPlanner' if '",
                    controller_plugin, "' == 'dwb' else "
                    "'nav2_regulated_pure_pursuit_controller::"
                    "RegulatedPurePursuitController'"]),
            }], remappings=velocity_remaps,
            condition=IfCondition(LaunchConfiguration("use_controller")),
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server", name="behavior_server",
            output="screen", parameters=[params_file], remappings=velocity_remaps,
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
            output="screen", parameters=[params_file], remappings=common_remaps,
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{"use_sim_time": True, "autostart": True, "node_names": managed}],
        ),
        ExecuteProcess(
            cmd=[sys.executable, "-m", "nav2_integration.odom_adapter",
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
            condition=IfCondition(PythonExpression(["'", odom_mode, "' == 'ground_truth'"])),
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_odom_identity",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "map", "--child-frame-id", "odom"],
            condition=IfCondition(PythonExpression(["'", odom_mode, "' == 'ground_truth'"])),
        ),
        ExecuteProcess(
            cmd=[sys.executable, "-m", "nav2_integration.wheel_odom_node",
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
            condition=IfCondition(PythonExpression(["'", odom_mode, "' == 'wheel_odom'"])),
        ),
        ExecuteProcess(
            cmd=[sys.executable, "-m", "nav2_integration.landmark_localizer",
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
            condition=IfCondition(PythonExpression(["'", odom_mode, "' == 'wheel_odom'"])),
        ),
    ]
    return LaunchDescription(actions)
