"""Launch wall-time Nav2 for raised-wheel lifecycle and TF validation only."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("vgr_nav2_bringup")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    common_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    velocity_remaps = common_remaps + [("cmd_vel", "/cmd_vel_nav")]
    managed = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "nav2_real_params.yaml"),
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(share, "maps", "vgr_nav2.yaml"),
        ),
        DeclareLaunchArgument(
            # 架空 bench 用 identity map→odom；落地跑真定位時設 false，
            # 由 landmark_localizer 從 /aruco/pose 發布 map→odom 修正。
            "use_identity_map_to_odom",
            default_value="true",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="bench_map_to_odom_identity",
            condition=IfCondition(LaunchConfiguration("use_identity_map_to_odom")),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"yaml_filename": map_file}],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
            remappings=velocity_remaps,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file],
            remappings=velocity_remaps,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": managed,
            }],
        ),
    ])
