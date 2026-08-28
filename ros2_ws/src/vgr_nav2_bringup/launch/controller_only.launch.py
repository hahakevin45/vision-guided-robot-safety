"""Launch only the Nav2 controller (DWB/RPP) and local costmap.

No planner, behavior tree, or navigation action server. The controller follows
the supplied path while its local costmap consumes visual obstacle points.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("vgr_nav2_bringup")
    params_file = LaunchConfiguration("params_file")
    controller_plugin = LaunchConfiguration("controller_plugin")
    common_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    velocity_remaps = common_remaps + [("cmd_vel", "/cmd_vel_safe")]

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument(
            "controller_plugin", default_value="dwb",
            description="dwb or rpp"),
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            parameters=[params_file, {
                "controller_plugins": ["FollowPath"],
                "FollowPath.plugin": PythonExpression([
                    "'dwb_core::DWBLocalPlanner' if '",
                    controller_plugin, "' == 'dwb' else "
                    "'nav2_regulated_pure_pursuit_controller::"
                    "RegulatedPurePursuitController'"]),
                # inflation 對齊（公平性）：= SAPF d_safe 0.28
                "local_costmap.robot_radius": 0.23,
                "local_costmap.local_costmap.inflation_layer.inflation_radius": 0.28,
                "local_costmap.local_costmap.inflation_layer.cost_scaling_factor": 3.0,
            }],
            remappings=velocity_remaps,
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "tf2_ros", "static_transform_publisher",
                 "--x", "0", "--y", "0", "--z", "0",
                 "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                 "--frame-id", "map", "--child-frame-id", "odom"],
            output="screen",
        ),
    ])
