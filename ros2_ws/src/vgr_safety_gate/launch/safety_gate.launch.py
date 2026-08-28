"""Launch the vgr_safety_gate node with parameters."""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ARENA = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))
    # Flattened: [0.0, -1.0, 4.0, -1.0, 4.0, 1.0, 0.0, 1.0]
    default_geofence = "[0.0, -1.0, 4.0, -1.0, 4.0, 1.0, 0.0, 1.0]"

    return LaunchDescription([
        DeclareLaunchArgument(
            "filter_name",
            default_value="safe_apf",
            description="Name of the safety filter"
        ),
        DeclareLaunchArgument(
            "geofence",
            default_value=default_geofence,
            description="Flat list of geofence coordinates: [x1, y1, x2, y2, ...]"
        ),
        Node(
            package="vgr_safety_gate",
            executable="safety_gate_node",
            name="safety_gate",
            output="screen",
            parameters=[{
                "filter_name": LaunchConfiguration("filter_name"),
                "geofence": LaunchConfiguration("geofence"),
            }]
        )
    ])
