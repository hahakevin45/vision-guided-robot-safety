from pathlib import Path
import math
import xml.etree.ElementTree as ET

from gazebo_sim.generators.generate_robot_sdf import build_robot_sdf
from nav2_integration.ros_helpers import joint_radians_to_counts
from vgr_core.motion import DiffDriveParams


def test_joint_radians_use_simulator_signs_not_real_encoder_signs() -> None:
    left, right = joint_radians_to_counts(
        left_rad=2.0 * math.pi,
        right_rad=-2.0 * math.pi,
        left_counts_per_rev=750.0,
        right_counts_per_rev=749.0,
        left_joint_sign=1,
        right_joint_sign=-1,
    )
    assert left == 750
    assert right == 749


def test_ground_truth_adapter_normalizes_nav2_frames_and_owns_local_tf() -> None:
    text = Path("nav2_integration/odom_adapter.py").read_text(encoding="utf-8")
    assert 'frame_id = "odom"' in text
    assert 'child_frame_id = "base_link"' in text
    assert "TransformBroadcaster" in text
    assert '"/sim/true_pose_raw"' in text


def test_landmark_localizer_owns_only_global_tf() -> None:
    text = Path("nav2_integration/landmark_localizer.py").read_text(encoding="utf-8")
    assert 'header.frame_id = "map"' in text
    assert 'child_frame_id = "odom"' in text
    assert 'child_frame_id = "base_link"' not in text
    assert "map_to_odom" in text
    assert 'lookup_transform(\n                    "odom", "base_link", Time(),' in text
    assert "Time.from_msg" not in text
    assert "create_timer(1.0 / 20.0, self._publish_latest)" in text


def test_robot_sdf_publishes_joint_state_without_changing_safe_topic() -> None:
    root = ET.fromstring(build_robot_sdf(DiffDriveParams()))
    joint_state = root.find(
        ".//plugin[@name='ignition::gazebo::systems::JointStatePublisher']"
    )
    assert joint_state is not None
    assert [item.text for item in joint_state.findall("joint_name")] == [
        "left_wheel_joint",
        "right_wheel_joint",
    ]
    diff_drive = root.find(".//plugin[@name='ignition::gazebo::systems::DiffDrive']")
    assert diff_drive is not None
    assert diff_drive.findtext("topic") == "/cmd_vel_safe"


def test_wheel_odom_republishes_stationary_tf_on_a_timer() -> None:
    text = Path("nav2_integration/wheel_odom_node.py").read_text(encoding="utf-8")
    assert "create_timer(1.0 / 20.0, self._publish_latest)" in text
    assert "def _publish_latest" in text
    assert "self.get_clock().now().to_msg()" in text


def test_aruco_detector_accepts_a_navigation_marker_map_parameter() -> None:
    text = Path("gazebo_sim/nodes/aruco_detector.py").read_text(encoding="utf-8")
    assert 'declare_parameter("marker_map_path"' in text
    assert 'get_parameter("marker_map_path")' in text
