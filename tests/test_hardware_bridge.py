import os
import pty
from pathlib import Path
from types import SimpleNamespace

import pytest

from vgr_core.model import CommandID
from vgr_core.model import MotorIntent
from vgr_driver.driver import ControllerBridge
from vgr_core.protocol import EncoderPacket
from vgr_driver.driver import (
    HardwareBridgeConfig,
    HardwareBridgeSession,
    HardwareFault,
    odom_payload,
)
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver import PosixSerial
from vgr_core.motion import OdomState


class RecordingBridge:
    def __init__(self, packets=((100, 200), (100, 200))):
        self.calls = []
        self.packets = iter(packets)

    def send_command(self, command):
        self.calls.append(("command", command))
        return SimpleNamespace(latency_ms=1.0)

    def send_set_wheel_speed(self, left, right):
        self.calls.append(("speed", left, right))
        return SimpleNamespace(latency_ms=1.5)

    def read_encoders(self):
        self.calls.append(("encoders",))
        left, right = next(self.packets)
        return SimpleNamespace(
            latency_ms=2.0,
            packet=EncoderPacket(
                sequence=0,
                left_count=left,
                right_count=right,
            ),
        )


def test_startup_stops_before_encoder_baseline():
    bridge = RecordingBridge()
    session = HardwareBridgeSession(bridge, HardwareBridgeConfig())

    sample = session.startup(10.0)

    assert bridge.calls == [("command", CommandID.STOP), ("encoders",)]
    assert sample.odom.x == 0.0
    assert sample.odom.linear_mps == 0.0


def test_default_encoder_signs_match_firmware_forward_positive_convention():
    config = HardwareBridgeConfig()

    assert config.left_encoder_sign == 1
    assert config.right_encoder_sign == 1


def test_motion_disabled_rejects_nonzero_command_and_sends_stop():
    bridge = RecordingBridge()
    session = HardwareBridgeSession(
        bridge,
        HardwareBridgeConfig(allow_motion=False),
    )
    session.startup(0.0)
    session.update_command(0.05, 0.0, stamp_s=0.01)

    sample = session.cycle(0.05)

    assert bridge.calls[-2:] == [
        ("command", CommandID.STOP),
        ("encoders",),
    ]
    assert sample.left_target_cps == 0
    assert sample.right_target_cps == 0


def test_fresh_enabled_command_reaches_speed_exchange():
    bridge = RecordingBridge(packets=((0, 0), (10, 10)))
    session = HardwareBridgeSession(
        bridge,
        HardwareBridgeConfig(allow_motion=True),
    )
    session.startup(0.0)
    session.update_command(0.05, 0.0, stamp_s=0.01)

    sample = session.cycle(0.05)

    assert bridge.calls[-2][0] == "speed"
    assert sample.left_target_cps > 0
    assert sample.right_target_cps > 0
    assert sample.odom.x > 0.0


def test_stale_command_sends_explicit_stop_before_encoder_read():
    bridge = RecordingBridge()
    session = HardwareBridgeSession(
        bridge,
        HardwareBridgeConfig(allow_motion=True, cmd_timeout_s=0.20),
    )
    session.startup(0.0)
    session.update_command(0.05, 0.0, stamp_s=0.01)

    session.cycle(0.25)

    assert bridge.calls[-2:] == [
        ("command", CommandID.STOP),
        ("encoders",),
    ]


def test_zero_command_uses_explicit_stop_instead_of_speed_packet():
    bridge = RecordingBridge()
    session = HardwareBridgeSession(
        bridge,
        HardwareBridgeConfig(allow_motion=True),
    )
    session.startup(0.0)
    session.update_command(0.0, 0.0, stamp_s=0.01)

    session.cycle(0.05)

    assert bridge.calls[-2:] == [
        ("command", CommandID.STOP),
        ("encoders",),
    ]


def test_targets_are_clamped_to_bench_limit_without_changing_curvature():
    bridge = RecordingBridge()
    session = HardwareBridgeSession(
        bridge,
        HardwareBridgeConfig(allow_motion=True, max_counts_per_s=120),
    )
    session.startup(0.0)
    session.update_command(1.0, 1.0, stamp_s=0.01)

    sample = session.cycle(0.05)

    assert max(abs(sample.left_target_cps), abs(sample.right_target_cps)) == 120
    assert 0 < sample.left_target_cps < sample.right_target_cps


def test_serial_exception_latches_fault_and_attempts_stop():
    bridge = RecordingBridge()
    session = HardwareBridgeSession(
        bridge,
        HardwareBridgeConfig(allow_motion=True),
    )
    session.startup(0.0)

    def fail_speed(_left, _right):
        raise TimeoutError("link")

    bridge.send_set_wheel_speed = fail_speed
    session.update_command(0.05, 0.0, stamp_s=0.01)

    with pytest.raises(HardwareFault, match="link"):
        session.cycle(0.05)

    assert session.fault == "TimeoutError: link"
    assert bridge.calls[-1] == ("command", CommandID.STOP)


def test_faulted_session_rejects_future_cycles():
    bridge = RecordingBridge()
    session = HardwareBridgeSession(
        bridge,
        HardwareBridgeConfig(allow_motion=True),
    )
    session.startup(0.0)

    def fail_speed(_left, _right):
        raise TimeoutError("link")

    bridge.send_set_wheel_speed = fail_speed
    session.update_command(0.05, 0.0, stamp_s=0.01)
    with pytest.raises(HardwareFault):
        session.cycle(0.05)

    with pytest.raises(HardwareFault, match="latched"):
        session.cycle(0.10)


def test_session_uses_real_binary_codec_with_pty_mock_mcu():
    master_fd, slave_fd = pty.openpty()
    mock = MockSerialMCU(master_fd)
    mock.start()
    try:
        with PosixSerial(os.ttyname(slave_fd), timeout_s=1.0) as serial:
            session = HardwareBridgeSession(
                ControllerBridge(serial),
                HardwareBridgeConfig(
                    allow_motion=True,
                    left_encoder_sign=1,
                    right_encoder_sign=1,
                ),
            )
            baseline = session.startup(0.0)
            session.update_command(0.05, 0.0, stamp_s=0.01)
            moving = session.cycle(0.05)
            stopped = session.cycle(0.30)

        assert baseline.raw_left == 0
        assert moving.left_target_cps > 0
        assert stopped.left_target_cps == 0
        assert mock.mcu.motor_intent == MotorIntent.STOP
        assert mock.stats.decode_errors == 0
    finally:
        mock.stop()
        os.close(master_fd)
        os.close(slave_fd)


def test_odom_payload_uses_required_frames_covariance_and_yaw():
    state = OdomState(
        x=1.0,
        y=2.0,
        theta=0.5,
        linear_mps=0.1,
        angular_rad_s=-0.2,
        stamp_s=4.0,
    )

    payload = odom_payload(state)

    assert payload["frame_id"] == "odom"
    assert payload["child_frame_id"] == "base_link"
    assert payload["pose_covariance"][0] > 0.0
    assert payload["pose_covariance"][7] > 0.0
    assert payload["pose_covariance"][35] > 0.0
    assert payload["twist_covariance"][0] > 0.0
    assert payload["twist_covariance"][35] > 0.0
    assert payload["position"] == {"x": 1.0, "y": 2.0}
    assert payload["twist"]["linear_x"] == 0.1
    assert payload["twist"]["angular_z"] == -0.2
    assert payload["quaternion"][2] > 0.0
    assert payload["quaternion"][3] > 0.0


def test_ros_node_source_owns_only_safe_actuator_input():
    source = Path("ros2_ws/src/vgr_runtime/vgr_runtime/ros/hardware_bridge.py").read_text(encoding="utf-8")

    assert '"/cmd_vel_safe"' in source
    assert '"/odom"' in source
    assert '"/hardware/status"' in source
    assert "TransformBroadcaster" in source
    assert 'declare_parameter("allow_motion", False)' in source
    assert 'create_subscription(Twist, "/cmd_vel"' not in source
    assert 'create_subscription(Twist, "/cmd_vel_nav"' not in source


def test_status_payload_has_stable_diagnostic_keys():
    expected = {
        "allow_motion",
        "fault",
        "raw_left",
        "raw_right",
        "left_target_cps",
        "right_target_cps",
        "command_latency_ms",
        "encoder_latency_ms",
        "last_exchange_age_s",
    }
    source = Path("ros2_ws/src/vgr_runtime/vgr_runtime/ros/hardware_bridge.py").read_text(encoding="utf-8")

    for key in expected:
        assert f'"{key}"' in source


def test_ros_node_shutdown_is_idempotent_after_signal_handler():
    source = Path("ros2_ws/src/vgr_runtime/vgr_runtime/ros/hardware_bridge.py").read_text(encoding="utf-8")

    assert "rclpy.try_shutdown()" in source
    assert "        rclpy.shutdown()\n" not in source
    assert "from rclpy.executors import ExternalShutdownException" in source
    assert "except (KeyboardInterrupt, ExternalShutdownException):" in source
