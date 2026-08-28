"""整合測試：cmd_vel 逆運動學 → SET_WHEEL_SPEED → PTY mock MCU 解碼。

驗證 twist_to_wheel_counts 算出的左右目標，經 serial 編碼、被 mock MCU 正確
解回相同的 counts/s。不需硬體、不需 ROS。
"""
import os
import pty

from vgr_core.model import CommandID
from vgr_driver.driver import ControllerBridge
from vgr_core.motion import DiffDriveParams, twist_to_wheel_counts
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver import PosixSerial

PARAMS = DiffDriveParams()


def _send_twist_get_mock_targets(v: float, w: float):
    left_cps, right_cps = twist_to_wheel_counts(v, w, PARAMS)
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    mock = MockSerialMCU(master_fd)
    mock.start()
    try:
        with PosixSerial(slave_name, baudrate=115200, timeout_s=1.0) as serial:
            bridge = ControllerBridge(serial)
            bridge.send_command(CommandID.HEARTBEAT)
            bridge.send_set_wheel_speed(left_cps, right_cps)
    finally:
        mock.stop()
        os.close(master_fd)
        os.close(slave_fd)
    return (left_cps, right_cps), (mock.left_target_cps, mock.right_target_cps)


def test_straight_forward_targets_reach_mcu():
    sent, received = _send_twist_get_mock_targets(0.1, 0.0)
    assert received == sent
    assert received[0] > 0 and received[1] > 0


def test_spin_in_place_opposite_targets_reach_mcu():
    sent, received = _send_twist_get_mock_targets(0.0, 1.0)
    assert received == sent
    assert received[0] < 0 < received[1]


def test_forward_left_turn_targets_reach_mcu():
    sent, received = _send_twist_get_mock_targets(0.15, 0.5)
    assert received == sent
    # 左轉右輪較快。
    assert received[1] > received[0]


def test_reverse_targets_reach_mcu():
    sent, received = _send_twist_get_mock_targets(-0.1, 0.0)
    assert received == sent
    assert received[0] < 0 and received[1] < 0
