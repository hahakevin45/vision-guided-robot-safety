import os
import pty

from vgr_core.model import CommandID
from vgr_driver.driver import ControllerBridge
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver import PosixSerial


def test_bridge_reads_encoder_snapshot_from_mock_serial_mcu():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    mock = MockSerialMCU(master_fd)
    mock.start()
    try:
        with PosixSerial(slave_name, baudrate=115200, timeout_s=1.0) as serial:
            bridge = ControllerBridge(serial)
            heartbeat = bridge.send_command(CommandID.HEARTBEAT)
            snapshot = bridge.read_encoders()
    finally:
        mock.stop()
        os.close(master_fd)
        os.close(slave_fd)

    assert heartbeat.state.sequence == 0
    assert snapshot.command == CommandID.READ_ENCODERS
    assert snapshot.sequence == 1
    assert snapshot.packet.sequence == 1
    assert snapshot.packet.left_count == 0
    assert snapshot.packet.right_count == 0
