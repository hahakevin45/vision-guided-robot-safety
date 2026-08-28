import os
import pty

from vgr_core.model import CommandID, ErrorCode, MCUState
from vgr_driver.driver import ControllerBridge
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver import PosixSerial


def test_serial_bridge_round_trip_with_pty_mock_mcu():
    master_fd, slave_fd = pty.openpty()
    mock = MockSerialMCU(master_fd)
    mock.start()
    try:
        device = os.ttyname(slave_fd)
        with PosixSerial(device=device, timeout_s=1.0) as serial:
            bridge = ControllerBridge(serial)
            exchange = bridge.send_command(CommandID.FORWARD)

        assert exchange.sequence == 0
        assert exchange.state.sequence == 0
        assert exchange.state.state == MCUState.TRACKING
        assert exchange.state.error == ErrorCode.OK
    finally:
        mock.stop()
        os.close(master_fd)
        os.close(slave_fd)
