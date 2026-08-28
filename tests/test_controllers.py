from vgr_driver.driver import MockController
from vgr_core.model import CommandID, ErrorCode, MCUState


def test_mock_controller_resync_and_send():
    controller = MockController(command_timeout_s=0.5)

    resync = controller.resync()
    response = controller.send(CommandID.FORWARD)

    assert resync is not None
    assert resync.error == ErrorCode.OK
    assert resync.state == MCUState.ARMED
    assert response.error == ErrorCode.OK
    assert response.state == MCUState.TRACKING
