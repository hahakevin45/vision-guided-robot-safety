from vgr_driver.driver import MockMCU
from vgr_core.model import CommandID, ErrorCode, MCUState, MotorIntent
from vgr_core.protocol import CommandPacket, encode_command


def test_mock_mcu_accepts_valid_sequence():
    mcu = MockMCU()
    response = mcu.receive(encode_command(CommandPacket(0, CommandID.FORWARD)))

    assert response.accepted
    assert response.error == ErrorCode.OK
    assert response.state == MCUState.TRACKING
    assert response.motor_intent == MotorIntent.FORWARD


def test_mock_mcu_rejects_sequence_gap():
    mcu = MockMCU()
    mcu.receive(encode_command(CommandPacket(0, CommandID.FORWARD)))
    response = mcu.receive(encode_command(CommandPacket(2, CommandID.FORWARD)))

    assert not response.accepted
    assert response.error == ErrorCode.BAD_SEQUENCE
    assert response.state == MCUState.SAFE_STOP
    assert response.motor_intent == MotorIntent.STOP


def test_mock_mcu_rejects_bad_checksum():
    mcu = MockMCU()
    raw = bytearray(encode_command(CommandPacket(0, CommandID.FORWARD)))
    raw[-1] ^= 0xFF
    response = mcu.receive(bytes(raw))

    assert not response.accepted
    assert response.error == ErrorCode.BAD_CHECKSUM


def test_mock_mcu_allows_heartbeat_zero_resync():
    mcu = MockMCU()
    mcu.receive(encode_command(CommandPacket(0, CommandID.FORWARD)))
    response = mcu.receive(encode_command(CommandPacket(0, CommandID.HEARTBEAT)))

    assert response.accepted
    assert response.error == ErrorCode.OK


def test_mock_mcu_reports_motor_intent_for_turn_and_stop():
    mcu = MockMCU()
    left = mcu.receive(encode_command(CommandPacket(0, CommandID.TURN_LEFT)))
    right = mcu.receive(encode_command(CommandPacket(1, CommandID.TURN_RIGHT)))
    stop = mcu.receive(encode_command(CommandPacket(2, CommandID.STOP)))

    assert left.motor_intent == MotorIntent.TURN_LEFT
    assert right.motor_intent == MotorIntent.TURN_RIGHT
    assert stop.motor_intent == MotorIntent.STOP
