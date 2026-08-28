import pytest

from vgr_core.model import ErrorCode, MCUState, MotorIntent
from vgr_core.protocol import STATE_PACKET_LEN, StatePacket, decode_state, encode_state


def test_state_packet_round_trip():
    raw = encode_state(
        StatePacket(sequence=3, state=MCUState.TRACKING, error=ErrorCode.OK, uptime_ms=1234)
    )
    packet = decode_state(raw)

    assert packet.sequence == 3
    assert packet.state == MCUState.TRACKING
    assert packet.error == ErrorCode.OK
    assert packet.motor_intent == MotorIntent.STOP
    assert packet.uptime_ms == 1234


def test_state_packet_round_trips_motor_intent():
    raw = encode_state(
        StatePacket(
            sequence=7,
            state=MCUState.TRACKING,
            error=ErrorCode.OK,
            motor_intent=MotorIntent.TURN_LEFT,
            uptime_ms=4321,
        )
    )
    packet = decode_state(raw)

    assert len(raw) == STATE_PACKET_LEN
    assert packet.motor_intent == MotorIntent.TURN_LEFT
    assert packet.uptime_ms == 4321


def test_state_packet_bad_checksum_rejected():
    raw = bytearray(
        encode_state(StatePacket(sequence=3, state=MCUState.SAFE_STOP, error=ErrorCode.OK))
    )
    raw[-1] ^= 0xFF

    with pytest.raises(ValueError, match="checksum"):
        decode_state(bytes(raw))
