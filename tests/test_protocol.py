import pytest

from vgr_core.model import CommandID
from vgr_core.protocol import (
    CommandPacket,
    decode_command,
    decode_set_wheel_speed,
    encode_command,
    encode_set_wheel_speed,
)


def test_round_trip_command_packet():
    raw = encode_command(CommandPacket(sequence=7, command=CommandID.TURN_LEFT))
    packet = decode_command(raw)

    assert packet.sequence == 7
    assert packet.command == CommandID.TURN_LEFT


def test_bad_checksum_rejected():
    raw = bytearray(encode_command(CommandPacket(sequence=7, command=CommandID.STOP)))
    raw[-1] ^= 0xFF

    with pytest.raises(ValueError, match="checksum"):
        decode_command(bytes(raw))


def test_round_trip_set_wheel_speed():
    raw = encode_set_wheel_speed(3, 749, -749)
    packet = decode_command(raw)

    assert packet.sequence == 3
    assert packet.command == CommandID.SET_WHEEL_SPEED
    assert decode_set_wheel_speed(packet) == (749, -749)
