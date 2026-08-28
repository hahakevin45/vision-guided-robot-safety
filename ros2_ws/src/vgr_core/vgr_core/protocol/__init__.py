"""Protocol codecs for host <-> MCU serial communication."""
from __future__ import annotations

from ..model.models import ErrorCode, MCUState, MotorIntent
from .encoder_protocol import (
    ENCODER_PACKET_LEN,
    ENCODER_PACKET_TYPE,
    EncoderPacket,
    decode_encoder,
    encode_encoder,
)
from .host_codec import (
    HEADER,
    VERSION,
    CommandPacket,
    checksum,
    decode_command,
    decode_set_wheel_speed,
    encode_command,
    encode_set_wheel_speed,
)
from .state_protocol import (
    STATE_PACKET_LEN,
    STATE_PACKET_TYPE,
    StatePacket,
    decode_state,
    encode_state,
)

__all__ = [
    'HEADER',
    'VERSION',
    'checksum',
    'CommandPacket',
    'encode_command',
    'decode_command',
    'encode_set_wheel_speed',
    'decode_set_wheel_speed',
    'STATE_PACKET_TYPE',
    'STATE_PACKET_LEN',
    'StatePacket',
    'encode_state',
    'decode_state',
    'ENCODER_PACKET_TYPE',
    'ENCODER_PACKET_LEN',
    'EncoderPacket',
    'encode_encoder',
    'decode_encoder',
    # re-exported from model for convenience
    'MCUState',
    'MotorIntent',
    'ErrorCode',
]
