"""Migrated to vgr_core.protocol. Update callers to import from vgr_core.protocol instead."""
from vgr_core.model import ErrorCode, MCUState, MotorIntent
from vgr_core.protocol import (
    STATE_PACKET_LEN,
    STATE_PACKET_TYPE,
    StatePacket,
    checksum,
    decode_state,
    encode_state,
    HEADER,
    VERSION,
)

__all__ = [
    'STATE_PACKET_TYPE',
    'STATE_PACKET_LEN',
    'StatePacket',
    'encode_state',
    'decode_state',
    'MCUState',
    'MotorIntent',
    'ErrorCode',
    'HEADER',
    'VERSION',
    'checksum',
]
