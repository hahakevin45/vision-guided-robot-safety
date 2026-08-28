"""Migrated to vgr_core.protocol. Update callers to import from vgr_core.protocol instead."""
from vgr_core.protocol import (
    ENCODER_PACKET_LEN,
    ENCODER_PACKET_TYPE,
    EncoderPacket,
    checksum,
    decode_encoder,
    encode_encoder,
    HEADER,
    VERSION,
)

__all__ = [
    'ENCODER_PACKET_TYPE',
    'ENCODER_PACKET_LEN',
    'EncoderPacket',
    'encode_encoder',
    'decode_encoder',
    'HEADER',
    'VERSION',
    'checksum',
]
