from __future__ import annotations

from dataclasses import dataclass

from .host_codec import HEADER, VERSION, checksum


ENCODER_PACKET_TYPE = 0x81
ENCODER_PACKET_LEN = 14


@dataclass(frozen=True)
class EncoderPacket:
    """MCU 回傳給 host 的左右輪編碼器快照。"""

    sequence: int
    left_count: int
    right_count: int
    flags: int = 0


def encode_encoder(packet: EncoderPacket) -> bytes:
    """將 encoder snapshot 編成固定 14-byte telemetry packet。"""

    if not 0 <= packet.sequence <= 255:
        raise ValueError("sequence must fit in one byte")
    if not 0 <= packet.flags <= 255:
        raise ValueError("flags must fit in one byte")
    body = bytes([HEADER, VERSION, packet.sequence, ENCODER_PACKET_TYPE])
    body += int(packet.left_count).to_bytes(4, "little", signed=True)
    body += int(packet.right_count).to_bytes(4, "little", signed=True)
    body += bytes([packet.flags])
    return body + bytes([checksum(body)])


def decode_encoder(raw: bytes) -> EncoderPacket:
    """解析 STM32 回傳的 encoder snapshot packet。"""

    if len(raw) != ENCODER_PACKET_LEN:
        raise ValueError("bad encoder packet length")
    if raw[0] != HEADER:
        raise ValueError("bad header")
    if raw[1] != VERSION:
        raise ValueError("bad version")
    if raw[3] != ENCODER_PACKET_TYPE:
        raise ValueError("bad packet type")
    if checksum(raw[:-1]) != raw[-1]:
        raise ValueError("bad checksum")
    return EncoderPacket(
        sequence=raw[2],
        left_count=int.from_bytes(raw[4:8], "little", signed=True),
        right_count=int.from_bytes(raw[8:12], "little", signed=True),
        flags=raw[12],
    )
