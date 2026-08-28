from __future__ import annotations

from dataclasses import dataclass

from ..model.models import ErrorCode, MCUState, MotorIntent
from .host_codec import HEADER, VERSION, checksum


STATE_PACKET_TYPE = 0x80
STATE_PACKET_LEN = 10


@dataclass(frozen=True)
class StatePacket:
    """MCU 回傳給 host 的狀態封包內容。"""

    sequence: int
    state: MCUState
    error: ErrorCode
    motor_intent: MotorIntent = MotorIntent.STOP
    uptime_ms: int = 0


def encode_state(packet: StatePacket) -> bytes:
    """將 MCU 狀態編成二進位封包；mock MCU 與韌體規格共用這個格式。"""

    if not 0 <= packet.sequence <= 255:
        raise ValueError("sequence must fit in one byte")
    if not 0 <= packet.uptime_ms <= 0xFFFF:
        raise ValueError("uptime_ms must fit in two bytes")
    body = bytes(
        [
            HEADER,
            VERSION,
            packet.sequence,
            STATE_PACKET_TYPE,
            int(packet.state),
            int(packet.error),
            int(packet.motor_intent),
            packet.uptime_ms & 0xFF,
            (packet.uptime_ms >> 8) & 0xFF,
        ]
    )
    return body + bytes([checksum(body)])


def decode_state(raw: bytes) -> StatePacket:
    """解析 STM32 回傳的 10-byte state packet。"""

    if len(raw) != STATE_PACKET_LEN:
        raise ValueError("bad state packet length")
    if raw[0] != HEADER:
        raise ValueError("bad header")
    if raw[1] != VERSION:
        raise ValueError("bad version")
    if raw[3] != STATE_PACKET_TYPE:
        raise ValueError("bad packet type")
    if checksum(raw[:-1]) != raw[-1]:
        raise ValueError("bad checksum")
    try:
        state = MCUState(raw[4])
        error = ErrorCode(raw[5])
        motor_intent = MotorIntent(raw[6])
    except ValueError as exc:
        raise ValueError("bad state payload") from exc
    uptime_ms = raw[7] | (raw[8] << 8)
    return StatePacket(
        sequence=raw[2],
        state=state,
        error=error,
        motor_intent=motor_intent,
        uptime_ms=uptime_ms,
    )
