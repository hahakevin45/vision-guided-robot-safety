from __future__ import annotations

import struct
from dataclasses import dataclass

from ..model.models import CommandID


HEADER = 0xA5
VERSION = 1


@dataclass(frozen=True)
class CommandPacket:
    """Host 傳給 MCU 的命令封包。"""

    sequence: int
    command: CommandID
    payload: bytes = b""


def checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def encode_command(packet: CommandPacket) -> bytes:
    """將命令封成 MCU 可解析的二進位格式。

    格式：
    [HEADER, VERSION, SEQ, COMMAND_ID, PAYLOAD_LEN, PAYLOAD..., CHECKSUM]
    checksum 使用簡單 sum modulo 256，方便 STM32 端用 C 快速實作。
    """

    if not 0 <= packet.sequence <= 255:
        raise ValueError("sequence must fit in one byte")
    if len(packet.payload) > 255:
        raise ValueError("payload too large")
    body = bytes(
        [
            HEADER,
            VERSION,
            packet.sequence,
            int(packet.command),
            len(packet.payload),
        ]
    ) + packet.payload
    return body + bytes([checksum(body)])


def decode_command(raw: bytes) -> CommandPacket:
    """解析命令封包；mock MCU 與測試用它驗證 host 端輸出。"""

    if len(raw) < 6:
        raise ValueError("packet too short")
    if raw[0] != HEADER:
        raise ValueError("bad header")
    if checksum(raw[:-1]) != raw[-1]:
        raise ValueError("bad checksum")
    payload_len = raw[4]
    expected_len = 6 + payload_len
    if len(raw) != expected_len:
        raise ValueError("bad length")
    try:
        command = CommandID(raw[3])
    except ValueError as exc:
        raise ValueError("invalid command") from exc
    return CommandPacket(sequence=raw[2], command=command, payload=raw[5:-1])


def encode_set_wheel_speed(sequence: int, left_cps: int, right_cps: int) -> bytes:
    """編碼 SET_WHEEL_SPEED 命令：payload 為兩個 little-endian signed int16 (left, right)。"""

    payload = struct.pack("<hh", left_cps, right_cps)
    packet = CommandPacket(sequence=sequence, command=CommandID.SET_WHEEL_SPEED, payload=payload)
    return encode_command(packet)


def decode_set_wheel_speed(packet: CommandPacket) -> tuple[int, int]:
    """解析 SET_WHEEL_SPEED payload，回傳 (left_cps, right_cps)。"""

    left_cps, right_cps = struct.unpack("<hh", packet.payload)
    return left_cps, right_cps
