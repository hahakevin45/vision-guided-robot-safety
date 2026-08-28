from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

from vgr_core.model import CommandID
from vgr_core.protocol import (
    CommandPacket,
    ENCODER_PACKET_LEN,
    ENCODER_PACKET_TYPE,
    EncoderPacket,
    STATE_PACKET_LEN,
    StatePacket,
    checksum,
    decode_encoder,
    decode_state,
    encode_command,
    encode_set_wheel_speed,
    HEADER,
    VERSION,
)
from .serial_transport import PosixSerial


@dataclass(frozen=True)
class BridgeExchange:
    """一次 host-to-MCU 交換的結果。"""

    command: CommandID
    sequence: int
    state: StatePacket
    latency_ms: float


@dataclass(frozen=True)
class EncoderExchange:
    """一次 encoder snapshot 讀取結果。"""

    command: CommandID
    sequence: int
    packet: EncoderPacket
    latency_ms: float


class ControllerBridge:
    """Phase 2 的核心邊界：送 command packet，收 STM32 state packet。"""

    def __init__(self, serial_port: PosixSerial) -> None:
        self.serial_port = serial_port
        self.sequence = 0

    def send_command(self, command: CommandID) -> BridgeExchange:
        """送出一筆命令並等待 MCU 回傳狀態。

        這裡量到的是 host serial write 到 state packet read 完成的 round-trip latency。
        sequence 每次遞增，讓 STM32 可以偵測漏包、重送或 host 重新同步。
        """

        sequence = self.sequence
        raw = encode_command(CommandPacket(sequence=sequence, command=command))
        start = monotonic()
        self.serial_port.write(raw)
        state = decode_state(self.serial_port.read_exact(STATE_PACKET_LEN))
        latency_ms = (monotonic() - start) * 1000.0
        self.sequence = (self.sequence + 1) & 0xFF
        return BridgeExchange(
            command=command,
            sequence=sequence,
            state=state,
            latency_ms=latency_ms,
        )

    def send_set_wheel_speed(self, left_cps: int, right_cps: int) -> BridgeExchange:
        """送出 SET_WHEEL_SPEED 命令 (兩個 signed int16 counts/s)，等待 MCU state 回應。"""

        sequence = self.sequence
        raw = encode_set_wheel_speed(sequence, left_cps, right_cps)
        start = monotonic()
        self.serial_port.write(raw)
        state = decode_state(self.serial_port.read_exact(STATE_PACKET_LEN))
        latency_ms = (monotonic() - start) * 1000.0
        self.sequence = (self.sequence + 1) & 0xFF
        return BridgeExchange(
            command=CommandID.SET_WHEEL_SPEED,
            sequence=sequence,
            state=state,
            latency_ms=latency_ms,
        )

    def read_encoders(self) -> EncoderExchange:
        """要求 STM32 回傳左右輪 encoder 計數快照。"""

        sequence = self.sequence
        raw = encode_command(
            CommandPacket(sequence=sequence, command=CommandID.READ_ENCODERS)
        )
        start = monotonic()
        self.serial_port.write(raw)
        packet = decode_encoder(self.serial_port.read_exact(ENCODER_PACKET_LEN))
        latency_ms = (monotonic() - start) * 1000.0
        self.sequence = (self.sequence + 1) & 0xFF
        return EncoderExchange(
            command=CommandID.READ_ENCODERS,
            sequence=sequence,
            packet=packet,
            latency_ms=latency_ms,
        )
