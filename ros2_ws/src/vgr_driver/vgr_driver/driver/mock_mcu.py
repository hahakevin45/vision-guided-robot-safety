"""Python mock MCU for testing host <-> MCU protocol without real hardware."""
from __future__ import annotations

from time import monotonic

from vgr_core.model import CommandID, ErrorCode, MCUResponse, MCUState, MotorIntent
from vgr_core.protocol import decode_command


class MockMCU:
    """Simulates MCU command parsing and state machine for protocol testing."""

    def __init__(self, command_timeout_s: float = 0.5) -> None:
        self.command_timeout_s = command_timeout_s
        self.state = MCUState.IDLE
        self.motor_intent = MotorIntent.STOP
        self._last_sequence: int | None = None
        self._last_command_ts: float | None = None
        self._forced_fault = False

    def force_fault(self) -> None:
        """Force MCU into FAULT state (for testing)."""
        self.state = MCUState.FAULT
        self.motor_intent = MotorIntent.STOP

    def receive(self, raw_packet: bytes, timestamp: float | None = None) -> MCUResponse:
        """Handle incoming raw command packet, return MCUResponse."""
        start = monotonic()
        event_ts = monotonic() if timestamp is None else timestamp
        if self._forced_fault:
            return self._response(ErrorCode.FORCED_FAULT, 0, False, "forced fault", start)

        try:
            packet = decode_command(raw_packet)
        except ValueError as exc:
            return self._response(_map_decode_error(str(exc)), 0, False, str(exc), start)

        if packet.command == CommandID.HEARTBEAT and packet.sequence == 0:
            self._last_sequence = None

        if self._last_sequence is not None:
            expected = (self._last_sequence + 1) & 0xFF
            if packet.sequence != expected:
                self.state = MCUState.SAFE_STOP
                self.motor_intent = MotorIntent.STOP
                self._last_sequence = packet.sequence
                return self._response(
                    ErrorCode.BAD_SEQUENCE,
                    packet.sequence,
                    False,
                    f"expected sequence {expected}, got {packet.sequence}",
                    start,
                )

        self._last_sequence = packet.sequence
        self._last_command_ts = event_ts
        self._apply_command(packet.command)
        return self._response(ErrorCode.OK, packet.sequence, True, "accepted", start)

    def tick(self, timestamp: float | None = None) -> MCUResponse | None:
        """Simulate periodic MCU check for command timeout."""
        if self._last_command_ts is None or self.state == MCUState.FAULT:
            return None
        event_ts = monotonic() if timestamp is None else timestamp
        elapsed = event_ts - self._last_command_ts
        if elapsed <= self.command_timeout_s:
            return None
        self.state = MCUState.SAFE_STOP
        self.motor_intent = MotorIntent.STOP
        return MCUResponse(
            state=self.state,
            error=ErrorCode.COMMAND_TIMEOUT,
            sequence=self._last_sequence or 0,
            accepted=False,
            message=f"command timeout after {elapsed:.3f}s",
            latency_ms=0.0,
            motor_intent=self.motor_intent,
        )

    def _apply_command(self, command: CommandID) -> None:
        """Update mock MCU state based on command."""
        if command == CommandID.STOP:
            self.state = MCUState.SAFE_STOP
            self.motor_intent = MotorIntent.STOP
        elif command == CommandID.HEARTBEAT:
            if self.state == MCUState.IDLE:
                self.state = MCUState.ARMED
            self.motor_intent = MotorIntent.STOP
        elif command == CommandID.FORWARD:
            self.state = MCUState.TRACKING
            self.motor_intent = MotorIntent.FORWARD
        elif command == CommandID.TURN_LEFT:
            self.state = MCUState.TRACKING
            self.motor_intent = MotorIntent.TURN_LEFT
        elif command == CommandID.TURN_RIGHT:
            self.state = MCUState.TRACKING
            self.motor_intent = MotorIntent.TURN_RIGHT
        elif command == CommandID.READ_ENCODERS:
            if self.state == MCUState.IDLE:
                self.state = MCUState.ARMED
        else:
            self.state = MCUState.TRACKING

    def _response(
        self,
        error: ErrorCode,
        sequence: int,
        accepted: bool,
        message: str,
        start: float,
    ) -> MCUResponse:
        """Build MCUResponse with mock processing delay."""
        return MCUResponse(
            state=self.state,
            error=error,
            sequence=sequence,
            accepted=accepted,
            message=message,
            latency_ms=(monotonic() - start) * 1000.0,
            motor_intent=self.motor_intent,
        )


def _map_decode_error(message: str) -> ErrorCode:
    """Map protocol parser errors to MCU error codes."""
    if "header" in message:
        return ErrorCode.BAD_HEADER
    if "checksum" in message:
        return ErrorCode.BAD_CHECKSUM
    if "command" in message:
        return ErrorCode.INVALID_COMMAND
    return ErrorCode.INVALID_COMMAND
