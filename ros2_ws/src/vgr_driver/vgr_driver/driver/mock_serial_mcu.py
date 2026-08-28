"""MockSerialMCU: PTY-backed MCU simulator for serial bridge testing.

This module is NOT the same as vgr_driver.driver.MockMCU.
That one is a pure in-process mock for the Phase 1 pipeline.
This one wraps a pty pair and implements the wire-level protocol so the
real ControllerBridge can be exercised end-to-end.
"""
from __future__ import annotations

import os
import select
import threading
import time
from errno import EIO
from dataclasses import dataclass

from vgr_driver.driver import MockMCU
from vgr_core.model import CommandID
from vgr_core.protocol import decode_command, decode_set_wheel_speed, ENCODER_PACKET_LEN, EncoderPacket, encode_encoder, StatePacket, encode_state


COMMAND_PACKET_LEN = 6


@dataclass
class MockSerialMCUStats:
    commands_seen: int = 0
    responses_written: int = 0
    decode_errors: int = 0


class MockSerialMCU:
    def __init__(self, fd: int, timeout_s: float = 0.5) -> None:
        self.fd = fd
        self._timeout_s = timeout_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_advance_ts = time.monotonic()
        self.mcu = MockMCU(command_timeout_s=timeout_s)
        self.stats = MockSerialMCUStats()

        # encoder state (exposed for tests to read)
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_target_cps = 0
        self.right_target_cps = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        buffer = bytearray()
        while not self._stop.is_set():
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                continue
            try:
                data = os.read(self.fd, 256)
            except OSError as exc:
                if exc.errno == EIO:
                    break
                raise
            if not data:
                continue
            buffer.extend(data)
            # Commands are variable length: COMMAND_PACKET_LEN + payload_len.
            # SET_WHEEL_SPEED carries a 4-byte payload, so a fixed-width slice
            # would split the frame and fail the checksum.
            while len(buffer) >= COMMAND_PACKET_LEN:
                packet_len = COMMAND_PACKET_LEN + buffer[4]
                if len(buffer) < packet_len:
                    break
                raw = bytes(buffer[:packet_len])
                del buffer[:packet_len]
                self._handle(raw)

    def _handle(self, raw: bytes) -> None:
        self.stats.commands_seen += 1
        timestamp = time.monotonic()

        # Decode command
        try:
            packet = decode_command(raw)
        except ValueError:
            # Drop the corrupt frame; a decode error must not kill the reader.
            self.stats.decode_errors += 1
            return

        # Route by command type
        if packet.command == CommandID.READ_ENCODERS:
            self._handle_read_encoders(packet, raw, timestamp)
        elif packet.command == CommandID.SET_WHEEL_SPEED:
            try:
                left_cps, right_cps = decode_set_wheel_speed(packet)
                self.left_target_cps = left_cps
                self.right_target_cps = right_cps
            except Exception:
                pass  # best-effort: keep previous targets
            self._respond_state(raw, timestamp)
            self._advance_targeted_encoders()
        else:
            # STOP, TURN_LEFT, TURN_RIGHT, HEARTBEAT
            self._respond_state(raw, timestamp)
            # _respond_state refreshes motor_intent; advance against it.
            self._advance_mock_encoders()

    def _handle_read_encoders(
        self, packet, raw: bytes, timestamp: float
    ) -> None:
        """Advance the MCU sequence, then return the encoder wire packet."""
        self.mcu.receive(raw, timestamp=timestamp)
        self._advance_targeted_encoders()
        encoder_packet = EncoderPacket(
            sequence=packet.sequence,
            left_count=self.left_encoder_count,
            right_count=self.right_encoder_count,
            flags=0,
        )
        out = encode_encoder(encoder_packet)
        try:
            os.write(self.fd, out)
            self.stats.responses_written += 1
        except OSError:
            pass

    def _respond_state(self, raw: bytes, timestamp: float) -> None:
        """Send a state response for non-encoder commands."""
        response = self.mcu.receive(raw, timestamp=timestamp)
        self._write_state_response(response, timestamp)

    def _write_state_response(self, response, timestamp: float) -> None:
        state = response.state
        state_packet = StatePacket(
            sequence=response.sequence,
            state=state,
            error=response.error,
            motor_intent=response.motor_intent,
            uptime_ms=0,
        )
        out = encode_state(state_packet)
        try:
            os.write(self.fd, out)
            self.stats.responses_written += 1
        except OSError:
            pass

    def _advance_mock_encoders(self) -> None:
        """Advance encoders one fixed step per intent-driven command.

        Deliberately not time-integrated: bench CLIs assert on which wheel
        moved, not how far, and must not depend on scheduling jitter. This
        also leaves _last_advance_ts untouched so targeted (SET_WHEEL_SPEED)
        integration keeps an accurate dt.
        """
        intent = self.mcu.motor_intent.name
        if intent == "FORWARD":
            self.left_encoder_count += 10
            self.right_encoder_count += 10
        elif intent == "TURN_LEFT":
            self.right_encoder_count += 10
        elif intent == "TURN_RIGHT":
            self.left_encoder_count += 10

    def _advance_targeted_encoders(self) -> None:
        now = time.monotonic()
        dt = now - self._last_advance_ts
        self._last_advance_ts = now
        self.left_encoder_count += int(self.left_target_cps * dt)
        self.right_encoder_count += int(self.right_target_cps * dt)
