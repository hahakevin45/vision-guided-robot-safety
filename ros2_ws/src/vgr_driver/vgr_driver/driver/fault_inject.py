"""Serial fault injector for firmware fault drills.

Only actuator command frames are modified. Encoder requests and all reads
stay on the original serial path so the host watchdog is not part of the
injected fault.
"""
from __future__ import annotations

import os
import time
from typing import Protocol

from vgr_core.model import CommandID
from vgr_core.protocol import HEADER


class WriteProtocol(Protocol):
    def write(self, data: bytes) -> int: ...


class FaultInjectingSerial:
    MODES = {"none", "bad_checksum", "garbage"}

    def __init__(
        self,
        serial_port: WriteProtocol,
        *,
        mode: str = "none",
        at_s: float = -1.0,
        count: int = 10,
        clock=time.monotonic,
        logger=None,
    ) -> None:
        if mode not in self.MODES:
            raise ValueError(f"fault_inject_mode must be one of {sorted(self.MODES)}")
        if count < 0:
            raise ValueError("fault_inject_count must be non-negative")
        self.serial_port = serial_port
        self.mode = mode
        self.at_s = float(at_s)
        self.remaining = int(count)
        self._clock = clock
        self._started_s = clock()
        self._logger = logger
        self.injection_started_s: float | None = None

    def write(self, data: bytes, *, now_s: float | None = None) -> None:
        raw = bytes(data)
        elapsed_s = (
            float(now_s)
            if now_s is not None
            else self._clock() - self._started_s
        )
        outgoing = raw
        if self._eligible(raw) and self._due(elapsed_s):
            if self.mode == "bad_checksum":
                corrupted = bytearray(raw)
                corrupted[-1] ^= 0xFF
                outgoing = bytes(corrupted)
            elif self.mode == "garbage":
                outgoing = os.urandom(len(raw))
            self.remaining -= 1
            if self.injection_started_s is None:
                self.injection_started_s = elapsed_s
                if self._logger is not None:
                    self._logger.info(
                        "fault injection started: "
                        f"mode={self.mode} at_s={elapsed_s:.3f} count={self.remaining + 1}"
                    )
        self.serial_port.write(outgoing)

    def __getattr__(self, name: str):
        return getattr(self.serial_port, name)

    def _due(self, elapsed_s: float) -> bool:
        return (
            self.mode != "none"
            and self.at_s >= 0.0
            and elapsed_s >= self.at_s
            and self.remaining > 0
        )

    @staticmethod
    def _eligible(raw: bytes) -> bool:
        return (
            len(raw) >= 6
            and raw[0] == HEADER
            and raw[3] != int(CommandID.READ_ENCODERS)
        )
