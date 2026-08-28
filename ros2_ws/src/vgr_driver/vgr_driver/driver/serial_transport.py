from __future__ import annotations

import os
import select
import termios
import time
from dataclasses import dataclass
from errno import EAGAIN, EWOULDBLOCK


BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


@dataclass
class SerialStats:
    bytes_written: int = 0
    bytes_read: int = 0
    read_timeouts: int = 0
    reconnects: int = 0


class PosixSerial:
    def __init__(self, device: str, baudrate: int = 115200, timeout_s: float = 0.5) -> None:
        self.device = device
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.stats = SerialStats()
        self._fd: int | None = None

    def __enter__(self) -> "PosixSerial":
        self.open()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def open(self) -> None:
        if self._fd is not None:
            return
        self._fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._configure(self._fd)

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def write(self, data: bytes) -> None:
        fd = self._require_fd()
        written = 0
        while written < len(data):
            _, writable, _ = select.select([], [fd], [], self.timeout_s)
            if not writable:
                raise TimeoutError("serial write timeout")
            chunk = os.write(fd, data[written:])
            written += chunk
            self.stats.bytes_written += chunk

    def read_exact(self, size: int) -> bytes:
        fd = self._require_fd()
        deadline = time.monotonic() + self.timeout_s
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.stats.read_timeouts += 1
                raise TimeoutError(f"serial read timeout after {len(data)}/{size} bytes")
            readable, _, _ = select.select([fd], [], [], remaining)
            if not readable:
                self.stats.read_timeouts += 1
                raise TimeoutError(f"serial read timeout after {len(data)}/{size} bytes")
            try:
                chunk = os.read(fd, size - len(data))
            except BlockingIOError as exc:
                if exc.errno in (EAGAIN, EWOULDBLOCK):
                    continue
                raise
            if not chunk:
                continue
            data.extend(chunk)
            self.stats.bytes_read += len(chunk)
        return bytes(data)

    def read_available(self, max_bytes: int = 256) -> bytes:
        fd = self._require_fd()
        readable, _, _ = select.select([fd], [], [], 0.0)
        if not readable:
            return b""
        try:
            data = os.read(fd, max_bytes)
        except BlockingIOError as exc:
            if exc.errno in (EAGAIN, EWOULDBLOCK):
                return b""
            raise
        self.stats.bytes_read += len(data)
        return data

    def flush_input(self) -> None:
        termios.tcflush(self._require_fd(), termios.TCIFLUSH)

    def _configure(self, fd: int) -> None:
        attrs = termios.tcgetattr(fd)
        baud = BAUD_RATES.get(self.baudrate)
        if baud is None:
            raise ValueError(f"unsupported baudrate: {self.baudrate}")

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud
        attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)

    def _require_fd(self) -> int:
        if self._fd is None:
            raise RuntimeError("serial port is not open")
        return self._fd
