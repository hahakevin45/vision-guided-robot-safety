import errno
import time

from vgr_driver.driver import PosixSerial


def test_read_exact_retries_blocking_io_error(monkeypatch):
    serial = PosixSerial("/dev/null", timeout_s=0.5)
    serial._fd = 123
    reads = iter([BlockingIOError(errno.EAGAIN, "again"), b"ok"])

    monkeypatch.setattr("select.select", lambda *_args: ([123], [], []))

    def fake_read(_fd, _size):
        item = next(reads)
        if isinstance(item, BaseException):
            raise item
        return item

    monkeypatch.setattr("os.read", fake_read)

    assert serial.read_exact(2) == b"ok"
