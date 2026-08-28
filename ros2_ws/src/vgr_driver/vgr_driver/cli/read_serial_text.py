from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_driver.driver.serial_transport import PosixSerial


def main() -> int:
    parser = argparse.ArgumentParser(description="Read raw serial text for UART smoke tests.")
    parser.add_argument("--device", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/serial_text_read.json"))
    args = parser.parse_args()

    result = {
        "pass": False,
        "device": args.device,
        "baudrate": args.baudrate,
        "duration_s": args.duration_s,
        "rx_hex": "",
        "rx_text": "",
        "error": None,
    }

    try:
        with PosixSerial(args.device, baudrate=args.baudrate, timeout_s=0.2) as serial:
            deadline = time.monotonic() + args.duration_s
            data = bytearray()
            while time.monotonic() < deadline:
                data.extend(serial.read_available(512))
                time.sleep(0.05)
        result["rx_hex"] = bytes(data).hex(" ")
        result["rx_text"] = bytes(data).decode("utf-8", errors="replace")
        result["pass"] = len(data) > 0
    except Exception as exc:  # noqa: BLE001 - hardware diagnostics should preserve details.
        result["error"] = str(exc)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if result["pass"]:
        print("SERIAL TEXT READ: PASS")
        return 0
    print("SERIAL TEXT READ: FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
