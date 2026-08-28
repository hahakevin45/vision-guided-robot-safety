from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_core.model import CommandID
from vgr_core.protocol import CommandPacket, encode_command

from vgr_core.protocol import ENCODER_PACKET_LEN, decode_encoder
from vgr_driver.driver.serial_transport import PosixSerial
from vgr_core.protocol import STATE_PACKET_LEN, decode_state


def main() -> int:
    parser = argparse.ArgumentParser(description="Diagnose a raw STM32/ESP32 serial link.")
    parser.add_argument("--device", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--command", choices=[c.name for c in CommandID], default="HEARTBEAT")
    parser.add_argument("--sequence", type=int, default=0)
    parser.add_argument("--read-bytes", type=int, default=None)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--probe-idle-s", type=float, default=0.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/serial_diagnostic.json"))
    args = parser.parse_args()

    raw_command = encode_command(
        CommandPacket(sequence=args.sequence & 0xFF, command=CommandID[args.command])
    )
    read_bytes = args.read_bytes
    if read_bytes is None:
        read_bytes = ENCODER_PACKET_LEN if args.command == "READ_ENCODERS" else STATE_PACKET_LEN
    result = {
        "pass": False,
        "device": args.device,
        "baudrate": args.baudrate,
        "command": args.command,
        "sequence": args.sequence & 0xFF,
        "tx_hex": raw_command.hex(" "),
        "rx_hex": "",
        "decoded_state": None,
        "decoded_encoder": None,
        "error": None,
        "notes": [],
    }

    try:
        with PosixSerial(args.device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if args.settle_s > 0:
                time.sleep(args.settle_s)
            if args.probe_idle_s > 0:
                idle_deadline = time.monotonic() + args.probe_idle_s
                idle_rx = bytearray()
                while time.monotonic() < idle_deadline:
                    idle_rx.extend(serial.read_available())
                    time.sleep(0.05)
                result["idle_rx_hex"] = bytes(idle_rx).hex(" ")
                serial.flush_input()
            serial.write(raw_command)
            try:
                raw_response = serial.read_exact(read_bytes)
                result["rx_hex"] = raw_response.hex(" ")
            except TimeoutError as exc:
                result["error"] = str(exc)
                result["notes"].append("Port opened and command was written, but no full response was received.")
                result["notes"].append("Check that MCU firmware reads 6-byte command packets and writes the expected response packet.")
                raise

        if len(bytes.fromhex(result["rx_hex"])) == STATE_PACKET_LEN:
            state = decode_state(bytes.fromhex(result["rx_hex"]))
            result["decoded_state"] = {
                "sequence": state.sequence,
                "state": state.state.name,
                "error": state.error.name,
                "motor_intent": state.motor_intent.name,
                "uptime_ms": state.uptime_ms,
            }
            result["pass"] = True
        elif len(bytes.fromhex(result["rx_hex"])) == ENCODER_PACKET_LEN:
            encoder = decode_encoder(bytes.fromhex(result["rx_hex"]))
            result["decoded_encoder"] = {
                "sequence": encoder.sequence,
                "left_count": encoder.left_count,
                "right_count": encoder.right_count,
                "flags": encoder.flags,
            }
            result["pass"] = True
    except Exception as exc:  # noqa: BLE001 - CLI should preserve hardware failure details.
        if result["error"] is None:
            result["error"] = str(exc)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if result["pass"]:
        print("SERIAL DIAGNOSTIC: PASS")
        return 0
    print("SERIAL DIAGNOSTIC: FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
