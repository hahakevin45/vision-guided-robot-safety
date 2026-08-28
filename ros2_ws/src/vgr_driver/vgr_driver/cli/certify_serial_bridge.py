from __future__ import annotations

import argparse
import json
import os
import pty
import time
from collections import Counter
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver.serial_transport import PosixSerial


def main() -> int:
    parser = argparse.ArgumentParser(description="Certify Phase 2 serial bridge.")
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/serial_bridge_certification.json"))
    parser.add_argument(
        "--commands",
        nargs="+",
        default=["HEARTBEAT", "FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"],
        choices=[command.name for command in CommandID if command != CommandID.READ_ENCODERS],
    )
    args = parser.parse_args()

    master_fd = None
    slave_fd = None
    mock_mcu = None
    device = args.device
    using_pty = device is None
    if using_pty:
        master_fd, slave_fd = pty.openpty()
        device = os.ttyname(slave_fd)
        mock_mcu = MockSerialMCU(master_fd, timeout_s=args.timeout_s)
        mock_mcu.start()

    result = {
        "pass": False,
        "device": device,
        "using_pty_mock_mcu": using_pty,
        "commands_requested": args.commands,
        "exchanges": [],
        "checks": {},
        "error": None,
    }

    try:
        with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if not using_pty and args.settle_s > 0:
                time.sleep(args.settle_s)
                serial.flush_input()
            bridge = ControllerBridge(serial)
            for command_name in args.commands:
                exchange = bridge.send_command(CommandID[command_name])
                result["exchanges"].append(
                    {
                        "command": exchange.command.name,
                        "sequence": exchange.sequence,
                        "state_sequence": exchange.state.sequence,
                        "mcu_state": exchange.state.state.name,
                        "mcu_error": exchange.state.error.name,
                        "motor_intent": exchange.state.motor_intent.name,
                        "latency_ms": exchange.latency_ms,
                    }
                )

        errors = Counter(exchange["mcu_error"] for exchange in result["exchanges"])
        sequence_ok = all(
            exchange["sequence"] == exchange["state_sequence"]
            for exchange in result["exchanges"]
        )
        all_ok = errors == Counter({"OK": len(result["exchanges"])})
        read_all = len(result["exchanges"]) == len(args.commands)
        result["checks"] = {
            "opened_serial_device": True,
            "read_all_state_packets": read_all,
            "sequence_echo_ok": sequence_ok,
            "mcu_errors_ok": all_ok,
        }
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - CLI must report device failures clearly.
        result["error"] = str(exc)
    finally:
        if mock_mcu is not None:
            mock_mcu.stop()
        if master_fd is not None:
            os.close(master_fd)
        if slave_fd is not None:
            os.close(slave_fd)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if result["pass"]:
        print("SERIAL BRIDGE CERTIFICATION: PASS")
        return 0
    print("SERIAL BRIDGE CERTIFICATION: FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
