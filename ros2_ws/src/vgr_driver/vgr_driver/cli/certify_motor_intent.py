from __future__ import annotations

import argparse
import json
import os
import pty
import time
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode, MotorIntent

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver.serial_transport import PosixSerial


EXPECTED_INTENTS = {
    CommandID.FORWARD: MotorIntent.FORWARD,
    CommandID.TURN_LEFT: MotorIntent.TURN_LEFT,
    CommandID.TURN_RIGHT: MotorIntent.TURN_RIGHT,
    CommandID.STOP: MotorIntent.STOP,
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Certify Phase 2 dry-run motor intent telemetry.")
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/motor_intent_certification.json"))
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
        "checks": {},
        "exchanges": [],
        "error": None,
    }

    try:
        with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if not using_pty and args.settle_s > 0:
                time.sleep(args.settle_s)
                serial.flush_input()
            bridge = ControllerBridge(serial)
            heartbeat = bridge.send_command(CommandID.HEARTBEAT)
            result["exchanges"].append(describe_exchange(heartbeat, MotorIntent.STOP))
            for command, expected in EXPECTED_INTENTS.items():
                exchange = bridge.send_command(command)
                result["exchanges"].append(describe_exchange(exchange, expected))

        result["checks"] = {
            "opened_serial_device": True,
            "all_errors_ok": all(exchange["mcu_error"] == ErrorCode.OK.name for exchange in result["exchanges"]),
            "sequence_echo_ok": all(
                exchange["sequence"] == exchange["state_sequence"] for exchange in result["exchanges"]
            ),
            "motor_intents_match": all(
                exchange["motor_intent"] == exchange["expected_motor_intent"]
                for exchange in result["exchanges"]
            ),
        }
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - hardware certification preserves failure detail.
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
        print("MOTOR INTENT CERTIFICATION: PASS")
        return 0
    print("MOTOR INTENT CERTIFICATION: FAIL")
    return 1


def describe_exchange(exchange, expected: MotorIntent) -> dict:
    return {
        "command": exchange.command.name,
        "sequence": exchange.sequence,
        "state_sequence": exchange.state.sequence,
        "mcu_state": exchange.state.state.name,
        "mcu_error": exchange.state.error.name,
        "motor_intent": exchange.state.motor_intent.name,
        "expected_motor_intent": expected.name,
        "latency_ms": exchange.latency_ms,
    }


if __name__ == "__main__":
    raise SystemExit(main())
