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


STEPS = [
    (CommandID.HEARTBEAT, MotorIntent.STOP, "resync; motor must be stopped"),
    (CommandID.FORWARD, MotorIntent.FORWARD, "single connected wheel may rotate at bench duty"),
    (CommandID.STOP, MotorIntent.STOP, "wheel must stop immediately"),
    (CommandID.FORWARD, MotorIntent.FORWARD, "timeout observation: wheel should stop after firmware timeout if commands stop"),
    (CommandID.STOP, MotorIntent.STOP, "final explicit stop"),
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Check motor STOP and timeout safety during bench testing.")
    parser.add_argument("--device", default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--step-s", type=float, default=0.5)
    parser.add_argument("--timeout-observe-s", type=float, default=1.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/motor_safety_certification.json"))
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
        "step_s": args.step_s,
        "timeout_observe_s": args.timeout_observe_s,
        "manual_confirmation_required": True,
        "checks": {},
        "steps": [],
        "error": None,
    }

    try:
        with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if not using_pty:
                time.sleep(args.settle_s)
                serial.flush_input()
            bridge = ControllerBridge(serial)
            for command, expected_intent, observation in STEPS:
                print(f"{command.name}: {observation}")
                exchange = bridge.send_command(command)
                result["steps"].append(describe_exchange(exchange, expected_intent, observation))
                if command == CommandID.FORWARD and "timeout observation" in observation:
                    print(f"NO COMMANDS for {args.timeout_observe_s:.2f}s: wheel should stop by timeout")
                    time.sleep(args.timeout_observe_s)
                else:
                    time.sleep(args.step_s)

        result["checks"] = {
            "all_errors_ok": all(step["mcu_error"] == ErrorCode.OK.name for step in result["steps"]),
            "sequence_echo_ok": all(step["sequence"] == step["state_sequence"] for step in result["steps"]),
            "motor_intents_match": all(
                step["motor_intent"] == step["expected_motor_intent"] for step in result["steps"]
            ),
            "stop_commands_accepted": all(
                step["motor_intent"] == MotorIntent.STOP.name
                for step in result["steps"]
                if step["command"] == CommandID.STOP.name
            ),
            "ended_with_stop": result["steps"][-1]["motor_intent"] == MotorIntent.STOP.name,
        }
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - hardware certification must report failure detail.
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
    print("MOTOR SAFETY CERTIFICATION: PASS" if result["pass"] else "MOTOR SAFETY CERTIFICATION: FAIL")
    return 0 if result["pass"] else 1


def describe_exchange(exchange, expected: MotorIntent, observation: str) -> dict:
    return {
        "command": exchange.command.name,
        "sequence": exchange.sequence,
        "state_sequence": exchange.state.sequence,
        "mcu_state": exchange.state.state.name,
        "mcu_error": exchange.state.error.name,
        "motor_intent": exchange.state.motor_intent.name,
        "expected_motor_intent": expected.name,
        "latency_ms": exchange.latency_ms,
        "observe": observation,
    }


if __name__ == "__main__":
    raise SystemExit(main())
