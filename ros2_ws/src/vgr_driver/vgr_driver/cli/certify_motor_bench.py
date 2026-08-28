from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode, MotorIntent

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.serial_transport import PosixSerial


STEPS = [
    (CommandID.HEARTBEAT, MotorIntent.STOP, "controller resync, motors stopped"),
    (CommandID.FORWARD, MotorIntent.FORWARD, "both wheels should rotate forward"),
    (CommandID.STOP, MotorIntent.STOP, "both wheels should stop"),
    (CommandID.TURN_LEFT, MotorIntent.TURN_LEFT, "right wheel should rotate, left wheel should stop"),
    (CommandID.STOP, MotorIntent.STOP, "both wheels should stop"),
    (CommandID.TURN_RIGHT, MotorIntent.TURN_RIGHT, "left wheel should rotate, right wheel should stop"),
    (CommandID.STOP, MotorIntent.STOP, "both wheels should stop"),
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a low-duty bench motor sequence on the real STM32.")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--step-s", type=float, default=1.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/motor_bench_certification.json"))
    args = parser.parse_args()

    result = {
        "pass": False,
        "device": args.device,
        "step_s": args.step_s,
        "checks": {},
        "steps": [],
        "error": None,
        "manual_confirmation_required": True,
    }

    try:
        with PosixSerial(device=args.device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            time.sleep(args.settle_s)
            serial.flush_input()
            bridge = ControllerBridge(serial)
            for command, expected_intent, observation in STEPS:
                print(f"{command.name}: {observation}")
                exchange = bridge.send_command(command)
                result["steps"].append(
                    {
                        "command": exchange.command.name,
                        "expected_motor_intent": expected_intent.name,
                        "motor_intent": exchange.state.motor_intent.name,
                        "mcu_state": exchange.state.state.name,
                        "mcu_error": exchange.state.error.name,
                        "sequence": exchange.sequence,
                        "state_sequence": exchange.state.sequence,
                        "latency_ms": exchange.latency_ms,
                        "observe": observation,
                    }
                )
                time.sleep(args.step_s)

        result["checks"] = {
            "all_errors_ok": all(step["mcu_error"] == ErrorCode.OK.name for step in result["steps"]),
            "sequence_echo_ok": all(step["sequence"] == step["state_sequence"] for step in result["steps"]),
            "motor_intents_match": all(
                step["motor_intent"] == step["expected_motor_intent"] for step in result["steps"]
            ),
            "ended_with_stop": result["steps"][-1]["motor_intent"] == MotorIntent.STOP.name,
        }
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - bench testing must preserve device failure detail.
        result["error"] = str(exc)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("MOTOR BENCH CERTIFICATION: PASS" if result["pass"] else "MOTOR BENCH CERTIFICATION: FAIL")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
