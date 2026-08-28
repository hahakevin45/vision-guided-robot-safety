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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Step through low-duty wheel and encoder bring-up."
    )
    parser.add_argument("--device", default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--spin-s", type=float, default=0.5)
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/wheel_bringup_report.json"),
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
        "spin_s": args.spin_s,
        "manual_safety": [
            "Keep wheels lifted before running this on real hardware.",
            "Be ready to cut 12V motor VM power if a wheel keeps spinning.",
            "This tool sends STOP after each wheel step and again during cleanup.",
        ],
        "steps": [],
        "left": {},
        "right": {},
        "checks": {},
        "recommendations": [],
        "error": None,
    }

    try:
        with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if not using_pty and args.settle_s > 0:
                time.sleep(args.settle_s)
                serial.flush_input()
            bridge = ControllerBridge(serial)
            try:
                heartbeat = bridge.send_command(CommandID.HEARTBEAT)
                result["steps"].append(_state_step("heartbeat", heartbeat))

                initial = bridge.read_encoders()
                result["steps"].append(_encoder_step("initial_encoder", initial))

                print("LEFT WHEEL: TURN_RIGHT should rotate left wheel only")
                left_command = bridge.send_command(CommandID.TURN_RIGHT)
                result["steps"].append(_state_step("left_wheel_command", left_command))
                time.sleep(args.spin_s)
                left_after = bridge.read_encoders()
                result["steps"].append(_encoder_step("left_wheel_encoder", left_after))
                left_stop = bridge.send_command(CommandID.STOP)
                result["steps"].append(_state_step("left_wheel_stop", left_stop))

                print("RIGHT WHEEL: TURN_LEFT should rotate right wheel only")
                right_command = bridge.send_command(CommandID.TURN_LEFT)
                result["steps"].append(_state_step("right_wheel_command", right_command))
                time.sleep(args.spin_s)
                right_after = bridge.read_encoders()
                result["steps"].append(_encoder_step("right_wheel_encoder", right_after))
                right_stop = bridge.send_command(CommandID.STOP)
                result["steps"].append(_state_step("right_wheel_stop", right_stop))

                result["left"] = _wheel_delta(initial, left_after)
                result["right"] = _wheel_delta(left_after, right_after)
                result["checks"] = _checks(result, heartbeat, left_command, left_stop, right_command, right_stop)
                result["recommendations"] = _recommendations(result)
                result["pass"] = all(result["checks"].values())
            finally:
                try:
                    stop = bridge.send_command(CommandID.STOP)
                    result["steps"].append(_state_step("cleanup_stop", stop))
                except Exception:
                    pass
    except Exception as exc:  # noqa: BLE001 - bring-up must preserve hardware failure details.
        result["error"] = str(exc)
    finally:
        if mock_mcu is not None:
            mock_mcu.stop()
        if master_fd is not None:
            os.close(master_fd)
        if slave_fd is not None:
            os.close(slave_fd)

    if result["checks"]:
        result["checks"]["ended_with_stop"] = _ended_with_stop(result["steps"])
        result["pass"] = all(result["checks"].values())

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("WHEEL BRINGUP: PASS" if result["pass"] else "WHEEL BRINGUP: FAIL")
    return 0 if result["pass"] else 1


def _state_step(label: str, exchange) -> dict[str, object]:
    return {
        "label": label,
        "kind": "state",
        "command": exchange.command.name,
        "sequence": exchange.sequence,
        "state_sequence": exchange.state.sequence,
        "mcu_state": exchange.state.state.name,
        "mcu_error": exchange.state.error.name,
        "motor_intent": exchange.state.motor_intent.name,
        "latency_ms": exchange.latency_ms,
    }


def _encoder_step(label: str, exchange) -> dict[str, object]:
    return {
        "label": label,
        "kind": "encoder",
        "command": exchange.command.name,
        "sequence": exchange.sequence,
        "packet_sequence": exchange.packet.sequence,
        "left_count": exchange.packet.left_count,
        "right_count": exchange.packet.right_count,
        "flags": exchange.packet.flags,
        "latency_ms": exchange.latency_ms,
    }


def _wheel_delta(before, after) -> dict[str, int]:
    return {
        "before_left_count": before.packet.left_count,
        "before_right_count": before.packet.right_count,
        "after_left_count": after.packet.left_count,
        "after_right_count": after.packet.right_count,
        "delta_left": after.packet.left_count - before.packet.left_count,
        "delta_right": after.packet.right_count - before.packet.right_count,
    }


def _checks(result, heartbeat, left_command, left_stop, right_command, right_stop) -> dict[str, bool]:
    left = result["left"]
    right = result["right"]
    state_exchanges = [heartbeat, left_command, left_stop, right_command, right_stop]
    return {
        "state_commands_ok": all(exchange.state.error == ErrorCode.OK for exchange in state_exchanges),
        "state_sequence_echo_ok": all(
            exchange.sequence == exchange.state.sequence for exchange in state_exchanges
        ),
        "left_command_intent_ok": left_command.state.motor_intent == MotorIntent.TURN_RIGHT,
        "right_command_intent_ok": right_command.state.motor_intent == MotorIntent.TURN_LEFT,
        "left_stop_ok": left_stop.state.motor_intent == MotorIntent.STOP,
        "right_stop_ok": right_stop.state.motor_intent == MotorIntent.STOP,
        "left_encoder_changed": left["delta_left"] != 0,
        "left_cross_count_stable": left["delta_right"] == 0,
        "right_encoder_changed": right["delta_right"] != 0,
        "right_cross_count_stable": right["delta_left"] == 0,
        "ended_with_stop": False,
    }


def _recommendations(result) -> list[str]:
    recommendations: list[str] = []
    left = result["left"]
    right = result["right"]
    checks = result["checks"]

    if not checks["left_encoder_changed"]:
        recommendations.append("Left wheel command did not change left encoder count; check left motor power, encoder VCC/GND, and PA0/PA1 wiring.")
    if not checks["right_encoder_changed"]:
        recommendations.append("Right wheel command did not change right encoder count; check right motor power, encoder VCC/GND, and PA4/PB0 wiring.")
    if not checks["left_cross_count_stable"]:
        recommendations.append("Left wheel step changed right encoder count; check left/right encoder wiring may be swapped.")
    if not checks["right_cross_count_stable"]:
        recommendations.append("Right wheel step changed left encoder count; check left/right encoder wiring may be swapped.")
    if left.get("delta_left", 0) < 0:
        recommendations.append("Left encoder count decreased during left-wheel forward test; record this for a future left encoder inversion setting.")
    if right.get("delta_right", 0) < 0:
        recommendations.append("Right encoder count decreased during right-wheel forward test; record this for a future right encoder inversion setting.")
    if not recommendations:
        recommendations.append("Wheel bring-up telemetry is internally consistent; next hardware step is manual direction confirmation and then low-duty bench driving.")
    return recommendations


def _ended_with_stop(steps: list[dict[str, object]]) -> bool:
    state_steps = [step for step in steps if step.get("kind") == "state"]
    if not state_steps:
        return False
    last = state_steps[-1]
    return last.get("command") == CommandID.STOP.name and last.get("motor_intent") == MotorIntent.STOP.name


if __name__ == "__main__":
    raise SystemExit(main())
