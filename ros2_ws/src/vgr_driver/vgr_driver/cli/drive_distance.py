from __future__ import annotations

import argparse
import json
import math
import os
import pty
import time
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode, MotorIntent

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver.serial_transport import PosixSerial


def compute_distance_targets(
    meters: float,
    wheel_diameter_cm: float,
    left_cpr: float,
    right_cpr: float,
) -> dict:
    circumference_cm = math.pi * wheel_diameter_cm
    revolutions = meters * 100.0 / circumference_cm
    left_target_counts = round(revolutions * left_cpr)
    right_target_counts = round(revolutions * right_cpr)
    return {
        "circumference_cm": circumference_cm,
        "revolutions": revolutions,
        "left_target_counts": left_target_counts,
        "right_target_counts": right_target_counts,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Drive a fixed distance using encoder feedback."
    )
    parser.add_argument("--device", default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--meters", type=float, required=True)
    parser.add_argument("--wheel-diameter-cm", type=float, default=6.5)
    parser.add_argument("--left-counts-per-rev", type=float, default=750.0)
    parser.add_argument("--right-counts-per-rev", type=float, default=749.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--closed-loop",
        action="store_true",
        help="用 SET_WHEEL_SPEED 低速閉環巡航取代開環 FORWARD 脈衝；每輪各自 PID "
        "追速度(走得直)、到自己的距離目標就把該輪速度設 0(小超程)，兩輪都到才收尾 STOP。",
    )
    parser.add_argument(
        "--cruise-counts-per-s",
        type=int,
        default=200,
        help="closed-loop 巡航速度 (counts/s)。越低超程越小、越慢；預設 200≈5.4cm/s。",
    )
    parser.add_argument("--max-seconds", type=float, default=20.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.05)
    parser.add_argument("--post-stop-s", type=float, default=0.05)
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/drive_distance.json"),
    )
    args = parser.parse_args()

    if args.meters <= 0:
        parser.error(f"--meters must be positive, got {args.meters}")
    if args.wheel_diameter_cm <= 0:
        parser.error(f"--wheel-diameter-cm must be positive, got {args.wheel_diameter_cm}")
    if args.left_counts_per_rev <= 0:
        parser.error(f"--left-counts-per-rev must be positive, got {args.left_counts_per_rev}")
    if args.right_counts_per_rev <= 0:
        parser.error(f"--right-counts-per-rev must be positive, got {args.right_counts_per_rev}")

    targets = compute_distance_targets(
        args.meters,
        args.wheel_diameter_cm,
        args.left_counts_per_rev,
        args.right_counts_per_rev,
    )

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

    result: dict = {
        "pass": False,
        "device": device,
        "using_pty_mock_mcu": using_pty,
        "dry_run": args.dry_run,
        "closed_loop": args.closed_loop,
        "cruise_counts_per_s": args.cruise_counts_per_s if args.closed_loop else None,
        "meters": args.meters,
        "wheel_diameter_cm": args.wheel_diameter_cm,
        "circumference_cm": targets["circumference_cm"],
        "revolutions": targets["revolutions"],
        "left_target_counts": targets["left_target_counts"],
        "right_target_counts": targets["right_target_counts"],
        "steps": [],
        "checks": {},
        "motor_commands_sent": 0,
        "error": None,
    }

    try:
        with PosixSerial(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            if not using_pty and args.settle_s > 0:
                time.sleep(args.settle_s)
                serial.flush_input()
            bridge = ControllerBridge(serial)

            if args.dry_run:
                heartbeat = bridge.send_command(CommandID.HEARTBEAT)
                result["steps"].append(_state_step("heartbeat", heartbeat))
                stop = bridge.send_command(CommandID.STOP)
                result["steps"].append(_state_step("dry_run_stop", stop))

                state_steps = [s for s in result["steps"] if s.get("kind") == "state"]
                result["checks"] = {
                    "no_motion_command_sent": result["motor_commands_sent"] == 0,
                    "targets_computed": targets["revolutions"] > 0,
                    "ended_with_stop": _ended_with_stop(result["steps"]),
                    "no_mcu_error": all(
                        s["mcu_error"] == ErrorCode.OK.name for s in state_steps
                    ),
                    "state_sequence_echo_ok": all(
                        s["sequence"] == s["state_sequence"] for s in state_steps
                    ),
                    "per_state_motor_intent_valid": _check_per_state_motor_intents(state_steps),
                }
            else:
                try:
                    heartbeat = bridge.send_command(CommandID.HEARTBEAT)
                    result["steps"].append(_state_step("heartbeat", heartbeat))

                    stop_init = bridge.send_command(CommandID.STOP)
                    result["steps"].append(_state_step("initial_stop", stop_init))

                    initial = bridge.read_encoders()
                    result["steps"].append(_encoder_step("initial_encoders", initial))
                    init_left = initial.packet.left_count
                    init_right = initial.packet.right_count

                    left_reached = False
                    right_reached = False
                    start_ts = time.monotonic()

                    while time.monotonic() - start_ts < args.max_seconds:
                        enc = bridge.read_encoders()
                        result["steps"].append(_encoder_step("poll_encoders", enc))
                        delta_left = abs(enc.packet.left_count - init_left)
                        delta_right = abs(enc.packet.right_count - init_right)
                        left_reached = delta_left >= targets["left_target_counts"]
                        right_reached = delta_right >= targets["right_target_counts"]
                        if left_reached and right_reached:
                            break
                        if args.closed_loop:
                            # 每輪各自 PID 追巡航速度(走直)；到自己距離目標就把該輪
                            # 目標設 0，另一輪續走，避免快輪等慢輪時空轉超程。不在
                            # 迭代間 STOP——持續巡航(deadman 500ms > poll 50ms 不會誤煞)。
                            left_cmd = 0 if left_reached else args.cruise_counts_per_s
                            right_cmd = 0 if right_reached else args.cruise_counts_per_s
                            spd = bridge.send_set_wheel_speed(left_cmd, right_cmd)
                            result["steps"].append(_state_step("set_wheel_speed", spd))
                            result["motor_commands_sent"] += 1
                            time.sleep(args.poll_interval_s)
                        else:
                            fwd = bridge.send_command(CommandID.FORWARD)
                            result["steps"].append(_state_step("forward", fwd))
                            result["motor_commands_sent"] += 1
                            time.sleep(args.poll_interval_s)
                            stp = bridge.send_command(CommandID.STOP)
                            result["steps"].append(_state_step("stop", stp))
                            time.sleep(args.post_stop_s)

                    result["checks"] = {
                        "left_target_reached": left_reached,
                        "right_target_reached": right_reached,
                        "ended_with_stop": False,
                        "no_mcu_error": True,
                        "state_sequence_echo_ok": True,
                        "encoder_sequence_echo_ok": True,
                    }
                finally:
                    cleanup_ok = False
                    try:
                        cleanup = bridge.send_command(CommandID.STOP)
                        result["steps"].append(_state_step("cleanup_stop", cleanup))
                        cleanup_ok = True
                    except Exception as cleanup_exc:
                        result["error"] = result["error"] or f"cleanup STOP failed: {cleanup_exc}"
                    if result["checks"]:
                        state_steps = [s for s in result["steps"] if s.get("kind") == "state"]
                        encoder_steps = [s for s in result["steps"] if s.get("kind") == "encoder"]
                        result["checks"]["ended_with_stop"] = cleanup_ok and _ended_with_stop(result["steps"])
                        result["checks"]["no_mcu_error"] = all(
                            s["mcu_error"] == ErrorCode.OK.name for s in state_steps
                        )
                        result["checks"]["state_sequence_echo_ok"] = all(
                            s["sequence"] == s["state_sequence"] for s in state_steps
                        )
                        result["checks"]["encoder_sequence_echo_ok"] = all(
                            s["sequence"] == s["packet_sequence"] for s in encoder_steps
                        )
                        result["checks"]["per_state_motor_intent_valid"] = _check_per_state_motor_intents(state_steps)

    except Exception as exc:  # noqa: BLE001
        result["error"] = str(exc)
    finally:
        if mock_mcu is not None:
            mock_mcu.stop()
        if master_fd is not None:
            os.close(master_fd)
        if slave_fd is not None:
            os.close(slave_fd)

    if result["checks"]:
        result["pass"] = all(result["checks"].values())

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("DRIVE DISTANCE: PASS" if result["pass"] else "DRIVE DISTANCE: FAIL")
    return 0 if result["pass"] else 1


def _state_step(label: str, exchange) -> dict:
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


def _encoder_step(label: str, exchange) -> dict:
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


def _check_per_state_motor_intents(state_steps: list[dict]) -> bool:
    """Each state step's motor_intent must match the command sent.

    HEARTBEAT and STOP commands must receive STOP intent; FORWARD must receive FORWARD.
    """
    stop_name = MotorIntent.STOP.name
    forward_name = MotorIntent.FORWARD.name
    for s in state_steps:
        cmd = s.get("command")
        intent = s.get("motor_intent")
        if cmd in (CommandID.HEARTBEAT.name, CommandID.STOP.name):
            if intent != stop_name:
                return False
        elif cmd == CommandID.FORWARD.name:
            if intent != forward_name:
                return False
    return True


def _ended_with_stop(steps: list[dict]) -> bool:
    state_steps = [s for s in steps if s.get("kind") == "state"]
    if not state_steps:
        return False
    last = state_steps[-1]
    return last.get("command") == CommandID.STOP.name and last.get("motor_intent") == MotorIntent.STOP.name


if __name__ == "__main__":
    raise SystemExit(main())
