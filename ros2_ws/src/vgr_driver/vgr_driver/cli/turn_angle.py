"""原地轉向固定角度：雙輪反向閉環巡航 + 編碼器角度收尾。

差動原地旋轉時每輪走的弧長 = (輪距/2) × 角度(rad)。正角度 = 右轉
(左輪前進、右輪後退)，負角度 = 左轉。每輪用 |Δcounts| 追自己的目標，
到了就把該輪速度設 0，兩輪都到才收尾 STOP —— 與 drive_distance
--closed-loop 同一套收尾邏輯。

注意：編碼器推算的是「輪子轉了多少」，原地旋轉時輪胎打滑與前方
萬向輪的轉向阻力會讓實際車身轉角小於編碼器推算值；差多少要靠
marker 往返法實測。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pty
import time
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.cli.drive_distance import _check_per_state_motor_intents, _encoder_step, _ended_with_stop, _state_step
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver.serial_transport import PosixSerial


def compute_turn_targets(
    degrees: float,
    wheel_base_m: float,
    wheel_diameter_cm: float,
    left_cpr: float,
    right_cpr: float,
) -> dict:
    """回傳原地轉 degrees 度時，每輪弧長與各輪目標 counts(絕對值)與方向。"""
    radians = math.radians(abs(degrees))
    arc_m = (wheel_base_m / 2.0) * radians
    circumference_cm = math.pi * wheel_diameter_cm
    revolutions = arc_m * 100.0 / circumference_cm
    turn_right = degrees > 0
    return {
        "arc_m": arc_m,
        "circumference_cm": circumference_cm,
        "revolutions": revolutions,
        "left_target_counts": round(revolutions * left_cpr),
        "right_target_counts": round(revolutions * right_cpr),
        # 右轉 = 左輪前進(+)、右輪後退(−)；左轉相反。
        "left_sign": 1 if turn_right else -1,
        "right_sign": -1 if turn_right else 1,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Turn in place by a fixed angle using encoder feedback (closed-loop)."
    )
    parser.add_argument("--device", default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument(
        "--degrees",
        type=float,
        required=True,
        help="轉角(度)。正 = 右轉(順時針俯視)，負 = 左轉。",
    )
    parser.add_argument("--wheel-base-m", type=float, default=0.165)
    parser.add_argument("--wheel-diameter-cm", type=float, default=6.5)
    parser.add_argument("--left-counts-per-rev", type=float, default=750.0)
    parser.add_argument("--right-counts-per-rev", type=float, default=749.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--cruise-counts-per-s",
        type=int,
        default=200,
        help="巡航速度大小 (counts/s)，兩輪各取正負。",
    )
    parser.add_argument("--max-seconds", type=float, default=20.0)
    parser.add_argument("--poll-interval-s", type=float, default=0.05)
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/turn_angle.json"),
    )
    args = parser.parse_args()

    if args.degrees == 0:
        parser.error("--degrees must be non-zero")
    if args.wheel_base_m <= 0:
        parser.error(f"--wheel-base-m must be positive, got {args.wheel_base_m}")
    if args.wheel_diameter_cm <= 0:
        parser.error(f"--wheel-diameter-cm must be positive, got {args.wheel_diameter_cm}")
    if args.left_counts_per_rev <= 0:
        parser.error(f"--left-counts-per-rev must be positive, got {args.left_counts_per_rev}")
    if args.right_counts_per_rev <= 0:
        parser.error(f"--right-counts-per-rev must be positive, got {args.right_counts_per_rev}")
    if args.cruise_counts_per_s <= 0:
        parser.error(f"--cruise-counts-per-s must be positive, got {args.cruise_counts_per_s}")

    targets = compute_turn_targets(
        args.degrees,
        args.wheel_base_m,
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
        "degrees": args.degrees,
        "cruise_counts_per_s": args.cruise_counts_per_s,
        "wheel_base_m": args.wheel_base_m,
        "wheel_diameter_cm": args.wheel_diameter_cm,
        "arc_m": targets["arc_m"],
        "revolutions": targets["revolutions"],
        "left_target_counts": targets["left_target_counts"],
        "right_target_counts": targets["right_target_counts"],
        "left_sign": targets["left_sign"],
        "right_sign": targets["right_sign"],
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
                        # 與 drive_distance closed-loop 同款收尾：各輪到自己的
                        # 目標就設 0，另一輪續走；持續巡航不在迭代間 STOP
                        # (deadman 500ms > poll 50ms)。
                        left_cmd = 0 if left_reached else targets["left_sign"] * args.cruise_counts_per_s
                        right_cmd = 0 if right_reached else targets["right_sign"] * args.cruise_counts_per_s
                        spd = bridge.send_set_wheel_speed(left_cmd, right_cmd)
                        result["steps"].append(_state_step("set_wheel_speed", spd))
                        result["motor_commands_sent"] += 1
                        time.sleep(args.poll_interval_s)

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
    print("TURN ANGLE: PASS" if result["pass"] else "TURN ANGLE: FAIL")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
