from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode, MotorIntent

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.serial_transport import PosixSerial


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure encoder counts for one manually rotated wheel revolution."
    )
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--wheel", choices=("left", "right", "both"), required=True)
    parser.add_argument("--left-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--revolutions", type=int, default=1)
    parser.add_argument(
        "--wait-s",
        type=float,
        default=None,
        help="Wait this many seconds instead of prompting for Enter.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/encoder_revolution_calibration.json"),
    )
    args = parser.parse_args()

    result: dict[str, object] = {
        "pass": False,
        "device": args.device,
        "wheel": args.wheel,
        "left_encoder_sign": args.left_encoder_sign,
        "right_encoder_sign": args.right_encoder_sign,
        "revolutions": args.revolutions,
        "steps": [],
        "checks": {},
        "error": None,
    }

    try:
        with PosixSerial(
            device=args.device, baudrate=args.baudrate, timeout_s=args.timeout_s
        ) as serial:
            if args.settle_s > 0:
                time.sleep(args.settle_s)
            serial.flush_input()
            bridge = ControllerBridge(serial)

            heartbeat = bridge.send_command(CommandID.HEARTBEAT)
            stop_before = bridge.send_command(CommandID.STOP)
            before = bridge.read_encoders()
            result["steps"].extend(
                [
                    _state_result("pre_heartbeat", heartbeat),
                    _state_result("pre_stop", stop_before),
                    _encoder_result("before_manual_rotation", before),
                ]
            )

            _wait_for_manual_rotation(args.wheel, args.revolutions, args.wait_s)

            after = bridge.read_encoders()
            stop_after = bridge.send_command(CommandID.STOP)
            result["steps"].extend(
                [
                    _encoder_result("after_manual_rotation", after),
                    _state_result("cleanup_stop", stop_after),
                ]
            )

        calibration = build_revolution_calibration(
            wheel=args.wheel,
            left_before=before.packet.left_count,
            right_before=before.packet.right_count,
            left_after=after.packet.left_count,
            right_after=after.packet.right_count,
            left_encoder_sign=args.left_encoder_sign,
            right_encoder_sign=args.right_encoder_sign,
            revolutions=args.revolutions,
        )
        result.update(calibration)
        result["checks"] = _build_checks(result)
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - calibration CLI must report device failures.
        result["error"] = str(exc)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(
        "ENCODER REVOLUTION CALIBRATION: PASS"
        if result["pass"]
        else "ENCODER REVOLUTION CALIBRATION: FAIL"
    )
    return 0 if result["pass"] else 1


def build_revolution_calibration(
    *,
    wheel: str,
    left_before: int,
    right_before: int,
    left_after: int,
    right_after: int,
    left_encoder_sign: int,
    right_encoder_sign: int,
    revolutions: int = 1,
) -> dict[str, object]:
    if revolutions <= 0:
        raise ValueError("revolutions must be positive")
    left_raw_delta = left_after - left_before
    right_raw_delta = right_after - right_before
    left_normalized = left_raw_delta * left_encoder_sign
    right_normalized = right_raw_delta * right_encoder_sign
    left_active = wheel in ("left", "both")
    right_active = wheel in ("right", "both")

    return {
        "wheel": wheel,
        "revolutions": revolutions,
        "left": {
            "before": left_before,
            "after": left_after,
            "raw_delta": left_raw_delta,
            "normalized_delta": left_normalized,
            "total_counts": abs(left_normalized) if left_active else None,
            "counts_per_rev": (
                abs(left_normalized) / revolutions if left_active else None
            ),
            "selected_for_calibration": left_active,
        },
        "right": {
            "before": right_before,
            "after": right_after,
            "raw_delta": right_raw_delta,
            "normalized_delta": right_normalized,
            "total_counts": abs(right_normalized) if right_active else None,
            "counts_per_rev": (
                abs(right_normalized) / revolutions if right_active else None
            ),
            "selected_for_calibration": right_active,
        },
        "odom_recommendation": {
            "left_encoder_sign": left_encoder_sign,
            "right_encoder_sign": right_encoder_sign,
            "left_counts_per_rev": (
                abs(left_normalized) / revolutions if left_active else None
            ),
            "right_counts_per_rev": (
                abs(right_normalized) / revolutions if right_active else None
            ),
        },
    }


def _wait_for_manual_rotation(
    wheel: str, revolutions: int, wait_s: float | None
) -> None:
    if wait_s is not None:
        print(
            f"Rotate {wheel} wheel exactly {revolutions} forward revolutions now; "
            f"waiting {wait_s:.1f}s."
        )
        time.sleep(wait_s)
        return
    input(
        f"Rotate the {wheel} wheel exactly {revolutions} forward revolutions, then press Enter..."
    )


def _state_result(label: str, exchange) -> dict[str, object]:
    return {
        "kind": "state",
        "label": label,
        "command": exchange.command.name,
        "sequence": exchange.sequence,
        "state_sequence": exchange.state.sequence,
        "mcu_state": exchange.state.state.name,
        "mcu_error": exchange.state.error.name,
        "motor_intent": exchange.state.motor_intent.name,
        "latency_ms": exchange.latency_ms,
    }


def _encoder_result(label: str, exchange) -> dict[str, object]:
    return {
        "kind": "encoder",
        "label": label,
        "sequence": exchange.sequence,
        "packet_sequence": exchange.packet.sequence,
        "left_count": exchange.packet.left_count,
        "right_count": exchange.packet.right_count,
        "flags": exchange.packet.flags,
        "latency_ms": exchange.latency_ms,
    }


def _build_checks(result: dict[str, object]) -> dict[str, bool]:
    steps = result["steps"]
    state_steps = [step for step in steps if step["kind"] == "state"]
    encoder_steps = [step for step in steps if step["kind"] == "encoder"]
    left_selected = result["left"]["selected_for_calibration"]
    right_selected = result["right"]["selected_for_calibration"]
    return {
        "state_errors_ok": all(
            step["mcu_error"] == ErrorCode.OK.name for step in state_steps
        ),
        "state_sequence_echo_ok": all(
            step["sequence"] == step["state_sequence"] for step in state_steps
        ),
        "encoder_sequence_echo_ok": all(
            step["sequence"] == step["packet_sequence"] for step in encoder_steps
        ),
        "ended_with_stop": bool(state_steps)
        and state_steps[-1]["motor_intent"] == MotorIntent.STOP.name,
        "selected_left_changed": (not left_selected)
        or result["left"]["counts_per_rev"] > 0,
        "selected_right_changed": (not right_selected)
        or result["right"]["counts_per_rev"] > 0,
    }


if __name__ == "__main__":
    raise SystemExit(main())
