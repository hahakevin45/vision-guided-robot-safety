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
        description="Monitor encoder counts without commanding motor movement."
    )
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--interval-s", type=float, default=0.2)
    parser.add_argument("--left-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--right-encoder-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument(
        "--report", type=Path, default=Path("outputs/encoder_monitor.json")
    )
    args = parser.parse_args()

    result: dict[str, object] = {
        "pass": False,
        "device": args.device,
        "duration_s": args.duration_s,
        "interval_s": args.interval_s,
        "left_encoder_sign": args.left_encoder_sign,
        "right_encoder_sign": args.right_encoder_sign,
        "samples": [],
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
            stop = bridge.send_command(CommandID.STOP)
            result["steps"].extend(
                [_state_result("pre_heartbeat", heartbeat), _state_result("pre_stop", stop)]
            )

            first = bridge.read_encoders()
            result["steps"].append(_encoder_result("initial", first))
            prev_left = first.packet.left_count
            prev_right = first.packet.right_count
            last_ts = time.monotonic()
            deadline = last_ts + args.duration_s
            sample_index = 0

            print("idx elapsed left raw_d norm_d norm_cps | right raw_d norm_d norm_cps")
            while time.monotonic() < deadline:
                time.sleep(args.interval_s)
                now = time.monotonic()
                snapshot = bridge.read_encoders()
                elapsed_s = now - last_ts
                sample = build_encoder_sample(
                    sample_index=sample_index,
                    elapsed_s=elapsed_s,
                    left_count=snapshot.packet.left_count,
                    right_count=snapshot.packet.right_count,
                    prev_left_count=prev_left,
                    prev_right_count=prev_right,
                    left_encoder_sign=args.left_encoder_sign,
                    right_encoder_sign=args.right_encoder_sign,
                )
                result["samples"].append(sample)
                print(_format_sample(sample), flush=True)
                prev_left = snapshot.packet.left_count
                prev_right = snapshot.packet.right_count
                last_ts = now
                sample_index += 1

            cleanup = bridge.send_command(CommandID.STOP)
            result["steps"].append(_state_result("cleanup_stop", cleanup))

        result["checks"] = _build_checks(result)
        result["pass"] = all(result["checks"].values())
    except KeyboardInterrupt:
        result["error"] = "interrupted"
    except Exception as exc:  # noqa: BLE001 - monitor should preserve device failure detail.
        result["error"] = str(exc)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("ENCODER MONITOR: PASS" if result["pass"] else "ENCODER MONITOR: FAIL")
    return 0 if result["pass"] else 1


def build_encoder_sample(
    *,
    sample_index: int,
    elapsed_s: float,
    left_count: int,
    right_count: int,
    prev_left_count: int,
    prev_right_count: int,
    left_encoder_sign: int,
    right_encoder_sign: int,
) -> dict[str, object]:
    left_raw_delta = left_count - prev_left_count
    right_raw_delta = right_count - prev_right_count
    left_normalized_delta = left_raw_delta * left_encoder_sign
    right_normalized_delta = right_raw_delta * right_encoder_sign
    return {
        "sample_index": sample_index,
        "elapsed_s": elapsed_s,
        "left_count": left_count,
        "right_count": right_count,
        "left_raw_delta": left_raw_delta,
        "right_raw_delta": right_raw_delta,
        "left_normalized_delta": left_normalized_delta,
        "right_normalized_delta": right_normalized_delta,
        "left_normalized_counts_per_s": _rate(left_normalized_delta, elapsed_s),
        "right_normalized_counts_per_s": _rate(right_normalized_delta, elapsed_s),
    }


def _format_sample(sample: dict[str, object]) -> str:
    return (
        f"{sample['sample_index']:>3} "
        f"{sample['elapsed_s']:.3f} "
        f"{sample['left_count']:>8} "
        f"{sample['left_raw_delta']:>6} "
        f"{sample['left_normalized_delta']:>6} "
        f"{sample['left_normalized_counts_per_s']:>8.1f} | "
        f"{sample['right_count']:>8} "
        f"{sample['right_raw_delta']:>6} "
        f"{sample['right_normalized_delta']:>6} "
        f"{sample['right_normalized_counts_per_s']:>8.1f}"
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
    state_steps = [step for step in result["steps"] if step["kind"] == "state"]
    encoder_steps = [step for step in result["steps"] if step["kind"] == "encoder"]
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
        "samples_collected": len(result["samples"]) > 0,
    }


def _rate(delta: int, elapsed_s: float) -> float:
    if elapsed_s <= 0:
        return 0.0
    return delta / elapsed_s


if __name__ == "__main__":
    raise SystemExit(main())
