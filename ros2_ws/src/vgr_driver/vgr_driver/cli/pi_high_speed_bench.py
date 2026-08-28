"""Fail-closed raised-wheel dual-speed evidence and runtime."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time
from typing import Sequence

from vgr_core.model import CommandID, ErrorCode, MotorIntent
from vgr_driver.driver import ControllerBridge
from vgr_driver.driver import PosixSerial


WHEEL_DIAMETER_M = 0.065


def counts_to_distance_m(counts: int, counts_per_rev: float) -> float:
    return counts / counts_per_rev * math.pi * WHEEL_DIAMETER_M


def should_continue_stop_collection(
    stop_start_s: float,
    last_moving_s: float,
    now_s: float,
) -> bool:
    return now_s - stop_start_s < 4.0 and now_s - last_moving_s < 2.0


@dataclass(frozen=True)
class SpeedSegmentEvidence:
    target_cps: int
    command_duration_s: float
    left_delta_counts: int
    right_delta_counts: int
    left_mean_cps: float
    right_mean_cps: float
    mean_distance_m: float
    wheel_distance_mismatch_ratio: float
    stop_acknowledged: bool
    final_abs_left_cps: float
    final_abs_right_cps: float
    stopped_observation_s: float
    max_mismatch_run_s: float
    faults: tuple[str, ...]


def evaluate_speed_segment(
    evidence: SpeedSegmentEvidence,
    *,
    preflight: bool = False,
) -> dict[str, object]:
    reasons: list[str] = []
    minimum_duration = 1.0 if preflight else 5.0
    if evidence.command_duration_s < minimum_duration:
        reasons.append("command duration was too short")
    if evidence.left_delta_counts <= 0 or evidence.right_delta_counts <= 0:
        reasons.append("encoder direction was not forward on both wheels")
    if not preflight and abs(
        evidence.left_mean_cps - evidence.target_cps
    ) / evidence.target_cps > 0.15:
        reasons.append("left speed error exceeded 15 percent")
    if not preflight and abs(
        evidence.right_mean_cps - evidence.target_cps
    ) / evidence.target_cps > 0.15:
        reasons.append("right speed error exceeded 15 percent")
    if not preflight and not 0.85 <= evidence.mean_distance_m <= 1.15:
        reasons.append("encoder-derived distance was outside [0.85, 1.15] m")
    if evidence.wheel_distance_mismatch_ratio > 0.10:
        reasons.append("wheel distance mismatch exceeded 10 percent")
    if evidence.max_mismatch_run_s >= 0.50:
        reasons.append("sustained instantaneous wheel mismatch reached 0.50 s")
    if not evidence.stop_acknowledged:
        reasons.append("final STOP was not acknowledged")
    if max(evidence.final_abs_left_cps, evidence.final_abs_right_cps) > 10.0:
        reasons.append("final wheel rate exceeded 10 counts/s")
    if evidence.stopped_observation_s < 2.0:
        reasons.append("stopped observation was shorter than 2 seconds")
    if evidence.faults:
        reasons.append("hardware fault was reported: " + "; ".join(evidence.faults))
    return {
        "pass": not reasons,
        "reasons": reasons,
        "metrics": asdict(evidence),
        "thresholds": {
            "max_speed_error_ratio": 0.15,
            "min_distance_m": None if preflight else 0.85,
            "max_distance_m": None if preflight else 1.15,
            "max_wheel_distance_mismatch_ratio": 0.10,
            "max_mismatch_run_s": 0.50,
            "max_final_abs_cps": 10.0,
            "min_stopped_observation_s": 2.0,
        },
    }


def atomic_json(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


class BenchInterrupted(RuntimeError):
    pass


def require_raised_confirmation(value: str) -> None:
    if value != "YES":
        raise ValueError("speed1m requires VGR_WHEELS_RAISED=YES")


def _send_target(bridge: ControllerBridge, target_cps: int):
    if target_cps == 600:
        return bridge.send_set_wheel_speed(600, 600)
    if target_cps == 735:
        return bridge.send_set_wheel_speed(735, 735)
    raise ValueError(f"unsupported fixed target: {target_cps}")


def _collect_segment(
    bridge: ControllerBridge,
    *,
    target_cps: int,
    command_duration_s: float,
    preflight: bool,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    baseline = bridge.read_encoders()
    initial_left = baseline.packet.left_count
    initial_right = baseline.packet.right_count
    previous_left = initial_left
    previous_right = initial_right
    previous_s = time.monotonic()
    start_s = previous_s
    last_command_s = 0.0
    mismatch_run_s = 0.0
    max_mismatch_run_s = 0.0
    samples: list[dict[str, object]] = []
    faults: list[str] = []

    while time.monotonic() - start_s < command_duration_s:
        now_s = time.monotonic()
        if now_s - last_command_s >= 0.10:
            exchange = _send_target(bridge, target_cps)
            if exchange.state.error != ErrorCode.OK:
                faults.append(exchange.state.error.name)
                break
            last_command_s = now_s
        encoder_exchange = bridge.read_encoders()
        sample_s = time.monotonic()
        dt_s = sample_s - previous_s
        left = encoder_exchange.packet.left_count
        right = encoder_exchange.packet.right_count
        left_cps = (left - previous_left) / dt_s if dt_s > 0.0 else 0.0
        right_cps = (right - previous_right) / dt_s if dt_s > 0.0 else 0.0
        elapsed_s = sample_s - start_s
        mean_abs_cps = (abs(left_cps) + abs(right_cps)) / 2.0
        mismatch_ratio = abs(left_cps - right_cps) / max(mean_abs_cps, 1.0)
        if elapsed_s >= 0.30 and mismatch_ratio > 0.25:
            mismatch_run_s += dt_s
        else:
            mismatch_run_s = 0.0
        max_mismatch_run_s = max(max_mismatch_run_s, mismatch_run_s)
        samples.append({
            "phase": "command",
            "t_s": elapsed_s,
            "left_count": left,
            "right_count": right,
            "left_cps": left_cps,
            "right_cps": right_cps,
            "mismatch_ratio": mismatch_ratio,
        })
        previous_left = left
        previous_right = right
        previous_s = sample_s
        if mismatch_run_s >= 0.50:
            faults.append("sustained wheel mismatch")
            break
        time.sleep(0.05)

    command_elapsed_s = time.monotonic() - start_s
    stop_exchange = bridge.send_command(CommandID.STOP)
    stop_acknowledged = stop_exchange.state.motor_intent == MotorIntent.STOP
    stop_start_s = time.monotonic()
    last_moving_s = stop_start_s
    stop_rates: list[tuple[float, float, float]] = []
    while True:
        encoder_exchange = bridge.read_encoders()
        sample_s = time.monotonic()
        dt_s = sample_s - previous_s
        left = encoder_exchange.packet.left_count
        right = encoder_exchange.packet.right_count
        left_cps = (left - previous_left) / dt_s if dt_s > 0.0 else 0.0
        right_cps = (right - previous_right) / dt_s if dt_s > 0.0 else 0.0
        if max(abs(left_cps), abs(right_cps)) > 10.0:
            last_moving_s = sample_s
        stop_rates.append((sample_s, left_cps, right_cps))
        samples.append({
            "phase": "stop",
            "t_s": sample_s - start_s,
            "left_count": left,
            "right_count": right,
            "left_cps": left_cps,
            "right_cps": right_cps,
        })
        previous_left = left
        previous_right = right
        previous_s = sample_s
        if not should_continue_stop_collection(
            stop_start_s,
            last_moving_s,
            sample_s,
        ):
            break
        time.sleep(0.05)

    final = bridge.read_encoders()
    final_left = final.packet.left_count
    final_right = final.packet.right_count
    left_delta = final_left - initial_left
    right_delta = final_right - initial_right
    steady = [
        sample
        for sample in samples
        if sample["phase"] == "command"
        and float(sample["t_s"]) >= max(0.0, command_duration_s - 1.0)
    ]
    left_mean = (
        sum(float(sample["left_cps"]) for sample in steady) / len(steady)
        if steady else 0.0
    )
    right_mean = (
        sum(float(sample["right_cps"]) for sample in steady) / len(steady)
        if steady else 0.0
    )
    left_distance = counts_to_distance_m(left_delta, 750.0)
    right_distance = counts_to_distance_m(right_delta, 749.0)
    mean_distance = (left_distance + right_distance) / 2.0
    distance_mismatch = abs(left_distance - right_distance) / max(
        abs(mean_distance),
        1e-9,
    )
    final_left_cps = abs(stop_rates[-1][1]) if stop_rates else math.inf
    final_right_cps = abs(stop_rates[-1][2]) if stop_rates else math.inf
    stopped_observation_s = max(
        0.0,
        (stop_rates[-1][0] if stop_rates else stop_start_s) - last_moving_s,
    )
    evidence = SpeedSegmentEvidence(
        target_cps=target_cps,
        command_duration_s=command_elapsed_s,
        left_delta_counts=left_delta,
        right_delta_counts=right_delta,
        left_mean_cps=left_mean,
        right_mean_cps=right_mean,
        mean_distance_m=mean_distance,
        wheel_distance_mismatch_ratio=distance_mismatch,
        stop_acknowledged=stop_acknowledged,
        final_abs_left_cps=final_left_cps,
        final_abs_right_cps=final_right_cps,
        stopped_observation_s=stopped_observation_s,
        max_mismatch_run_s=max_mismatch_run_s,
        faults=tuple(faults),
    )
    return evaluate_speed_segment(evidence, preflight=preflight), samples


def run_serial_bench(device: str) -> dict[str, object]:
    bridge: ControllerBridge | None = None
    old_term = signal.getsignal(signal.SIGTERM)
    old_int = signal.getsignal(signal.SIGINT)
    old_alarm = signal.getsignal(signal.SIGALRM)

    def interrupt(signum, _frame) -> None:
        raise BenchInterrupted(f"received signal {signum}")

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGALRM, interrupt)
    signal.alarm(20)
    try:
        with PosixSerial(device, baudrate=115200, timeout_s=0.5) as serial:
            time.sleep(0.5)
            serial.flush_input()
            bridge = ControllerBridge(serial)
            bridge.send_command(CommandID.HEARTBEAT)
            bridge.send_command(CommandID.STOP)
            preflight, preflight_samples = _collect_segment(
                bridge,
                target_cps=600,
                command_duration_s=1.0,
                preflight=True,
            )
            full: dict[str, object] | None = None
            full_samples: list[dict[str, object]] = []
            if preflight["pass"]:
                full, full_samples = _collect_segment(
                    bridge,
                    target_cps=735,
                    command_duration_s=5.0,
                    preflight=False,
                )
            return {
                "mode": "high_speed",
                "pass": bool(preflight["pass"] and full and full["pass"]),
                "preflight": preflight,
                "full_speed": full,
                "samples": {
                    "preflight": preflight_samples,
                    "full_speed": full_samples,
                },
                "generated_at": datetime.now(timezone.utc).isoformat(),
            }
    finally:
        if bridge is not None:
            for _ in range(3):
                try:
                    bridge.send_command(CommandID.STOP)
                except Exception:
                    pass
        signal.alarm(0)
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGALRM, old_alarm)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--wheels-raised", required=True)
    parser.add_argument("--report", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        require_raised_confirmation(args.wheels_raised)
        report = run_serial_bench(args.device)
    except Exception as exc:
        report = {
            "mode": "high_speed",
            "pass": False,
            "reasons": [f"{type(exc).__name__}: {exc}"],
            "generated_at": datetime.now(timezone.utc).isoformat(),
        }
    atomic_json(args.report, report)
    passed = bool(report.get("pass"))
    print("PI_HIGH_SPEED_PASS" if passed else "PI_HIGH_SPEED_FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
