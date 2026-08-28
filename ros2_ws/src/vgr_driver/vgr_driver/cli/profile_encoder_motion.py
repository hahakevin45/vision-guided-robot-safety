from __future__ import annotations

import argparse
import json
import os
import pty
import time
from dataclasses import dataclass
from pathlib import Path

from vgr_core.model import CommandID, ErrorCode, MotorIntent

from vgr_driver.driver.controller_bridge import ControllerBridge, BridgeExchange, EncoderExchange
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver.serial_transport import PosixSerial


@dataclass(frozen=True)
class EncoderSegment:
    label: str
    command: str
    expected_physical_motion: str
    elapsed_s: float
    left_delta: int
    right_delta: int
    requested_duration_s: float | None = None

    @property
    def left_counts_per_s(self) -> float:
        return _counts_per_s(self.left_delta, self.elapsed_s)

    @property
    def right_counts_per_s(self) -> float:
        return _counts_per_s(self.right_delta, self.elapsed_s)


DIRECTION_SEQUENCE = [
    (
        "turn_left_right_wheel",
        CommandID.TURN_LEFT,
        MotorIntent.TURN_LEFT,
        "right wheel",
    ),
    (
        "turn_right_left_wheel",
        CommandID.TURN_RIGHT,
        MotorIntent.TURN_RIGHT,
        "left wheel",
    ),
    (
        "forward_both_wheels",
        CommandID.FORWARD,
        MotorIntent.FORWARD,
        "both wheels",
    ),
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Profile encoder direction and fixed-duty wheel speed."
    )
    parser.add_argument("--device", default=None)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--direction-pulse-s", type=float, default=0.15)
    parser.add_argument(
        "--speed-pulse-s",
        type=float,
        action="append",
        default=None,
        help="Fixed-duty FORWARD duration. Repeat for multiple speed samples.",
    )
    parser.add_argument("--gap-s", type=float, default=0.8)
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/encoder_motion_profile.json"),
    )
    args = parser.parse_args()

    speed_pulses = args.speed_pulse_s or [1.0, 3.0]
    device = args.device
    using_pty = device is None
    master_fd = None
    slave_fd = None
    mock_mcu = None

    if using_pty:
        master_fd, slave_fd = pty.openpty()
        device = os.ttyname(slave_fd)
        mock_mcu = MockSerialMCU(master_fd, timeout_s=args.timeout_s)
        mock_mcu.start()

    result: dict[str, object] = {
        "pass": False,
        "device": device,
        "using_pty_mock_mcu": using_pty,
        "direction_pulse_s": args.direction_pulse_s,
        "speed_pulse_s": speed_pulses,
        "gap_s": args.gap_s,
        "manual_observation_required": not using_pty,
        "direction_segments": [],
        "speed_segments": [],
        "steps": [],
        "checks": {},
        "error": None,
    }

    try:
        with PosixSerial(
            device=device, baudrate=args.baudrate, timeout_s=args.timeout_s
        ) as serial:
            if args.settle_s > 0:
                time.sleep(args.settle_s)
            serial.flush_input()
            bridge = ControllerBridge(serial)
            steps: list[dict[str, object]] = []

            steps.append(_state_result("pre_heartbeat", bridge.send_command(CommandID.HEARTBEAT)))
            steps.append(_state_result("pre_stop", bridge.send_command(CommandID.STOP)))
            time.sleep(args.gap_s)

            direction_segments = [
                _run_segment(
                    bridge=bridge,
                    steps=steps,
                    label=label,
                    command=command,
                    expected_intent=expected_intent,
                    expected_physical_motion=expected_physical_motion,
                    duration_s=args.direction_pulse_s,
                    gap_s=args.gap_s,
                )
                for label, command, expected_intent, expected_physical_motion in DIRECTION_SEQUENCE
            ]

            speed_segments = [
                _run_segment(
                    bridge=bridge,
                    steps=steps,
                    label=f"forward_speed_{duration_s:.1f}s",
                    command=CommandID.FORWARD,
                    expected_intent=MotorIntent.FORWARD,
                    expected_physical_motion="both wheels",
                    duration_s=duration_s,
                    gap_s=args.gap_s,
                )
                for duration_s in speed_pulses
            ]

            cleanup = bridge.send_command(CommandID.STOP)
            steps.append(_state_result("cleanup_stop", cleanup))

        profile = build_encoder_motion_profile(direction_segments, speed_segments)
        result.update(profile)
        result["steps"] = steps
        result["checks"] = _build_checks(result, steps)
        result["pass"] = all(result["checks"].values())
    except Exception as exc:  # noqa: BLE001 - CLI must preserve device failure detail.
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
    print("ENCODER MOTION PROFILE: PASS" if result["pass"] else "ENCODER MOTION PROFILE: FAIL")
    return 0 if result["pass"] else 1


def build_encoder_motion_profile(
    direction_segments: list[EncoderSegment],
    speed_segments: list[EncoderSegment],
) -> dict[str, object]:
    direction_items = [_segment_result(segment) for segment in direction_segments]
    forward_segment = _find_segment(direction_segments, "forward_both_wheels")
    left_sign = _forward_sign(forward_segment.left_delta if forward_segment else 0)
    right_sign = _forward_sign(forward_segment.right_delta if forward_segment else 0)

    return {
        "encoder_mapping": _infer_encoder_mapping(direction_segments),
        "odom_recommendation": {
            "left_encoder_sign": left_sign,
            "right_encoder_sign": right_sign,
            "note": "Multiply raw encoder counts by these signs before odom integration.",
        },
        "direction_segments": direction_items,
        "speed_segments": [
            _speed_segment_result(segment, left_sign, right_sign)
            for segment in speed_segments
        ],
    }


def _run_segment(
    *,
    bridge: ControllerBridge,
    steps: list[dict[str, object]],
    label: str,
    command: CommandID,
    expected_intent: MotorIntent,
    expected_physical_motion: str,
    duration_s: float,
    gap_s: float,
) -> EncoderSegment:
    before = bridge.read_encoders()
    steps.append(_encoder_result(f"{label}_before", before))

    command_exchange = bridge.send_command(command)
    steps.append(
        _state_result(
            f"{label}_command",
            command_exchange,
            expected_motor_intent=expected_intent,
        )
    )
    start = time.monotonic()
    time.sleep(duration_s)
    elapsed_s = time.monotonic() - start

    stop = bridge.send_command(CommandID.STOP)
    steps.append(_state_result(f"{label}_stop", stop, expected_motor_intent=MotorIntent.STOP))

    after = bridge.read_encoders()
    steps.append(_encoder_result(f"{label}_after", after))
    if gap_s > 0:
        time.sleep(gap_s)

    return EncoderSegment(
        label=label,
        command=command.name,
        expected_physical_motion=expected_physical_motion,
        elapsed_s=elapsed_s,
        left_delta=after.packet.left_count - before.packet.left_count,
        right_delta=after.packet.right_count - before.packet.right_count,
        requested_duration_s=duration_s,
    )


def _state_result(
    label: str,
    exchange: BridgeExchange,
    expected_motor_intent: MotorIntent | None = None,
) -> dict[str, object]:
    return {
        "kind": "state",
        "label": label,
        "command": exchange.command.name,
        "sequence": exchange.sequence,
        "state_sequence": exchange.state.sequence,
        "mcu_state": exchange.state.state.name,
        "mcu_error": exchange.state.error.name,
        "motor_intent": exchange.state.motor_intent.name,
        "expected_motor_intent": (
            expected_motor_intent.name if expected_motor_intent is not None else None
        ),
        "latency_ms": exchange.latency_ms,
    }


def _encoder_result(label: str, exchange: EncoderExchange) -> dict[str, object]:
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


def _segment_result(segment: EncoderSegment) -> dict[str, object]:
    return {
        "label": segment.label,
        "command": segment.command,
        "expected_physical_motion": segment.expected_physical_motion,
        "duration_s": (
            segment.requested_duration_s
            if segment.requested_duration_s is not None
            else segment.elapsed_s
        ),
        "elapsed_s": segment.elapsed_s,
        "left_delta": segment.left_delta,
        "right_delta": segment.right_delta,
        "left_counts_per_s": segment.left_counts_per_s,
        "right_counts_per_s": segment.right_counts_per_s,
    }


def _speed_segment_result(
    segment: EncoderSegment, left_sign: int, right_sign: int
) -> dict[str, object]:
    item = _segment_result(segment)
    item["left_normalized_counts_per_s"] = segment.left_counts_per_s * left_sign
    item["right_normalized_counts_per_s"] = segment.right_counts_per_s * right_sign
    return item


def _infer_encoder_mapping(segments: list[EncoderSegment]) -> str:
    right_only = _find_segment(segments, "turn_left_right_wheel")
    left_only = _find_segment(segments, "turn_right_left_wheel")
    if right_only is None or left_only is None:
        return "unknown"

    right_field_moves_for_right_wheel = abs(right_only.right_delta) > abs(
        right_only.left_delta
    )
    left_field_moves_for_left_wheel = abs(left_only.left_delta) > abs(
        left_only.right_delta
    )
    swapped = (
        abs(right_only.left_delta) > abs(right_only.right_delta)
        and abs(left_only.right_delta) > abs(left_only.left_delta)
    )

    if right_field_moves_for_right_wheel and left_field_moves_for_left_wheel:
        return "ok"
    if swapped:
        return "swapped"
    return "ambiguous"


def _build_checks(
    result: dict[str, object], steps: list[dict[str, object]]
) -> dict[str, bool]:
    state_steps = [step for step in steps if step["kind"] == "state"]
    encoder_steps = [step for step in steps if step["kind"] == "encoder"]
    direction_segments = result.get("direction_segments", [])
    speed_segments = result.get("speed_segments", [])
    return {
        "all_state_errors_ok": all(
            step["mcu_error"] == ErrorCode.OK.name for step in state_steps
        ),
        "state_sequence_echo_ok": all(
            step["sequence"] == step["state_sequence"] for step in state_steps
        ),
        "encoder_sequence_echo_ok": all(
            step["sequence"] == step["packet_sequence"] for step in encoder_steps
        ),
        "motor_intents_match": all(
            step["expected_motor_intent"] is None
            or step["motor_intent"] == step["expected_motor_intent"]
            for step in state_steps
        ),
        "ended_with_stop": bool(state_steps)
        and state_steps[-1]["motor_intent"] == MotorIntent.STOP.name,
        "encoder_mapping_ok": result.get("encoder_mapping") == "ok",
        "forward_signs_known": all(
            result["odom_recommendation"][key] != 0
            for key in ("left_encoder_sign", "right_encoder_sign")
        ),
        "any_direction_count_changed": any(
            item["left_delta"] != 0 or item["right_delta"] != 0
            for item in direction_segments
        ),
        "any_speed_count_changed": any(
            item["left_delta"] != 0 or item["right_delta"] != 0
            for item in speed_segments
        ),
    }


def _find_segment(
    segments: list[EncoderSegment], label: str
) -> EncoderSegment | None:
    return next((segment for segment in segments if segment.label == label), None)


def _forward_sign(delta: int) -> int:
    if delta > 0:
        return 1
    if delta < 0:
        return -1
    return 0


def _counts_per_s(delta: int, elapsed_s: float) -> float:
    if elapsed_s <= 0:
        return 0.0
    return delta / elapsed_s


if __name__ == "__main__":
    raise SystemExit(main())
