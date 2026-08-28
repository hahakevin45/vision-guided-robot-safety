from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from vgr_core.model import CommandID, ErrorCode, MotorIntent

from vgr_driver.driver.controller_bridge import ControllerBridge
from vgr_driver.driver.serial_transport import PosixSerial


@dataclass(frozen=True)
class MarkerObservation:
    frame_index: int
    t_s: float
    cx: float
    cy: float
    area: int
    angle_rad: float


# Spinning a single wheel to turn the car: TURN_RIGHT drives only the left
# wheel (car pivots right), TURN_LEFT drives only the right wheel. Physically
# correct now that motor channels are mapped left/right in firmware.
WHEEL_COMMAND = {
    "left": CommandID.TURN_RIGHT,
    "right": CommandID.TURN_LEFT,
}

# Firmware now normalizes both encoders to forward = positive
# (left encoder polarity is inverted in stm32_encoder.c), so no host-side
# sign compensation is needed.
ENCODER_SIGN = {
    "left": 1,
    "right": 1,
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Use camera marker angle and encoder counts to pulse a wheel toward a target revolution count."
    )
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--wheel", choices=("left", "right"), required=True)
    parser.add_argument("--target-revolutions", type=float, default=1.0)
    parser.add_argument("--encoder-counts-per-rev", type=float, required=True)
    parser.add_argument("--pulse-s", type=float, default=0.08)
    parser.add_argument("--controller", choices=("fixed", "pid"), default="fixed")
    parser.add_argument("--pid-kp", type=float, default=0.00018)
    parser.add_argument("--pid-ki", type=float, default=0.0)
    parser.add_argument("--pid-kd", type=float, default=0.00008)
    parser.add_argument("--min-pulse-s", type=float, default=0.01)
    parser.add_argument("--max-pulse-s", type=float, default=0.08)
    parser.add_argument("--post-stop-s", type=float, default=0.25)
    parser.add_argument("--max-pulses", type=int, default=20)
    parser.add_argument("--wheel-center-x", type=float, default=335.0)
    parser.add_argument("--wheel-center-y", type=float, default=200.0)
    parser.add_argument("--marker-min-radius-px", type=float, default=10.0)
    parser.add_argument("--marker-max-radius-px", type=float, default=105.0)
    parser.add_argument("--marker-min-area", type=int, default=30)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--dry-camera-s", type=float, default=0.0)
    parser.add_argument("--agreement-tolerance-revolutions", type=float, default=0.25)
    parser.add_argument("--agreement-tolerance-ratio", type=float, default=0.15)
    parser.add_argument("--expected-encoder-direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--max-vision-step-revolutions", type=float, default=0.20)
    parser.add_argument("--max-reverse-step-ratio", type=float, default=0.20)
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("outputs/vision_wheel_revolution.json"),
    )
    args = parser.parse_args()

    result: dict[str, object] = {
        "pass": False,
        "camera_index": args.camera_index,
        "device": args.device,
        "wheel": args.wheel,
        "target_revolutions": args.target_revolutions,
        "encoder_counts_per_rev": args.encoder_counts_per_rev,
        "pulse_s": args.pulse_s,
        "controller": args.controller,
        "pid": {
            "kp": args.pid_kp,
            "ki": args.pid_ki,
            "kd": args.pid_kd,
            "min_pulse_s": args.min_pulse_s,
            "max_pulse_s": args.max_pulse_s,
        },
        "post_stop_s": args.post_stop_s,
        "max_pulses": args.max_pulses,
        "wheel_center": {
            "x": args.wheel_center_x,
            "y": args.wheel_center_y,
        },
        "pulses": [],
        "steps": [],
        "camera": {},
        "vision_estimate": {},
        "vision_observations": [],
        "encoder_estimate": {},
        "checks": {},
        "error": None,
    }

    observations: list[MarkerObservation] = []
    frames: list[tuple[int, float, np.ndarray, MarkerObservation | None]] = []
    stop_capture = threading.Event()
    capture_error: list[str] = []
    output_dir = args.report.parent / f"{args.report.stem}_frames"
    output_dir.mkdir(parents=True, exist_ok=True)

    capture_thread = threading.Thread(
        target=_capture_loop,
        kwargs={
            "camera_index": args.camera_index,
            "width": args.camera_width,
            "height": args.camera_height,
            "fps": args.camera_fps,
            "wheel_center": (args.wheel_center_x, args.wheel_center_y),
            "min_radius_px": args.marker_min_radius_px,
            "max_radius_px": args.marker_max_radius_px,
            "min_area": args.marker_min_area,
            "observations": observations,
            "frames": frames,
            "stop_capture": stop_capture,
            "capture_error": capture_error,
        },
        daemon=True,
    )
    capture_thread.start()

    try:
        time.sleep(max(args.settle_s, 0.3))
        if args.dry_camera_s > 0:
            time.sleep(args.dry_camera_s)
        else:
            _run_pulse_sequence(args, result)
        time.sleep(0.3)
    except Exception as exc:  # noqa: BLE001 - hardware CLI must preserve failure detail and always stop.
        result["error"] = str(exc)
        _, stop_cleanup_error = _best_effort_stop(args)
        if stop_cleanup_error is not None:
            result["stop_cleanup_error"] = stop_cleanup_error
    finally:
        stop_capture.set()
        capture_thread.join(timeout=2.0)

    if capture_error and result["error"] is None:
        result["error"] = capture_error[0]

    _save_representative_frames(output_dir, frames)
    result["camera"] = _camera_summary(frames, observations)
    result["vision_estimate"] = estimate_revolutions(
        observations,
        max_step_revolutions=args.max_vision_step_revolutions,
        max_reverse_step_ratio=args.max_reverse_step_ratio,
    )
    result["vision_observations"] = [_observation_result(obs) for obs in observations]
    if result["pulses"]:
        final_counts = result["pulses"][-1]["encoder_abs_total_counts"]
    else:
        final_counts = 0.0
    result["encoder_estimate"] = {
        "absolute_counts": final_counts,
        "absolute_revolutions": final_counts / args.encoder_counts_per_rev
        if args.encoder_counts_per_rev > 0
        else 0.0,
    }
    result["cross_calibration"] = build_cross_calibration(
        encoder_abs_counts=final_counts,
        encoder_counts_per_rev=args.encoder_counts_per_rev,
        vision_abs_revolutions=result["vision_estimate"].get("absolute_revolutions", 0.0),
        agreement_tolerance_revolutions=args.agreement_tolerance_revolutions,
        agreement_tolerance_ratio=args.agreement_tolerance_ratio,
    )
    result["checks"] = _build_checks(args, result)
    result["pass"] = all(result["checks"].values()) and result["error"] is None

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "pulses"}, indent=2))
    print("VISION WHEEL REVOLUTION:", "PASS" if result["pass"] else "FAIL")
    return 0 if result["pass"] else 1


def detect_yellow_marker(
    frame: np.ndarray,
    *,
    wheel_center: tuple[float, float],
    min_area: int = 30,
    previous_angle_rad: float | None = None,
    min_radius_px: float = 10.0,
    max_radius_px: float = 105.0,
) -> MarkerObservation | None:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([18, 60, 60]), np.array([45, 255, 255]))
    num, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    candidates: list[dict[str, float | int]] = []
    for idx in range(1, num):
        x, y, w, h, area = stats[idx]
        if int(area) < min_area:
            continue
        cx = float(centroids[idx][0])
        cy = float(centroids[idx][1])
        radius = math.hypot(cx - wheel_center[0], cy - wheel_center[1])
        if radius < min_radius_px or radius > max_radius_px:
            continue
        angle = math.atan2(cy - wheel_center[1], cx - wheel_center[0])
        candidates.append(
            {
                "x": int(x),
                "y": int(y),
                "w": int(w),
                "h": int(h),
                "area": int(area),
                "cx": cx,
                "cy": cy,
                "angle": angle,
            }
        )

    if not candidates:
        return None
    if previous_angle_rad is None:
        best = max(candidates, key=lambda item: int(item["area"]))
    else:
        best = min(
            candidates,
            key=lambda item: (
                _angle_distance(float(item["angle"]), previous_angle_rad),
                -int(item["area"]),
            ),
        )
    return MarkerObservation(
        frame_index=-1,
        t_s=0.0,
        cx=float(best["cx"]),
        cy=float(best["cy"]),
        area=int(best["area"]),
        angle_rad=float(best["angle"]),
    )


def estimate_revolutions(
    observations: list[MarkerObservation],
    *,
    max_step_revolutions: float = 0.20,
    max_reverse_step_ratio: float = 0.20,
) -> dict[str, object]:
    if len(observations) < 2:
        return {
            "valid_observations": len(observations),
            "signed_revolutions": 0.0,
            "absolute_revolutions": 0.0,
            "direction": 0,
            "rpm": 0.0,
            "tracking_reliable": False,
            "rejected_angle_steps": 0,
            "reverse_step_ratio": 0.0,
        }

    angles = np.array([obs.angle_rad for obs in observations], dtype=float)
    unwrapped = np.unwrap(angles)
    step_revolutions = np.abs(np.diff(unwrapped)) / (2.0 * math.pi)
    signed_steps = np.diff(unwrapped) / (2.0 * math.pi)
    rejected_steps = int(np.count_nonzero(step_revolutions > max_step_revolutions))
    signed_revolutions = float((unwrapped[-1] - unwrapped[0]) / (2.0 * math.pi))
    nonzero_steps = signed_steps[np.abs(signed_steps) > 1e-4]
    if len(nonzero_steps) > 0:
        dominant_direction = 1 if float(np.median(nonzero_steps)) > 0 else -1
        reverse_steps = int(np.count_nonzero(nonzero_steps * dominant_direction < 0))
        reverse_step_ratio = reverse_steps / len(nonzero_steps)
    else:
        reverse_steps = 0
        reverse_step_ratio = 0.0
    elapsed_s = observations[-1].t_s - observations[0].t_s
    direction = 1 if signed_revolutions > 0 else -1 if signed_revolutions < 0 else 0
    return {
        "valid_observations": len(observations),
        "first_t_s": observations[0].t_s,
        "last_t_s": observations[-1].t_s,
        "elapsed_s": elapsed_s,
        "signed_revolutions": signed_revolutions,
        "absolute_revolutions": abs(signed_revolutions),
        "direction": direction,
        "rpm": abs(signed_revolutions) / elapsed_s * 60.0 if elapsed_s > 0 else 0.0,
        "tracking_reliable": rejected_steps == 0
        and reverse_step_ratio <= max_reverse_step_ratio,
        "rejected_angle_steps": rejected_steps,
        "max_step_revolutions": max_step_revolutions,
        "reverse_steps": reverse_steps,
        "reverse_step_ratio": reverse_step_ratio,
        "max_reverse_step_ratio": max_reverse_step_ratio,
    }


def is_encoder_direction_valid(*, raw_delta: int, expected_direction: int) -> bool:
    if raw_delta == 0:
        return True
    return (raw_delta > 0 and expected_direction > 0) or (
        raw_delta < 0 and expected_direction < 0
    )


def compute_pid_pulse_s(
    *,
    error_counts: float,
    previous_error_counts: float,
    integral_counts: float,
    kp: float,
    ki: float,
    kd: float,
    min_pulse_s: float,
    max_pulse_s: float,
) -> float:
    derivative = error_counts - previous_error_counts
    pulse_s = (kp * error_counts) + (ki * integral_counts) + (kd * derivative)
    if pulse_s < min_pulse_s:
        return min_pulse_s
    if pulse_s > max_pulse_s:
        return max_pulse_s
    return pulse_s


def build_cross_calibration(
    *,
    encoder_abs_counts: float,
    encoder_counts_per_rev: float,
    vision_abs_revolutions: float,
    agreement_tolerance_revolutions: float = 0.10,
    agreement_tolerance_ratio: float = 0.15,
) -> dict[str, object]:
    encoder_revolutions = (
        encoder_abs_counts / encoder_counts_per_rev
        if encoder_counts_per_rev > 0
        else 0.0
    )
    counts_per_visual_revolution = (
        encoder_abs_counts / vision_abs_revolutions
        if vision_abs_revolutions > 0
        else None
    )
    delta_revolutions = abs(vision_abs_revolutions - encoder_revolutions)
    relative_delta_ratio = (
        delta_revolutions / max(encoder_revolutions, vision_abs_revolutions)
        if max(encoder_revolutions, vision_abs_revolutions) > 0
        else 0.0
    )
    agree = (
        delta_revolutions <= agreement_tolerance_revolutions
        and relative_delta_ratio <= agreement_tolerance_ratio
    )
    return {
        "encoder_revolutions_from_input_calibration": encoder_revolutions,
        "vision_revolutions": vision_abs_revolutions,
        "vision_encoder_delta_revolutions": delta_revolutions,
        "vision_encoder_relative_delta_ratio": relative_delta_ratio,
        "agreement_tolerance_revolutions": agreement_tolerance_revolutions,
        "agreement_tolerance_ratio": agreement_tolerance_ratio,
        "vision_encoder_agree": agree,
        "vision_to_encoder_revolution_ratio": (
            vision_abs_revolutions / encoder_revolutions
            if encoder_revolutions > 0
            else None
        ),
        "counts_per_visual_revolution": counts_per_visual_revolution,
    }


def _angle_distance(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _run_pulse_sequence(args: argparse.Namespace, result: dict[str, object]) -> None:
    command = WHEEL_COMMAND[args.wheel]
    encoder_sign = ENCODER_SIGN[args.wheel]
    target_counts = args.target_revolutions * args.encoder_counts_per_rev
    with PosixSerial(args.device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
        time.sleep(args.settle_s)
        serial.flush_input()
        bridge = ControllerBridge(serial)
        result["steps"].append(_state_result("pre_heartbeat", bridge.send_command(CommandID.HEARTBEAT)))
        result["steps"].append(_state_result("pre_stop", bridge.send_command(CommandID.STOP)))
        before = bridge.read_encoders()
        result["steps"].append(_encoder_result("initial_encoder", before))
        previous_error = target_counts
        integral = 0.0

        for pulse_index in range(args.max_pulses):
            pulse_before = bridge.read_encoders()
            total_before = abs(_wheel_delta(args.wheel, before, pulse_before))
            error_counts = max(0.0, target_counts - total_before)
            if error_counts <= 0.0:
                break
            integral += error_counts
            pulse_s = _next_pulse_s(args, error_counts, previous_error, integral)
            start = time.monotonic()
            state = bridge.send_command(command)
            time.sleep(pulse_s)
            stop = bridge.send_command(CommandID.STOP)
            time.sleep(args.post_stop_s)
            pulse_after = bridge.read_encoders()

            total_raw = _wheel_delta(args.wheel, before, pulse_after)
            pulse_raw = _wheel_delta(args.wheel, pulse_before, pulse_after)
            total_abs = abs(total_raw)
            pulse_abs = abs(pulse_raw)
            direction_ok = is_encoder_direction_valid(
                raw_delta=total_raw,
                expected_direction=args.expected_encoder_direction,
            )
            pulse = {
                "pulse_index": pulse_index,
                "command": command.name,
                "controller": args.controller,
                "requested_pulse_s": pulse_s,
                "elapsed_s": time.monotonic() - start,
                "state": _state_result("pulse_command", state),
                "stop": _state_result("pulse_stop", stop),
                "error_counts_before_pulse": error_counts,
                "pulse_raw_counts": pulse_raw,
                "encoder_direction_ok": direction_ok,
                "pulse_normalized_counts": pulse_raw * encoder_sign,
                "pulse_abs_counts": pulse_abs,
                "encoder_raw_total_counts": total_raw,
                "encoder_normalized_total_counts": total_raw * encoder_sign,
                "encoder_abs_total_counts": total_abs,
                "target_counts": target_counts,
                "target_reached": total_abs >= target_counts,
            }
            result["pulses"].append(pulse)
            print(
                f"pulse {pulse_index}: abs_counts={total_abs:.1f}/{target_counts:.1f} "
                f"pulse_abs={pulse_abs:.1f} pulse_s={pulse_s:.3f}",
                flush=True,
            )
            if pulse["target_reached"]:
                break
            if not direction_ok:
                break
            previous_error = max(0.0, target_counts - total_abs)

        result["steps"].append(_state_result("cleanup_stop", bridge.send_command(CommandID.STOP)))


def _next_pulse_s(
    args: argparse.Namespace,
    error_counts: float,
    previous_error_counts: float,
    integral_counts: float,
) -> float:
    if args.controller == "fixed":
        return args.pulse_s
    return compute_pid_pulse_s(
        error_counts=error_counts,
        previous_error_counts=previous_error_counts,
        integral_counts=integral_counts,
        kp=args.pid_kp,
        ki=args.pid_ki,
        kd=args.pid_kd,
        min_pulse_s=args.min_pulse_s,
        max_pulse_s=args.max_pulse_s,
    )


def _capture_loop(
    *,
    camera_index: int,
    width: int,
    height: int,
    fps: float,
    wheel_center: tuple[float, float],
    min_radius_px: float,
    max_radius_px: float,
    min_area: int,
    observations: list[MarkerObservation],
    frames: list[tuple[int, float, np.ndarray, MarkerObservation | None]],
    stop_capture: threading.Event,
    capture_error: list[str],
) -> None:
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        capture_error.append(f"failed to open camera index {camera_index}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    start = time.monotonic()
    frame_index = 0
    previous_angle: float | None = None
    try:
        while not stop_capture.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            t_s = time.monotonic() - start
            observation = detect_yellow_marker(
                frame,
                wheel_center=wheel_center,
                previous_angle_rad=previous_angle,
                min_area=min_area,
                min_radius_px=min_radius_px,
                max_radius_px=max_radius_px,
            )
            if observation is not None:
                observation = MarkerObservation(
                    frame_index=frame_index,
                    t_s=t_s,
                    cx=observation.cx,
                    cy=observation.cy,
                    area=observation.area,
                    angle_rad=observation.angle_rad,
                )
                previous_angle = observation.angle_rad
                observations.append(observation)
            if frame_index % 5 == 0:
                frames.append((frame_index, t_s, frame.copy(), observation))
            frame_index += 1
    finally:
        cap.release()


def _save_representative_frames(
    output_dir: Path,
    frames: list[tuple[int, float, np.ndarray, MarkerObservation | None]],
) -> None:
    if not frames:
        return
    picks = sorted(set([0, len(frames) // 3, 2 * len(frames) // 3, len(frames) - 1]))
    for save_index, frame_index in enumerate(picks):
        original_index, _t_s, frame, observation = frames[frame_index]
        annotated = frame.copy()
        if observation is not None:
            cv2.circle(annotated, (int(observation.cx), int(observation.cy)), 5, (0, 255, 0), -1)
            cv2.putText(
                annotated,
                f"{math.degrees(observation.angle_rad):.1f} deg",
                (int(observation.cx) + 8, int(observation.cy)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        cv2.imwrite(str(output_dir / f"frame_{save_index:02d}_{original_index:04d}.jpg"), annotated)


def _camera_summary(
    frames: list[tuple[int, float, np.ndarray, MarkerObservation | None]],
    observations: list[MarkerObservation],
) -> dict[str, object]:
    if not frames:
        return {"frames_saved_for_analysis": 0, "observations": len(observations), "measured_fps": 0.0}
    elapsed_s = frames[-1][1] - frames[0][1]
    return {
        "frames_saved_for_analysis": len(frames),
        "observations": len(observations),
        "first_saved_t_s": frames[0][1],
        "last_saved_t_s": frames[-1][1],
        "measured_saved_frame_rate": (len(frames) - 1) / elapsed_s if elapsed_s > 0 else 0.0,
    }


def _observation_result(observation: MarkerObservation) -> dict[str, object]:
    return {
        "frame_index": observation.frame_index,
        "t_s": observation.t_s,
        "cx": observation.cx,
        "cy": observation.cy,
        "area": observation.area,
        "angle_rad": observation.angle_rad,
        "angle_deg": math.degrees(observation.angle_rad),
    }


def _build_checks(args: argparse.Namespace, result: dict[str, object]) -> dict[str, bool]:
    vision = result["vision_estimate"]
    encoder = result["encoder_estimate"]
    state_steps = [step for step in result["steps"] if step["kind"] == "state"]
    return {
        "camera_observations_present": result["camera"].get("observations", 0) >= 5,
        "vision_motion_detected": vision.get("absolute_revolutions", 0.0) > 0.05,
        "vision_tracking_reliable": bool(vision.get("tracking_reliable", False)),
        "encoder_motion_detected": encoder.get("absolute_counts", 0.0) > 0,
        "encoder_direction_ok": all(
            pulse.get("encoder_direction_ok", False)
            for pulse in result.get("pulses", [])
        ),
        "encoder_target_reached": encoder.get("absolute_revolutions", 0.0) >= args.target_revolutions,
        "vision_encoder_agree": bool(
            result.get("cross_calibration", {}).get("vision_encoder_agree", False)
        ),
        "all_state_errors_ok": all(step["mcu_error"] == ErrorCode.OK.name for step in state_steps),
        "ended_with_stop": bool(state_steps) and state_steps[-1]["motor_intent"] == MotorIntent.STOP.name,
    }


def _wheel_delta(wheel: str, before, after) -> int:
    if wheel == "left":
        return after.packet.left_count - before.packet.left_count
    return after.packet.right_count - before.packet.right_count


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


def _best_effort_stop(args: argparse.Namespace) -> tuple[bool, str | None]:
    try:
        with PosixSerial(args.device, baudrate=args.baudrate, timeout_s=args.timeout_s) as serial:
            time.sleep(0.2)
            serial.flush_input()
            bridge = ControllerBridge(serial)
            bridge.send_command(CommandID.HEARTBEAT)
            state = bridge.send_command(CommandID.STOP)
            return state.state.motor_intent == MotorIntent.STOP, None
    except Exception as exc:  # noqa: BLE001 - cleanup-path failure must be reported, not swallowed.
        return False, f"{type(exc).__name__}: {exc}"


if __name__ == "__main__":
    raise SystemExit(main())
