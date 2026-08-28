"""Staged live-hardware wheel-takeover validator.

Stage 1: no-motion safety checks (serial open, heartbeat, STOP, encoder static, camera frame).
Stage 2: bounded motion pulse (only if stage 1 fully passes AND real hardware present).
Stage 3: finalize with STOP, emit JSON + Markdown report.

Motion limits: pulse <= 2.0s, target <= 0.03 rev (~0.006m).
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shlex
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from vgr_core.model import CommandID, MotorIntent
from vgr_driver.driver import BridgeExchange, ControllerBridge
from vgr_driver.driver import PosixSerial

try:
    import cv2

    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Device detection
# ---------------------------------------------------------------------------


def detect_serial_device(requested: str) -> tuple[str | None, list[str]]:
    """Return (chosen_path|None, all_candidates).

    Prefer *requested* if it exists; otherwise fall back to ttyACM* then ttyUSB*.
    """
    candidates: list[str] = []
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        candidates.extend(sorted(glob.glob(pattern)))

    if requested in candidates:
        return requested, candidates
    if candidates:
        return candidates[0], candidates
    return None, candidates


def is_real_serial_device(device_path: str | None) -> bool:
    """Return True only for real USB serial device nodes, never PTYs."""
    if not device_path:
        return False
    try:
        resolved = os.path.realpath(device_path)
    except OSError:
        resolved = device_path
    return resolved.startswith("/dev/ttyACM") or resolved.startswith("/dev/ttyUSB")


def detect_camera(index: int) -> dict[str, Any]:
    """Try to open camera at *index*; probe 0 and 1 as fallback.

    Returns dict with keys: opened, frame_read, width, height, actual_index, error.
    """
    result: dict[str, Any] = {
        "opened": False,
        "frame_read": False,
        "width": None,
        "height": None,
        "actual_index": None,
        "error": None,
    }
    if not _CV2_AVAILABLE:
        result["error"] = "cv2 not available"
        return result

    probe_indices = [index] + [i for i in (0, 1) if i != index]
    for idx in probe_indices:
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            h, w = frame.shape[:2]
            result.update(
                {
                    "opened": True,
                    "frame_read": True,
                    "width": w,
                    "height": h,
                    "actual_index": idx,
                }
            )
            return result
        # opened but read() failed — continue probing remaining indices
    result["error"] = "no camera found at indices probed"
    return result


# ---------------------------------------------------------------------------
# Stage 1 checks
# ---------------------------------------------------------------------------


def _check(name: str, *, passed: bool, detail: str, raw: Any = None) -> dict[str, Any]:
    return {"name": name, "pass": passed, "detail": detail, "raw": raw}


def run_stage1(
    bridge: ControllerBridge,
    camera_index: int,
    settle_s: float,
    timeout_s: float,
) -> tuple[list[dict[str, Any]], str | None]:
    """Run all no-motion checks.

    Returns (checks_list, stop_reason|None).  stop_reason is set on first failure.
    """
    checks: list[dict[str, Any]] = []
    stop_reason: str | None = None

    # (a) serial_open — already open if we reach here, record success
    checks.append(_check("serial_open", passed=True, detail="PosixSerial opened successfully"))

    # (b) heartbeat
    try:
        t0 = time.monotonic()
        hb: BridgeExchange = bridge.send_command(CommandID.HEARTBEAT)
        latency_ms = (time.monotonic() - t0) * 1000.0
        raw_hb = {
            "mcu_state": hb.state.state.name,
            "error": hb.state.error.name,
            "motor_intent": hb.state.motor_intent.name,
            "latency_ms": round(latency_ms, 2),
        }
        ok = hb.state.error.name == "OK"
        checks.append(_check("heartbeat", passed=ok, detail=f"MCU state={hb.state.state.name} error={hb.state.error.name}", raw=raw_hb))
        if not ok:
            stop_reason = f"heartbeat error: {hb.state.error.name}"
            return checks, stop_reason
    except Exception as exc:
        checks.append(_check("heartbeat", passed=False, detail=f"exception: {exc}"))
        stop_reason = f"heartbeat exception: {exc}"
        return checks, stop_reason

    # (c) STOP
    try:
        stop_ex: BridgeExchange = bridge.send_command(CommandID.STOP)
        stop_ok = stop_ex.state.motor_intent == MotorIntent.STOP and stop_ex.state.error.name == "OK"
        raw_stop = {
            "mcu_state": stop_ex.state.state.name,
            "error": stop_ex.state.error.name,
            "motor_intent": stop_ex.state.motor_intent.name,
        }
        checks.append(_check("stop_command", passed=stop_ok, detail=f"motor_intent={stop_ex.state.motor_intent.name} error={stop_ex.state.error.name}", raw=raw_stop))
        if not stop_ok:
            stop_reason = f"STOP failed: motor_intent={stop_ex.state.motor_intent.name} error={stop_ex.state.error.name}"
            return checks, stop_reason
    except Exception as exc:
        checks.append(_check("stop_command", passed=False, detail=f"exception: {exc}"))
        stop_reason = f"STOP command exception: {exc}"
        return checks, stop_reason

    # (d) encoder_static — two reads ~0.3s apart, assert |delta| <= 2 per side
    try:
        enc1 = bridge.read_encoders()
        time.sleep(0.3)
        enc2 = bridge.read_encoders()
        delta_left = abs(enc2.packet.left_count - enc1.packet.left_count)
        delta_right = abs(enc2.packet.right_count - enc1.packet.right_count)
        enc_ok = delta_left <= 2 and delta_right <= 2
        raw_enc = {
            "left1": enc1.packet.left_count,
            "right1": enc1.packet.right_count,
            "left2": enc2.packet.left_count,
            "right2": enc2.packet.right_count,
            "delta_left": delta_left,
            "delta_right": delta_right,
        }
        detail = f"delta_left={delta_left} delta_right={delta_right} (threshold=2)"
        checks.append(_check("encoder_static", passed=enc_ok, detail=detail, raw=raw_enc))
        if not enc_ok:
            stop_reason = f"encoder moving at rest: delta_left={delta_left} delta_right={delta_right}"
            return checks, stop_reason
    except Exception as exc:
        checks.append(_check("encoder_static", passed=False, detail=f"exception: {exc}"))
        stop_reason = f"encoder_static exception: {exc}"
        return checks, stop_reason

    # (e) camera_frame
    cam = detect_camera(camera_index)
    cam_ok = cam["frame_read"] is True
    cam_detail = (
        f"camera index {cam.get('actual_index')} {cam.get('width')}x{cam.get('height')}"
        if cam_ok
        else f"camera fail: {cam.get('error')}"
    )
    checks.append(_check("camera_frame", passed=cam_ok, detail=cam_detail, raw=cam))
    if not cam_ok:
        stop_reason = f"camera_frame failed: {cam.get('error')}"
        return checks, stop_reason

    return checks, None


# ---------------------------------------------------------------------------
# Stage 2 motion
# ---------------------------------------------------------------------------

MAX_MOTOR_SECONDS_HARD_CAP = 2.0
MAX_TARGET_REVOLUTIONS = 0.03


def run_stage2_motion(
    bridge: ControllerBridge,
    pulse_s: float,
    target_revolutions: float,
    max_motor_seconds: float,
    camera_index: int = 0,
) -> dict[str, Any]:
    """Bounded motion: STOP(pre) → camera → read_encoders → FORWARD(pulse) → STOP → read_encoders → camera.

    Returns motion_stage dict with keys: ran, skipped_reason, wheels_moved, exchanges,
    camera_before, camera_after, error.
    """
    # Enforce hard caps
    pulse_s = min(pulse_s, MAX_MOTOR_SECONDS_HARD_CAP, max_motor_seconds)
    target_revolutions = min(target_revolutions, MAX_TARGET_REVOLUTIONS)

    motion: dict[str, Any] = {
        "ran": True,
        "skipped_reason": None,
        "wheels_moved": False,
        "exchanges": [],
        "pulse_s_used": pulse_s,
        "target_revolutions_used": target_revolutions,
        "camera_before": None,
        "camera_after": None,
        "error": None,
    }

    def _record(label: str, ex: BridgeExchange | None, *, ok: bool, detail: str = "") -> dict[str, Any]:
        entry: dict[str, Any] = {
            "label": label,
            "pass": ok,
            "detail": detail,
        }
        if ex is not None:
            entry["raw"] = {
                "command": ex.command.name,
                "mcu_state": ex.state.state.name,
                "motor_intent": ex.state.motor_intent.name,
                "error": ex.state.error.name,
                "latency_ms": round(ex.latency_ms, 2),
            }
        motion["exchanges"].append(entry)
        return entry

    try:
        # pre-STOP
        pre_stop = bridge.send_command(CommandID.STOP)
        pre_ok = pre_stop.state.motor_intent == MotorIntent.STOP and pre_stop.state.error.name == "OK"
        _record("pre_stop", pre_stop, ok=pre_ok, detail=f"motor_intent={pre_stop.state.motor_intent.name} error={pre_stop.state.error.name}")
        if not pre_ok:
            motion["error"] = f"pre-STOP failed: motor_intent={pre_stop.state.motor_intent.name}"
            motion["ran"] = False
            return motion

        # camera frame before motion
        cam_before = detect_camera(camera_index)
        motion["camera_before"] = cam_before
        motion["exchanges"].append({
            "label": "camera_before",
            "pass": cam_before["frame_read"] is True,
            "detail": (f"index={cam_before.get('actual_index')} {cam_before.get('width')}x{cam_before.get('height')}"
                       if cam_before["frame_read"] else cam_before.get("error")),
            "raw": cam_before,
        })

        # read encoders before
        enc_before = bridge.read_encoders()
        motion["exchanges"].append({"label": "encoder_before", "pass": True, "raw": {
            "left": enc_before.packet.left_count,
            "right": enc_before.packet.right_count,
        }})

        # FORWARD pulse
        fwd = bridge.send_command(CommandID.FORWARD)
        fwd_ok = fwd.state.error.name == "OK"
        _record(
            "forward_command",
            fwd,
            ok=fwd_ok,
            detail=f"motor_intent={fwd.state.motor_intent.name} error={fwd.state.error.name} mcu_state={fwd.state.state.name}",
        )
        if not fwd_ok:
            motion["error"] = (
                f"FORWARD command fault: mcu_state={fwd.state.state.name}"
                f" error={fwd.state.error.name}"
            )
            # Emergency STOP after fault
            try:
                abort_stop = bridge.send_command(CommandID.STOP)
                abort_ok = abort_stop.state.motor_intent == MotorIntent.STOP
                _record("abort_stop", abort_stop, ok=abort_ok, detail="emergency STOP after FORWARD fault")
            except Exception as abort_exc:
                _record("abort_stop", None, ok=False, detail=f"abort STOP exception: {abort_exc}")
            return motion

        time.sleep(pulse_s)

        # STOP immediately
        post_stop = bridge.send_command(CommandID.STOP)
        post_ok = post_stop.state.motor_intent == MotorIntent.STOP and post_stop.state.error.name == "OK"
        _record("post_stop", post_stop, ok=post_ok, detail=f"motor_intent={post_stop.state.motor_intent.name} error={post_stop.state.error.name}")
        if not post_ok:
            motion["error"] = f"post-STOP failed: motor_intent={post_stop.state.motor_intent.name}"
            return motion

        time.sleep(0.1)

        # read encoders after
        enc_after = bridge.read_encoders()
        motion["exchanges"].append({"label": "encoder_after", "pass": True, "raw": {
            "left": enc_after.packet.left_count,
            "right": enc_after.packet.right_count,
        }})

        delta_left = abs(enc_after.packet.left_count - enc_before.packet.left_count)
        delta_right = abs(enc_after.packet.right_count - enc_before.packet.right_count)
        motion["wheels_moved"] = delta_left > 3 or delta_right > 3
        motion["encoder_delta"] = {"left": delta_left, "right": delta_right}

        # camera frame after motion
        cam_after = detect_camera(camera_index)
        motion["camera_after"] = cam_after
        motion["exchanges"].append({
            "label": "camera_after",
            "pass": cam_after["frame_read"] is True,
            "detail": (f"index={cam_after.get('actual_index')} {cam_after.get('width')}x{cam_after.get('height')}"
                       if cam_after["frame_read"] else cam_after.get("error")),
            "raw": cam_after,
        })

    except Exception as exc:
        motion["error"] = f"motion stage exception: {exc}"

    return motion


# ---------------------------------------------------------------------------
# Stage 3 finalize
# ---------------------------------------------------------------------------


def finalize_stop(bridge: ControllerBridge) -> tuple[str, bool]:
    """Send final STOP, return (state_name, ok)."""
    try:
        ex = bridge.send_command(CommandID.STOP)
        ok = ex.state.motor_intent == MotorIntent.STOP and ex.state.error.name == "OK"
        return ex.state.state.name, ok
    except Exception as exc:
        return f"exception:{exc}", False


# ---------------------------------------------------------------------------
# Report writing
# ---------------------------------------------------------------------------


def write_report_md(result: dict[str, Any], path: Path) -> None:
    lines: list[str] = []

    real_hw = result.get("real_hardware_used", False)
    overall = result.get("pass", False)
    camera_check = next((c for c in result.get("stage1_checks", []) if c.get("name") == "camera_frame"), None)
    camera_unavailable = bool(
        real_hw
        and camera_check
        and not camera_check.get("pass", False)
    )

    if not real_hw:
        if result.get("serial_device"):
            # Device was detected in filesystem but could not be opened (e.g. permission denied)
            lines.append("# HARDWARE NOT EXERCISED - serial device found but failed to open")
            lines.append("")
            lines.append("**Overall result: FAIL**")
            lines.append("")
            lines.append(f"> Serial device `{result['serial_device']}` was detected but could not be opened.")
            lines.append(f"> Failure: {result.get('stop_reason', 'unknown')}")
            lines.append("> This report does NOT represent a hardware validation.")
            lines.append("")
        else:
            lines.append("# HARDWARE NOT EXERCISED - no serial MCU detected")
            lines.append("")
            lines.append("**Overall result: FAIL**")
            lines.append("")
            lines.append("> This report does NOT represent a hardware validation.")
            lines.append("> No serial MCU was detected at the requested device path.")
            lines.append("")
    elif camera_unavailable:
        lines.append("# CAMERA NOT AVAILABLE - motion not exercised")
        lines.append("")
        lines.append("**Overall result: FAIL**")
        lines.append("")
        lines.append("> Serial MCU communication ran, but no real camera capture device was available.")
        lines.append(f"> Failure: {result.get('stop_reason', 'camera unavailable')}")
        lines.append("> Wheels were not exercised.")
        lines.append("")
    else:
        status = "PASS" if overall else "FAIL"
        lines.append(f"# Pi Live Hardware Validation — {status}")
        lines.append("")

    lines.append(f"**Timestamp:** {result.get('timestamp_utc', 'N/A')}")
    lines.append(f"**Command:** `{result.get('command_line', 'N/A')}`")
    lines.append(f"**Real hardware used:** {'YES' if real_hw else 'NO'}")
    lines.append(f"**Serial device:** {result.get('serial_device') or 'NOT FOUND'}")
    lines.append(f"**Serial candidates detected:** {result.get('serial_candidates', [])}")
    cam = result.get("camera_info", {})
    if cam.get("actual_index") is None or cam.get("width") is None or cam.get("height") is None:
        cam_detail = f"actual={cam.get('actual_index')} unavailable"
        if cam.get("error"):
            cam_detail += f" ({cam.get('error')})"
    else:
        cam_detail = f"actual={cam.get('actual_index')} {cam.get('width')}x{cam.get('height')}"
    lines.append(f"**Camera index:** {result.get('camera_index')} → {cam_detail}")
    lines.append("")

    lines.append("## Stage 1: No-Motion Safety Checks")
    lines.append("")
    for chk in result.get("stage1_checks", []):
        status_icon = "✓" if chk["pass"] else "✗"
        lines.append(f"- [{status_icon}] **{chk['name']}**: {chk['detail']}")
    lines.append("")

    stop_reason = result.get("stop_reason")
    if stop_reason:
        lines.append(f"**Stage 1 stop reason:** {stop_reason}")
        lines.append("")

    ms = result.get("motion_stage", {})
    lines.append("## Stage 2: Motion Test")
    if ms.get("ran"):
        lines.append(f"- Pulse used: {ms.get('pulse_s_used')}s")
        lines.append(f"- Target revolutions: {ms.get('target_revolutions_used')}")
        lines.append(f"- **Wheels moved:** {'YES' if ms.get('wheels_moved') else 'NO'}")
        enc_delta = ms.get("encoder_delta", {})
        lines.append(f"- Encoder delta: left={enc_delta.get('left')} right={enc_delta.get('right')}")
        if ms.get("error"):
            lines.append(f"- **Error:** {ms['error']}")
        lines.append("")
        lines.append("### Motion exchanges")
        for ex in ms.get("exchanges", []):
            icon = "✓" if ex.get("pass") else "✗"
            lines.append(f"  - [{icon}] {ex['label']}: {ex.get('detail', '')} raw={ex.get('raw')}")
    else:
        reason = ms.get("skipped_reason") or "stage 1 not fully passed or no real hardware"
        lines.append(f"- **Skipped**: {reason}")
        lines.append("- Wheels moved: NO")
    lines.append("")

    lines.append("## Final STOP State")
    lines.append(f"- final_stop_state: {result.get('final_stop_state')}")
    lines.append(f"- final_stop_ok: {result.get('final_stop_ok')}")
    lines.append("")

    err = result.get("error")
    if err:
        lines.append(f"## Error\n\n{err}")
        lines.append("")

    lines.append("## Overall Result")
    lines.append(f"**{'PASS' if overall else 'FAIL'}**")
    if not real_hw:
        lines.append("")
        lines.append("> HARDWARE NOT EXERCISED — connect serial MCU to run real hardware validation.")
    elif camera_unavailable:
        lines.append("")
        lines.append("> CAMERA NOT AVAILABLE — connect a real capture device before attempting motion.")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Main orchestrator
# ---------------------------------------------------------------------------


def run_validation(
    serial_device: str,
    baudrate: int,
    camera_index: int,
    timeout_s: float,
    settle_s: float,
    pulse_s: float,
    target_revolutions: float,
    max_motor_seconds: float,
    allow_motion: bool,
    mock_serial: bool,
) -> dict[str, Any]:
    now = datetime.now(timezone.utc)
    command_line = " ".join(shlex.quote(a) for a in sys.argv)

    result: dict[str, Any] = {
        "pass": False,
        "real_hardware_used": False,
        "serial_device": None,
        "serial_candidates": [],
        "camera_index": camera_index,
        "camera_info": {},
        "stage1_checks": [],
        "motion_stage": {
            "ran": False,
            "skipped_reason": "not started",
            "wheels_moved": False,
            "exchanges": [],
            "error": None,
        },
        "final_stop_state": None,
        "final_stop_ok": False,
        "wheels_moved": False,
        "stop_reason": None,
        "error": None,
        "timestamp_utc": now.isoformat(),
        "mock_serial": mock_serial,
        "command_line": command_line,
    }

    if mock_serial:
        # Import pty here — only needed for mock path
        import os
        import pty

        from vgr_driver.driver.mock_serial_mcu import MockSerialMCU

        master_fd, slave_fd = pty.openpty()
        mock_mcu = MockSerialMCU(master_fd)
        mock_mcu.start()
        device_path = os.ttyname(slave_fd)
        result["serial_device"] = device_path
        result["serial_candidates"] = [device_path]
        result["real_hardware_used"] = False  # mock is NEVER real hardware
        result["stop_reason"] = "mock_serial mode: real_hardware_used=False, motion gate will block"
        result["motion_stage"]["skipped_reason"] = "mock_serial: real_hardware_used=False"

        try:
            serial = PosixSerial(device=device_path, baudrate=baudrate, timeout_s=timeout_s)
            serial.open()
            bridge = ControllerBridge(serial)

            checks, stop_reason = run_stage1(bridge, camera_index, settle_s, timeout_s)
            result["stage1_checks"] = checks
            if stop_reason:
                result["stop_reason"] = stop_reason

            final_state, final_ok = finalize_stop(bridge)
            result["final_stop_state"] = final_state
            result["final_stop_ok"] = final_ok
            serial.close()
        finally:
            mock_mcu.stop()
            os.close(master_fd)
            os.close(slave_fd)

        result["pass"] = False  # mock run always FAIL (not real hardware)
        return result

    # Real hardware path
    chosen_device, candidates = detect_serial_device(serial_device)
    result["serial_candidates"] = candidates
    result["serial_device"] = chosen_device

    if chosen_device is None:
        detail = f"no serial device found; requested={serial_device} candidates={candidates}"
        result["stage1_checks"].append(_check("serial_open", passed=False, detail=detail))
        result["stop_reason"] = detail
        result["real_hardware_used"] = False
        result["motion_stage"]["skipped_reason"] = "no serial device detected"
        return result

    result["real_hardware_used"] = is_real_serial_device(chosen_device)

    try:
        serial = PosixSerial(device=chosen_device, baudrate=baudrate, timeout_s=timeout_s)
        serial.open()
    except Exception as exc:
        result["real_hardware_used"] = False
        result["stop_reason"] = f"serial open failed: {exc}"
        result["stage1_checks"].append(_check("serial_open", passed=False, detail=str(exc)))
        result["motion_stage"]["skipped_reason"] = "serial open failed"
        return result

    bridge = ControllerBridge(serial)
    motion_ran = False
    try:
        checks, stop_reason = run_stage1(bridge, camera_index, settle_s, timeout_s)
        result["stage1_checks"] = checks
        if stop_reason:
            result["stop_reason"] = stop_reason

        stage1_all_pass = all(c["pass"] for c in checks)
        cam_info_check = next((c for c in checks if c["name"] == "camera_frame"), None)
        if cam_info_check:
            result["camera_info"] = cam_info_check.get("raw") or {}

        gate_ok = stage1_all_pass and result["real_hardware_used"] and allow_motion

        if gate_ok:
            motion = run_stage2_motion(bridge, pulse_s, target_revolutions, max_motor_seconds, camera_index)
            result["motion_stage"] = motion
            motion_ran = motion.get("ran", False)
            if motion.get("error") and not result["stop_reason"]:
                result["stop_reason"] = motion["error"]
        else:
            skip_reason = []
            if not stage1_all_pass:
                skip_reason.append("stage1 not fully passed")
            if not result["real_hardware_used"]:
                skip_reason.append("serial path is not real USB serial hardware")
            if not allow_motion:
                skip_reason.append("--allow-motion=False")
            result["motion_stage"]["skipped_reason"] = "; ".join(skip_reason) or "gate blocked"

        final_state, final_ok = finalize_stop(bridge)
        result["final_stop_state"] = final_state
        result["final_stop_ok"] = final_ok

    except Exception as exc:
        result["error"] = str(exc)
        result["stop_reason"] = result["stop_reason"] or f"unexpected exception: {exc}"
        try:
            final_state, final_ok = finalize_stop(bridge)
            result["final_stop_state"] = final_state
            result["final_stop_ok"] = final_ok
        except Exception as stop_exc:
            result["final_stop_state"] = f"exception:{stop_exc}"
            result["final_stop_ok"] = False
            result["error"] = (result["error"] or "") + f"; finalize_stop failed: {stop_exc}"
    finally:
        try:
            serial.close()
        except Exception as close_exc:
            result["error"] = (result["error"] or "") + f"; serial.close failed: {close_exc}"

    wheels_moved = result["motion_stage"].get("wheels_moved", False)
    result["wheels_moved"] = wheels_moved

    stage1_all_pass = all(c["pass"] for c in result["stage1_checks"])
    motion_error = result["motion_stage"].get("error")
    # Motion is REQUIRED if real hardware present AND stage1 fully passed.
    # allow_motion=False on real hardware cannot yield overall PASS — that would certify
    # a run that never exercised the wheels as a successful hardware validation.
    motion_required = stage1_all_pass and result["real_hardware_used"]
    motion_pass = (not motion_error) and (
        (not motion_required)  # motion not needed (stage1 failed or not real hw)
        or (motion_ran and wheels_moved and result["final_stop_ok"])  # ran and succeeded
    )
    result["pass"] = (
        stage1_all_pass
        and result["real_hardware_used"]
        and motion_pass
        and not result["error"]
        and result["final_stop_ok"]
    )

    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Staged live-hardware wheel-takeover validator")
    parser.add_argument("--serial-device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--pulse-s", type=float, default=0.05)
    parser.add_argument("--target-revolutions", type=float, default=0.03)
    parser.add_argument("--max-motor-seconds", type=float, default=2.0)
    parser.add_argument("--allow-motion", action="store_true", default=True)
    parser.add_argument("--no-motion", dest="allow_motion", action="store_false")
    parser.add_argument("--mock-serial", action="store_true", default=False)
    parser.add_argument("--report", type=Path, default=Path("outputs/pi_live_hardware_validation.json"))
    parser.add_argument("--report-md", type=Path, default=Path("docs/pi_live_hardware_validation_report.md"))
    args = parser.parse_args(argv)

    # Enforce hard safety caps regardless of CLI args
    pulse_s = min(args.pulse_s, MAX_MOTOR_SECONDS_HARD_CAP)
    target_rev = min(args.target_revolutions, MAX_TARGET_REVOLUTIONS)
    max_motor_s = min(args.max_motor_seconds, MAX_MOTOR_SECONDS_HARD_CAP)

    result = run_validation(
        serial_device=args.serial_device,
        baudrate=args.baudrate,
        camera_index=args.camera_index,
        timeout_s=args.timeout_s,
        settle_s=args.settle_s,
        pulse_s=pulse_s,
        target_revolutions=target_rev,
        max_motor_seconds=max_motor_s,
        allow_motion=args.allow_motion,
        mock_serial=args.mock_serial,
    )

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    write_report_md(result, args.report_md)

    print(json.dumps(result, indent=2))
    verdict = "PASS" if result["pass"] else "FAIL"
    print(f"\nLIVE HARDWARE VALIDATION: {verdict}")
    if not result.get("real_hardware_used"):
        print("WARNING: HARDWARE NOT EXERCISED - no serial MCU detected or mock mode")

    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
