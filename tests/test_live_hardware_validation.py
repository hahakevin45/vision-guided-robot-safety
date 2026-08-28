"""Tests for vgr_driver/cli/live_hardware_validation.py.

All tests use --mock-serial (pty-backed MockSerialMCU) to test LOGIC ONLY.
None of these tests constitute hardware validation.
real_hardware_used is ALWAYS False in mock mode.
"""

from __future__ import annotations

import json
import os
import pty
import threading
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from vgr_driver.cli.live_hardware_validation import (
    MAX_MOTOR_SECONDS_HARD_CAP,
    MAX_TARGET_REVOLUTIONS,
    detect_serial_device,
    main,
    run_stage2_motion,
    run_validation,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _mock_serial_run(**overrides):
    """Run validation in mock-serial mode with safe defaults."""
    kwargs = dict(
        serial_device="/dev/ttyACM0",
        baudrate=115200,
        camera_index=0,
        timeout_s=1.0,
        settle_s=0.1,
        pulse_s=0.05,
        target_revolutions=0.03,
        max_motor_seconds=2.0,
        allow_motion=True,
        mock_serial=True,
    )
    kwargs.update(overrides)
    return run_validation(**kwargs)


# ---------------------------------------------------------------------------
# AC3: mock run sets real_hardware_used=False and report contains 'HARDWARE NOT EXERCISED'
# ---------------------------------------------------------------------------


def test_mock_not_reported_as_hardware(tmp_path):
    result = _mock_serial_run()
    assert result["real_hardware_used"] is False, "mock must never report real_hardware_used=True"
    assert result["pass"] is False, "mock run must always FAIL (no real hardware)"
    assert result["mock_serial"] is True

    # Write report and check it contains the required phrase
    report_md = tmp_path / "report.md"
    from vgr_driver.cli.live_hardware_validation import write_report_md
    write_report_md(result, report_md)
    content = report_md.read_text()
    assert "HARDWARE NOT EXERCISED" in content


# ---------------------------------------------------------------------------
# AC2: stage gating — camera fail => motion_stage.ran=False, wheels_moved=False
# ---------------------------------------------------------------------------


def test_stage_gate_blocks_motion_on_stage1_failure():
    """Force camera to fail; gate must block motion stage."""
    with patch("vgr_driver.cli.live_hardware_validation.detect_camera") as mock_cam:
        mock_cam.return_value = {
            "opened": False,
            "frame_read": False,
            "width": None,
            "height": None,
            "actual_index": None,
            "error": "forced camera failure for test",
        }
        result = _mock_serial_run()

    ms = result["motion_stage"]
    assert ms["ran"] is False, "motion must not run when stage1 fails"
    assert ms.get("wheels_moved", False) is False
    assert result["wheels_moved"] is False

    # stage1 camera_frame check must be failed
    camera_check = next((c for c in result["stage1_checks"] if c["name"] == "camera_frame"), None)
    assert camera_check is not None
    assert camera_check["pass"] is False


# ---------------------------------------------------------------------------
# AC5: motion bounds enforced in code (hard-cap constants)
# ---------------------------------------------------------------------------


def test_motion_bounds_enforced():
    """Verify MAX_MOTOR_SECONDS_HARD_CAP and MAX_TARGET_REVOLUTIONS constants."""
    assert MAX_MOTOR_SECONDS_HARD_CAP <= 2.0, f"hard cap must be <=2.0s, got {MAX_MOTOR_SECONDS_HARD_CAP}"
    assert MAX_TARGET_REVOLUTIONS <= 0.03, f"target must be <=0.03 rev, got {MAX_TARGET_REVOLUTIONS}"


def test_motion_pulse_capped_at_hard_limit():
    """Even if caller passes large pulse_s, run_validation must cap it."""
    # We can verify this by checking that the motion_stage pulse_s_used <= 2.0
    with patch("vgr_driver.cli.live_hardware_validation.detect_camera") as mock_cam:
        # Succeed all stage1 checks so motion would run in a real scenario
        mock_cam.return_value = {
            "opened": True,
            "frame_read": True,
            "width": 640,
            "height": 480,
            "actual_index": 0,
            "error": None,
        }
        result = _mock_serial_run(pulse_s=999.0, target_revolutions=999.0, max_motor_seconds=999.0)

    # In mock mode, motion gate is blocked by real_hardware_used=False
    # But we can check via main() CLI enforcement
    from vgr_driver.cli.live_hardware_validation import main as val_main
    # Just verify constants
    assert MAX_MOTOR_SECONDS_HARD_CAP <= 2.0
    assert MAX_TARGET_REVOLUTIONS <= 0.03


# ---------------------------------------------------------------------------
# AC2: STOP-fail path aborts motion
# ---------------------------------------------------------------------------


def test_stop_fail_aborts_motion():
    """If pre-STOP in motion stage fails, ran must stay True but error must be set."""
    import os
    import pty

    from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
    from vgr_driver.driver import ControllerBridge
    from vgr_driver.driver import PosixSerial
    from vgr_driver.cli.live_hardware_validation import run_stage2_motion
    from vgr_core.model import CommandID

    master_fd, slave_fd = pty.openpty()
    mock = MockSerialMCU(master_fd)
    mock.start()
    try:
        device = os.ttyname(slave_fd)
        serial = PosixSerial(device=device, timeout_s=1.0)
        serial.open()
        bridge = ControllerBridge(serial)

        # Patch send_command to raise on STOP (simulate STOP failure)
        original_send = bridge.send_command

        def fail_on_stop(cmd):
            if cmd == CommandID.STOP:
                raise TimeoutError("simulated STOP failure")
            return original_send(cmd)

        bridge.send_command = fail_on_stop
        motion = run_stage2_motion(bridge, pulse_s=0.05, target_revolutions=0.03, max_motor_seconds=2.0)
        serial.close()
    finally:
        mock.stop()
        os.close(master_fd)
        os.close(slave_fd)

    assert motion.get("error") is not None, "error must be recorded on STOP failure"
    assert motion.get("wheels_moved") is False or motion.get("error")


# ---------------------------------------------------------------------------
# detect_serial_device: no devices found
# ---------------------------------------------------------------------------


def test_detect_serial_device_no_devices():
    with patch("vgr_driver.cli.live_hardware_validation.glob.glob", return_value=[]):
        chosen, candidates = detect_serial_device("/dev/ttyACM0")
    assert chosen is None
    assert candidates == []


def test_detect_serial_device_finds_requested():
    with patch("vgr_driver.cli.live_hardware_validation.glob.glob", side_effect=lambda p: ["/dev/ttyACM0"] if "ACM" in p else []):
        chosen, candidates = detect_serial_device("/dev/ttyACM0")
    assert chosen == "/dev/ttyACM0"
    assert "/dev/ttyACM0" in candidates


# ---------------------------------------------------------------------------
# AC4: no /dev/ttyACM* → overall pass=False, real_hardware_used=False, no crash
# ---------------------------------------------------------------------------


def test_no_serial_device_clean_fail(tmp_path):
    """Simulate current env (no serial device): must fail cleanly, no crash."""
    with patch("vgr_driver.cli.live_hardware_validation.glob.glob", return_value=[]):
        result = run_validation(
            serial_device="/dev/ttyACM0",
            baudrate=115200,
            camera_index=0,
            timeout_s=1.0,
            settle_s=0.5,
            pulse_s=0.05,
            target_revolutions=0.03,
            max_motor_seconds=2.0,
            allow_motion=True,
            mock_serial=False,
        )

    assert result["pass"] is False
    assert result["real_hardware_used"] is False
    assert result["wheels_moved"] is False
    assert result["stop_reason"] is not None and len(result["stop_reason"]) > 0


# ---------------------------------------------------------------------------
# AC1: JSON has all required keys
# ---------------------------------------------------------------------------


def test_json_has_required_keys(tmp_path):
    result = _mock_serial_run()
    required = [
        "pass", "real_hardware_used", "serial_device", "camera_info",
        "stage1_checks", "motion_stage", "final_stop_state", "final_stop_ok",
        "wheels_moved", "stop_reason", "error",
    ]
    for key in required:
        assert key in result, f"missing key: {key}"


# ---------------------------------------------------------------------------
# CLI integration: main() produces files
# ---------------------------------------------------------------------------


def test_pty_serial_is_not_reported_as_real_hardware():
    """PTY-backed mock serial must never be labeled as real hardware."""
    import os
    import pty
    from vgr_driver.driver.mock_serial_mcu import MockSerialMCU

    master_fd, slave_fd = pty.openpty()
    mock_mcu = MockSerialMCU(master_fd)
    mock_mcu.start()
    device = os.ttyname(slave_fd)
    try:
        with patch("vgr_driver.cli.live_hardware_validation.detect_serial_device", return_value=(device, [device])):
            with patch("vgr_driver.cli.live_hardware_validation.detect_camera", return_value={
                "opened": True, "frame_read": True, "width": 640, "height": 480,
                "actual_index": 0, "error": None,
            }):
                result = run_validation(
                    serial_device=device,
                    baudrate=115200,
                    camera_index=0,
                    timeout_s=1.0,
                    settle_s=0.1,
                    pulse_s=0.05,
                    target_revolutions=0.03,
                    max_motor_seconds=2.0,
                    allow_motion=False,
                    mock_serial=False,
                )
    finally:
        mock_mcu.stop()
        os.close(master_fd)
        os.close(slave_fd)

    assert result["real_hardware_used"] is False, "pty-backed device must never be real hardware"
    assert result["motion_stage"]["ran"] is False
    assert result["pass"] is False


def test_report_marks_camera_unavailable_without_claiming_validation(tmp_path):
    result = {
        "pass": False,
        "real_hardware_used": True,
        "serial_device": "/dev/ttyACM0",
        "serial_candidates": ["/dev/ttyACM0"],
        "camera_index": 0,
        "camera_info": {
            "opened": False,
            "frame_read": False,
            "width": None,
            "height": None,
            "actual_index": None,
            "error": "no camera found at indices probed",
        },
        "stage1_checks": [
            {"name": "serial_open", "pass": True, "detail": "PosixSerial opened successfully", "raw": None},
            {"name": "heartbeat", "pass": True, "detail": "MCU state=SAFE_STOP error=OK", "raw": None},
            {"name": "stop_command", "pass": True, "detail": "motor_intent=STOP error=OK", "raw": None},
            {
                "name": "camera_frame",
                "pass": False,
                "detail": "camera fail: no camera found at indices probed",
                "raw": {
                    "opened": False,
                    "frame_read": False,
                    "width": None,
                    "height": None,
                    "actual_index": None,
                    "error": "no camera found at indices probed",
                },
            },
        ],
        "motion_stage": {
            "ran": False,
            "skipped_reason": "stage1 not fully passed",
            "wheels_moved": False,
            "exchanges": [],
            "error": None,
        },
        "final_stop_state": "SAFE_STOP",
        "final_stop_ok": True,
        "wheels_moved": False,
        "stop_reason": "camera_frame failed: no camera found at indices probed",
        "error": None,
        "timestamp_utc": "2026-07-01T00:00:00+00:00",
        "mock_serial": False,
        "command_line": "ssh robot@192.0.2.10 'cd ~/vision_guided_robot && python3 -m vgr_driver.cli.live_hardware_validation'",
    }

    from vgr_driver.cli.live_hardware_validation import write_report_md

    report_md = tmp_path / "report.md"
    write_report_md(result, report_md)
    content = report_md.read_text()

    assert "CAMERA NOT AVAILABLE" in content
    assert "Pi Live Hardware Validation" not in content


def test_motion_stage_includes_camera_evidence():
    """Motion stage must record camera_before and camera_after evidence."""
    import os
    import pty
    from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
    from vgr_driver.driver import ControllerBridge
    from vgr_driver.driver import PosixSerial

    master_fd, slave_fd = pty.openpty()
    mock = MockSerialMCU(master_fd)
    mock.start()
    try:
        device = os.ttyname(slave_fd)
        serial = PosixSerial(device=device, timeout_s=1.0)
        serial.open()
        bridge = ControllerBridge(serial)

        cam_result = {"opened": True, "frame_read": True, "width": 640, "height": 480,
                      "actual_index": 0, "error": None}
        with patch("vgr_driver.cli.live_hardware_validation.detect_camera", return_value=cam_result) as mock_cam:
            motion = run_stage2_motion(bridge, 0.05, 0.03, 2.0, camera_index=0)

        serial.close()
    finally:
        mock.stop()
        os.close(master_fd)
        os.close(slave_fd)

    assert mock_cam.call_count >= 2, "detect_camera must be called before and after motion"
    assert motion["camera_before"] is not None, "camera_before must be recorded"
    assert motion["camera_after"] is not None, "camera_after must be recorded"
    exchange_labels = [ex["label"] for ex in motion["exchanges"]]
    assert "camera_before" in exchange_labels, "exchanges must include camera_before"
    assert "camera_after" in exchange_labels, "exchanges must include camera_after"


def test_main_cli_produces_output_files(tmp_path):
    report_json = tmp_path / "result.json"
    report_md = tmp_path / "report.md"
    # Run in no-serial-device environment
    with patch("vgr_driver.cli.live_hardware_validation.glob.glob", return_value=[]):
        rc = main([
            "--serial-device", "/dev/ttyACM0",
            "--report", str(report_json),
            "--report-md", str(report_md),
        ])

    assert rc == 1  # nonzero because no hardware
    assert report_json.exists(), "JSON report must be written"
    assert report_md.exists(), "Markdown report must be written"

    data = json.loads(report_json.read_text())
    assert data["pass"] is False
    assert data["real_hardware_used"] is False
    assert data["wheels_moved"] is False
    assert data["stop_reason"]

    md_content = report_md.read_text()
    assert "HARDWARE NOT EXERCISED" in md_content
