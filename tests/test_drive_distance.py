import json
import math
import subprocess
import sys

from vgr_driver.cli.drive_distance import compute_distance_targets


def test_compute_distance_targets_1m():
    r = compute_distance_targets(1.0, 6.5, 750, 749)
    assert abs(r["circumference_cm"] - math.pi * 6.5) < 0.01
    assert 4.85 < r["revolutions"] < 4.95
    assert 3660 <= r["left_target_counts"] <= 3685
    assert 3655 <= r["right_target_counts"] <= 3680


def test_compute_distance_targets_0_2m():
    r = compute_distance_targets(0.20, 6.5, 750, 749)
    assert 0.95 < r["revolutions"] < 1.0


def test_compute_distance_targets_0_5m():
    r = compute_distance_targets(0.50, 6.5, 750, 749)
    assert 2.40 < r["revolutions"] < 2.50


def test_dry_run_cli(tmp_path):
    report = tmp_path / "dry_run.json"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.drive_distance",
            "--meters",
            "0.20",
            "--dry-run",
            "--report",
            str(report),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    data = json.loads(report.read_text(encoding="utf-8"))
    assert data["pass"] is True
    assert data["dry_run"] is True
    assert data["motor_commands_sent"] == 0
    assert 0.95 < data["revolutions"] < 1.0
    forward_steps = [s for s in data["steps"] if s.get("motor_intent") == "FORWARD"]
    assert forward_steps == [], f"dry-run sent FORWARD: {forward_steps}"
    # MCU error and sequence checks must be present and pass in happy path
    assert data["checks"]["no_mcu_error"] is True, "dry-run must check MCU error"
    assert data["checks"]["state_sequence_echo_ok"] is True, "dry-run must check state sequence"


def test_bench_mode_cli_pty_mock(tmp_path):
    report = tmp_path / "bench.json"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.drive_distance",
            "--meters",
            "0.05",
            "--max-seconds",
            "3",
            "--poll-interval-s",
            "0.01",
            "--post-stop-s",
            "0.01",
            "--report",
            str(report),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    data = json.loads(report.read_text(encoding="utf-8"))
    assert data["pass"] is True
    assert data["checks"]["ended_with_stop"] is True
    assert data["checks"]["no_mcu_error"] is True
    assert data["checks"]["state_sequence_echo_ok"] is True
    assert data["checks"]["encoder_sequence_echo_ok"] is True, "bench must check encoder sequence"
    cleanup_steps = [s for s in data["steps"] if s.get("label") == "cleanup_stop"]
    assert cleanup_steps, "no cleanup_stop step found"
    assert cleanup_steps[-1]["motor_intent"] == "STOP"
