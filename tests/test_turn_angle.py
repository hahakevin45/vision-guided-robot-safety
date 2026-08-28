import json
import math
import subprocess
import sys

from vgr_driver.cli.turn_angle import compute_turn_targets


def test_compute_turn_targets_45_right():
    r = compute_turn_targets(45.0, 0.165, 6.5, 750, 749)
    # 弧長 = 0.0825 * pi/4 ≈ 0.0648 m
    assert abs(r["arc_m"] - 0.0825 * math.pi / 4) < 1e-9
    assert 235 <= r["left_target_counts"] <= 241
    assert 235 <= r["right_target_counts"] <= 241
    assert r["left_sign"] == 1 and r["right_sign"] == -1


def test_compute_turn_targets_45_left_mirrors_right():
    left = compute_turn_targets(-45.0, 0.165, 6.5, 750, 749)
    right = compute_turn_targets(45.0, 0.165, 6.5, 750, 749)
    assert left["left_target_counts"] == right["left_target_counts"]
    assert left["right_target_counts"] == right["right_target_counts"]
    assert left["left_sign"] == -1 and left["right_sign"] == 1


def test_compute_turn_targets_90_doubles_45():
    r45 = compute_turn_targets(45.0, 0.165, 6.5, 750, 749)
    r90 = compute_turn_targets(90.0, 0.165, 6.5, 750, 749)
    assert abs(r90["arc_m"] - 2 * r45["arc_m"]) < 1e-9


def test_dry_run_cli(tmp_path):
    report = tmp_path / "dry_run.json"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.turn_angle",
            "--degrees",
            "45",
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
    assert data["motor_commands_sent"] == 0
    assert data["checks"]["no_mcu_error"] is True
    assert data["checks"]["state_sequence_echo_ok"] is True


def test_bench_mode_cli_pty_mock_right_turn(tmp_path):
    report = tmp_path / "bench.json"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.turn_angle",
            "--degrees",
            "45",
            "--max-seconds",
            "5",
            "--poll-interval-s",
            "0.01",
            "--report",
            str(report),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    data = json.loads(report.read_text(encoding="utf-8"))
    assert data["pass"] is True
    assert data["checks"]["ended_with_stop"] is True
    assert data["checks"]["encoder_sequence_echo_ok"] is True
    # 右轉：左輪 counts 增加、右輪減少
    enc = [s for s in data["steps"] if s.get("kind") == "encoder"]
    d_left = enc[-1]["left_count"] - enc[0]["left_count"]
    d_right = enc[-1]["right_count"] - enc[0]["right_count"]
    assert d_left > 0, f"right turn should advance left wheel, got {d_left}"
    assert d_right < 0, f"right turn should reverse right wheel, got {d_right}"
    assert d_left >= data["left_target_counts"]
    assert abs(d_right) >= data["right_target_counts"]
