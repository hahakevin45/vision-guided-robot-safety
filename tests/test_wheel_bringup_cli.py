import json
import subprocess
import sys


def test_wheel_bringup_cli_runs_against_mock_serial(tmp_path):
    report = tmp_path / "wheel_bringup.json"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.bringup_wheels",
            "--spin-s",
            "0.1",
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
    assert data["using_pty_mock_mcu"] is True
    assert data["checks"]["left_encoder_changed"] is True
    assert data["checks"]["right_encoder_changed"] is True
    assert data["checks"]["ended_with_stop"] is True
    assert data["left"]["delta_left"] > 0
    assert data["left"]["delta_right"] == 0
    assert data["right"]["delta_right"] > 0
    assert data["right"]["delta_left"] == 0
    assert data["steps"][-1]["label"] == "cleanup_stop"
    assert data["steps"][-1]["motor_intent"] == "STOP"
