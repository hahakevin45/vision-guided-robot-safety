import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_motor_safety_cli_runs_against_mock_serial(tmp_path):
    report = tmp_path / "motor_safety.json"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.certify_motor_safety",
            "--step-s",
            "0.01",
            "--timeout-observe-s",
            "0.01",
            "--report",
            str(report),
        ],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    )

    payload = json.loads(report.read_text(encoding="utf-8"))
    assert completed.returncode == 0
    assert payload["pass"] is True
    assert payload["manual_confirmation_required"] is True
    assert payload["checks"]["stop_commands_accepted"] is True
    assert payload["checks"]["ended_with_stop"] is True
    assert [step["command"] for step in payload["steps"]][-1] == "STOP"
