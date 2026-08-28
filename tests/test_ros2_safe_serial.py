import json
import subprocess
import sys
from pathlib import Path

from vgr_runtime.cli.certify_ros2_safe_serial import SAFE_COMMANDS, evaluate_safe_exchanges


ROOT = Path(__file__).resolve().parents[1]


def test_safe_command_list_excludes_motion_commands():
    assert [command.name for command in SAFE_COMMANDS] == ["HEARTBEAT", "STOP"]


def test_evaluate_safe_exchanges_rejects_motion_intent():
    result = evaluate_safe_exchanges(
        [
            {
                "command": "HEARTBEAT",
                "host_sequence": 0,
                "state_sequence": 0,
                "state": "SAFE_STOP",
                "error": "OK",
                "motor_intent": "STOP",
            },
            {
                "command": "STOP",
                "host_sequence": 1,
                "state_sequence": 1,
                "state": "TRACKING",
                "error": "OK",
                "motor_intent": "FORWARD",
            },
        ]
    )

    assert result["pass"] is False
    assert result["checks"]["all_motor_intents_stop"] is False


def test_ros2_safe_serial_cli_runs_against_mock_serial(tmp_path):
    report = tmp_path / "safe_serial.json"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_runtime.cli.certify_ros2_safe_serial",
            "--mock-serial",
            "--report",
            str(report),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    payload = json.loads(report.read_text(encoding="utf-8"))
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert payload["pass"] is True
    assert payload["using_pty_mock_mcu"] is True
    assert [item["command"] for item in payload["exchanges"]] == ["HEARTBEAT", "STOP"]
    assert {item["motor_intent"] for item in payload["exchanges"]} == {"STOP"}
