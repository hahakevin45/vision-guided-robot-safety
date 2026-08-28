import json
import subprocess
import sys
from pathlib import Path

from vgr_driver.cli.profile_camera_resolution import evaluate_profile_result


ROOT = Path(__file__).resolve().parents[1]


def test_evaluate_profile_result_requires_requested_resolution_and_fps():
    result = evaluate_profile_result(
        {
            "opened": True,
            "resolution_requested": [1280, 720],
            "resolution_actual": [1280, 720],
            "frames_requested": 120,
            "frames_read": 120,
            "fps_effective": 28.5,
            "std_min": 12.0,
            "error": None,
        },
        min_fps=20.0,
    )

    assert result is True


def test_evaluate_profile_result_rejects_wrong_resolution():
    result = evaluate_profile_result(
        {
            "opened": True,
            "resolution_requested": [1920, 1080],
            "resolution_actual": [640, 480],
            "frames_requested": 120,
            "frames_read": 120,
            "fps_effective": 28.5,
            "std_min": 12.0,
            "error": None,
        },
        min_fps=20.0,
    )

    assert result is False


def test_camera_resolution_profile_dry_run_writes_report(tmp_path):
    report = tmp_path / "profile.json"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.profile_camera_resolution",
            "--dry-run",
            "--report",
            str(report),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    payload = json.loads(report.read_text(encoding="utf-8"))
    assert completed.returncode == 0
    assert payload["pass"] is True
    assert [item["resolution_requested"] for item in payload["results"]] == [
        [640, 480],
        [1280, 720],
        [1920, 1080],
    ]
