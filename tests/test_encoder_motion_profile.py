import json
import subprocess
import sys

from vgr_driver.cli.profile_encoder_motion import (
    EncoderSegment,
    build_encoder_motion_profile,
)


def test_encoder_motion_profile_infers_mapping_and_forward_signs():
    profile = build_encoder_motion_profile(
        direction_segments=[
            EncoderSegment(
                label="turn_left_right_wheel",
                command="TURN_LEFT",
                expected_physical_motion="right wheel",
                elapsed_s=0.15,
                left_delta=0,
                right_delta=2,
            ),
            EncoderSegment(
                label="turn_right_left_wheel",
                command="TURN_RIGHT",
                expected_physical_motion="left wheel",
                elapsed_s=0.15,
                left_delta=-4,
                right_delta=0,
            ),
            EncoderSegment(
                label="forward_both_wheels",
                command="FORWARD",
                expected_physical_motion="both wheels",
                elapsed_s=0.15,
                left_delta=-7,
                right_delta=5,
            ),
        ],
        speed_segments=[
            EncoderSegment(
                label="forward_speed_1.0s",
                command="FORWARD",
                expected_physical_motion="both wheels",
                elapsed_s=1.0,
                left_delta=-40,
                right_delta=30,
            ),
            EncoderSegment(
                label="forward_speed_3.0s",
                command="FORWARD",
                expected_physical_motion="both wheels",
                elapsed_s=3.0,
                left_delta=-150,
                right_delta=120,
            ),
        ],
    )

    assert profile["encoder_mapping"] == "ok"
    assert profile["odom_recommendation"]["left_encoder_sign"] == -1
    assert profile["odom_recommendation"]["right_encoder_sign"] == 1
    assert profile["speed_segments"][0]["left_normalized_counts_per_s"] == 40.0
    assert profile["speed_segments"][0]["right_normalized_counts_per_s"] == 30.0
    assert profile["speed_segments"][1]["left_normalized_counts_per_s"] == 50.0
    assert profile["speed_segments"][1]["right_normalized_counts_per_s"] == 40.0


def test_encoder_motion_profile_cli_runs_against_mock_serial(tmp_path):
    report = tmp_path / "encoder_motion_profile.json"

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "vgr_driver.cli.profile_encoder_motion",
            "--direction-pulse-s",
            "0.01",
            "--speed-pulse-s",
            "0.01",
            "--speed-pulse-s",
            "0.02",
            "--gap-s",
            "0.01",
            "--settle-s",
            "0.01",
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
    assert data["checks"]["ended_with_stop"] is True
    assert data["checks"]["any_speed_count_changed"] is True
    assert [segment["duration_s"] for segment in data["speed_segments"]] == [0.01, 0.02]
