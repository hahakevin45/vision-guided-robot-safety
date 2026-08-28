import json
from pathlib import Path

import pytest

from vgr_driver.cli.pi_high_speed_bench import (
    SpeedSegmentEvidence,
    atomic_json,
    counts_to_distance_m,
    evaluate_speed_segment,
    should_continue_stop_collection,
)


def test_counts_to_distance_uses_measured_wheel_geometry():
    assert counts_to_distance_m(750, 750.0) == pytest.approx(
        3.141592653589793 * 0.065
    )


def test_stop_collection_waits_two_seconds_after_last_motion():
    assert should_continue_stop_collection(0.0, 0.28, 2.10) is True
    assert should_continue_stop_collection(0.0, 0.28, 2.29) is False


def test_stop_collection_has_four_second_absolute_deadline():
    assert should_continue_stop_collection(0.0, 3.9, 4.01) is False


def passing_segment(**changes):
    values = dict(
        target_cps=735,
        command_duration_s=5.0,
        left_delta_counts=3670,
        right_delta_counts=3665,
        left_mean_cps=730.0,
        right_mean_cps=728.0,
        mean_distance_m=0.998,
        wheel_distance_mismatch_ratio=0.002,
        stop_acknowledged=True,
        final_abs_left_cps=0.0,
        final_abs_right_cps=0.0,
        stopped_observation_s=2.1,
        max_mismatch_run_s=0.0,
        faults=(),
    )
    values.update(changes)
    return SpeedSegmentEvidence(**values)


def test_full_speed_segment_accepts_one_meter_closed_loop():
    report = evaluate_speed_segment(passing_segment())

    assert report["pass"] is True
    assert report["metrics"]["mean_distance_m"] == 0.998


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"command_duration_s": 4.99}, "duration"),
        ({"left_delta_counts": -1}, "direction"),
        ({"left_mean_cps": 620.0}, "left speed"),
        ({"right_mean_cps": 850.0}, "right speed"),
        ({"mean_distance_m": 0.849}, "distance"),
        ({"mean_distance_m": 1.151}, "distance"),
        ({"wheel_distance_mismatch_ratio": 0.101}, "mismatch"),
        ({"max_mismatch_run_s": 0.50}, "sustained"),
        ({"stop_acknowledged": False}, "STOP"),
        ({"final_abs_left_cps": 10.1}, "final wheel rate"),
        ({"stopped_observation_s": 1.99}, "observation"),
        ({"faults": ("serial",)}, "fault"),
    ],
)
def test_full_speed_segment_rejects_each_failed_condition(change, reason):
    report = evaluate_speed_segment(passing_segment(**change))

    assert report["pass"] is False
    assert any(reason in item for item in report["reasons"])


def test_preflight_uses_one_second_and_does_not_require_one_meter():
    evidence = passing_segment(
        target_cps=600,
        command_duration_s=1.0,
        left_delta_counts=590,
        right_delta_counts=585,
        left_mean_cps=590.0,
        right_mean_cps=585.0,
        mean_distance_m=0.16,
    )

    report = evaluate_speed_segment(evidence, preflight=True)

    assert report["pass"] is True


def test_preflight_requires_feedback_but_not_steady_state_accuracy():
    evidence = passing_segment(
        target_cps=600,
        command_duration_s=1.0,
        left_delta_counts=300,
        right_delta_counts=290,
        left_mean_cps=300.0,
        right_mean_cps=290.0,
        mean_distance_m=0.08,
    )

    report = evaluate_speed_segment(evidence, preflight=True)

    assert report["pass"] is True


def test_atomic_json_replaces_temporary_file(tmp_path):
    path = tmp_path / "nested" / "report.json"

    atomic_json(path, {"pass": True})

    assert json.loads(path.read_text(encoding="utf-8")) == {"pass": True}
    assert not path.with_suffix(".json.tmp").exists()


def test_runtime_is_fixed_two_stage_serial_only_and_fail_closed():
    source = Path("ros2_ws/src/vgr_driver/vgr_driver/cli/pi_high_speed_bench.py").read_text(encoding="utf-8")

    for required in (
        'require_raised_confirmation(args.wheels_raised)',
        'send_set_wheel_speed(600, 600)',
        'send_set_wheel_speed(735, 735)',
        'read_encoders()',
        'signal.SIGTERM',
        'finally:',
        'send_command(CommandID.STOP)',
        'command_duration_s=1.0',
        'command_duration_s=5.0',
        'time.sleep(0.05)',
    ):
        assert required in source
    for excluded in ("rclpy", "NavigateToPose", "/cmd_vel"):
        assert excluded not in source
    assert "ErrorCode.OK" in source
    assert "ErrorCode.NONE" not in source
