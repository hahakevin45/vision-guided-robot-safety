import pytest
from pathlib import Path

from vgr_runtime.cli.pi_nav2_1m_bench import OneMeterEvidence, evaluate_one_meter


def passing_evidence(**changes):
    values = dict(
        goal_count=1,
        goal_accepted=True,
        action_status="SUCCEEDED",
        initial_pose=(0.0, 0.0, 0.0),
        final_pose=(0.98, 0.01, 0.01),
        raw_encoder_delta=(3600, 3590),
        nav_cmd_count=110,
        safe_cmd_count=110,
        plan_count=6,
        max_nav_linear_mps=0.20,
        max_nav_angular_rad_s=0.10,
        max_safe_linear_mps=0.20,
        max_safe_angular_rad_s=0.10,
        max_abs_target_cps=820,
        final_targets=(0, 0),
        zero_target_observation_s=2.1,
        hardware_faults=(),
        safe_publisher_count=1,
        clamp_count=0,
        stale_count=0,
        goal_elapsed_s=6.0,
    )
    values.update(changes)
    return OneMeterEvidence(**values)


def test_one_meter_evidence_passes_closed_loop_and_performance():
    report = evaluate_one_meter(passing_evidence())

    assert report["closed_loop_pass"] is True
    assert report["performance_target_met"] is True
    assert report["pass"] is True
    assert report["metrics"]["delta_x_m"] == pytest.approx(0.98)


def test_slow_success_preserves_closed_loop_but_fails_performance():
    report = evaluate_one_meter(passing_evidence(goal_elapsed_s=7.01))

    assert report["closed_loop_pass"] is True
    assert report["performance_target_met"] is False
    assert report["pass"] is False


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"goal_count": 2}, "one goal"),
        ({"goal_accepted": False}, "accepted"),
        ({"action_status": "ABORTED"}, "action status"),
        ({"goal_elapsed_s": 12.01}, "deadline"),
        ({"final_pose": (0.949, 0.0, 0.0)}, "odometry x"),
        ({"final_pose": (1.021, 0.0, 0.0)}, "odometry x"),
        ({"final_pose": (0.98, 0.081, 0.0)}, "lateral"),
        ({"final_pose": (0.98, 0.0, 0.251)}, "yaw"),
        ({"raw_encoder_delta": (0, 3590)}, "encoder"),
        ({"nav_cmd_count": 0}, "/cmd_vel_nav"),
        ({"safe_cmd_count": 0}, "/cmd_vel_safe"),
        ({"plan_count": 0}, "/plan"),
        ({"max_safe_linear_mps": 0.201}, "linear"),
        ({"max_safe_angular_rad_s": 0.251}, "angular"),
        ({"max_abs_target_cps": 901}, "target"),
        ({"final_targets": (1, 0)}, "final targets"),
        ({"zero_target_observation_s": 1.99}, "zero-target"),
        ({"hardware_faults": ("serial",)}, "fault"),
        ({"safe_publisher_count": 2}, "publisher"),
    ],
)
def test_one_meter_evidence_rejects_each_closed_loop_failure(change, reason):
    report = evaluate_one_meter(passing_evidence(**change))

    assert report["closed_loop_pass"] is False
    assert report["pass"] is False
    assert any(reason in item for item in report["reasons"])


def test_runtime_uses_fixed_one_meter_action_and_bounded_relay():
    source = Path("ros2_ws/src/vgr_runtime/vgr_runtime/cli/pi_nav2_1m_bench.py").read_text(encoding="utf-8")

    for required in (
        'ActionClient(node, NavigateToPose, "/navigate_to_pose")',
        'goal.pose.pose.position.x = 1.00',
        'CommandLimiter(0.20, 0.25, 0.20)',
        '"/cmd_vel_nav"',
        '"/cmd_vel_safe"',
        '"/odom"',
        '"/hardware/status"',
        '"/plan"',
        'cancel_goal_async',
        'get_publishers_info_by_topic',
    ):
        assert required in source
    for excluded in ("/aruco/pose", "amcl", "safety_gate", "camera"):
        assert excluded not in source.lower()
