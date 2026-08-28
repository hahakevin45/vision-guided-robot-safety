import pytest

from vgr_runtime.cli.pi_nav2_bench import (
    BenchSample,
    evaluate_motion,
    evaluate_stationary,
    evaluate_turn,
    require_raised_confirmation,
)


def sample(t, x, left, right, lt, rt, fault=None, yaw=0.0):
    return BenchSample(
        stamp_s=t,
        x_m=x,
        yaw_rad=yaw,
        raw_left=left,
        raw_right=right,
        left_target_cps=lt,
        right_target_cps=rt,
        fault=fault,
    )


def test_stationary_gate_requires_rate_low_drift_and_zero_targets():
    samples = [
        sample(t=i / 20.0, x=0.0, left=100, right=200, lt=0, rt=0)
        for i in range(201)
    ]

    report = evaluate_stationary(samples, duration_s=10.0)

    assert report["pass"] is True
    assert report["odom_hz"] >= 19.0
    assert report["left_drift_counts"] == 0
    assert report["right_drift_counts"] == 0


def test_stationary_gate_rejects_encoder_drift():
    samples = [
        sample(t=0.0, x=0.0, left=100, right=200, lt=0, rt=0),
        sample(t=10.0, x=0.0, left=103, right=198, lt=0, rt=0),
    ]

    report = evaluate_stationary(samples, duration_s=10.0)

    assert report["pass"] is False
    assert "stationary encoder drift above 2 counts" in report["reasons"]


def test_stationary_gate_rejects_fault_and_nonzero_target():
    samples = [
        sample(t=0.0, x=0.0, left=0, right=0, lt=1, rt=0, fault="timeout"),
        sample(t=10.0, x=0.0, left=0, right=0, lt=0, rt=0),
    ]

    report = evaluate_stationary(samples, duration_s=10.0)

    assert report["pass"] is False
    assert "nonzero target during stationary gate" in report["reasons"]
    assert "hardware fault reported" in report["reasons"]


def test_motion_gate_requires_forward_odom_and_final_stop():
    samples = [
        sample(t=0.0, x=0.0, left=0, right=0, lt=0, rt=0),
        sample(t=0.5, x=0.01, left=40, right=40, lt=80, rt=80),
        sample(t=1.0, x=0.012, left=45, right=45, lt=0, rt=0),
        sample(t=2.5, x=0.012, left=45, right=45, lt=0, rt=0),
    ]

    report = evaluate_motion(samples)

    assert report["pass"] is True
    assert report["delta_x_m"] > 0.0
    assert report["final_targets"] == [0, 0]
    assert report["normalized_encoder_delta"] == [45, 45]


def test_turn_gate_requires_positive_yaw_differential_counts_and_final_stop():
    samples = [
        sample(t=0.0, x=0.0, yaw=0.0, left=100, right=100, lt=0, rt=0),
        sample(t=0.5, x=0.0, yaw=0.04, left=90, right=110, lt=-60, rt=60),
        sample(t=1.0, x=0.0, yaw=0.05, left=88, right=112, lt=0, rt=0),
        sample(t=2.6, x=0.0, yaw=0.05, left=88, right=112, lt=0, rt=0),
    ]

    report = evaluate_turn(samples)

    assert report["pass"] is True
    assert report["delta_yaw_rad"] > 0.0
    assert report["differential_encoder_delta"] == [-12, 12]
    assert report["final_targets"] == [0, 0]


@pytest.mark.parametrize(
    ("last", "reason"),
    [
        (sample(2.6, 0.0, 112, 88, 0, 0, yaw=-0.05), "yaw did not increase"),
        (
            sample(2.6, 0.0, 112, 112, 0, 0, yaw=0.05),
            "differential encoder direction is not left turn",
        ),
    ],
)
def test_turn_gate_rejects_wrong_yaw_or_encoder_direction(last, reason):
    first = sample(0.0, 0.0, 100, 100, -60, 60, yaw=0.0)

    report = evaluate_turn([first, last])

    assert report["pass"] is False
    assert reason in report["reasons"]


@pytest.mark.parametrize(
    ("samples", "command_s", "reason"),
    [
        (
            [
                sample(0.0, 0.0, 0, 0, 0, 0),
                sample(2.5, -0.01, -10, 10, 0, 0),
            ],
            0.5,
            "odometry x did not increase",
        ),
        (
            [
                sample(0.0, 0.0, 0, 0, 0, 0),
                sample(2.5, 0.01, -10, 10, 0, 0),
            ],
            0.5,
            "normalized encoder direction is not forward",
        ),
        (
            [
                sample(0.0, 0.0, 0, 0, 0, 0),
                sample(2.5, 0.01, 10, 10, 1, 1),
            ],
            0.5,
            "final targets are nonzero",
        ),
        (
            [
                sample(0.0, 0.0, 0, 0, 0, 0),
                sample(2.5, 0.01, 10, 10, 0, 0),
            ],
            0.51,
            "command duration above 0.5 s",
        ),
        (
            [
                sample(0.0, 0.0, 0, 0, 0, 0),
                sample(2.5, 0.01, 10, 10, 0, 0, fault="serial"),
            ],
            0.5,
            "hardware fault reported",
        ),
        (
            [
                sample(0.0, 0.0, 0, 0, 0, 0),
                sample(0.5, 0.01, 10, 10, 80, 80),
                sample(1.0, 0.012, 12, 12, 0, 0),
            ],
            0.5,
            "final zero-target observation shorter than 2 s",
        ),
    ],
)
def test_motion_gate_rejects_unsafe_evidence(samples, command_s, reason):
    report = evaluate_motion(samples, command_s=command_s)

    assert report["pass"] is False
    assert reason in report["reasons"]


def test_motion_mode_requires_exact_raised_confirmation():
    require_raised_confirmation("YES")

    with pytest.raises(ValueError, match="VGR_WHEELS_RAISED=YES"):
        require_raised_confirmation("yes")
