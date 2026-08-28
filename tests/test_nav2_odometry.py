import math

import pytest

from vgr_core.motion import DifferentialOdometry, EncoderConfig


CFG = EncoderConfig(
    wheel_base_m=0.165,
    wheel_diameter_m=0.065,
    left_counts_per_rev=750.0,
    right_counts_per_rev=749.0,
    left_sign=-1,
    right_sign=1,
)


def test_equal_normalized_counts_integrate_straight() -> None:
    odom = DifferentialOdometry(CFG)
    odom.update(raw_left=0, raw_right=0, stamp_s=0.0)
    pose = odom.update(raw_left=-750, raw_right=749, stamp_s=2.0)
    assert pose.x == pytest.approx(math.pi * 0.065, rel=1e-6)
    assert pose.y == pytest.approx(0.0, abs=1e-9)
    assert pose.theta == pytest.approx(0.0, abs=1e-9)
    assert pose.linear_mps == pytest.approx(math.pi * 0.065 / 2.0)
    assert pose.angular_rad_s == pytest.approx(0.0, abs=1e-9)


def test_opposite_wheel_motion_integrates_rotation() -> None:
    equal_cpr = EncoderConfig(0.165, 0.065, 750.0, 750.0, -1, 1)
    odom = DifferentialOdometry(equal_cpr)
    odom.update(raw_left=0, raw_right=0, stamp_s=0.0)
    state = odom.update(raw_left=-375, raw_right=-375, stamp_s=1.0)
    expected = (math.pi * 0.065) / equal_cpr.wheel_base_m
    assert state.x == pytest.approx(0.0, abs=1e-9)
    assert state.y == pytest.approx(0.0, abs=1e-9)
    assert state.theta == pytest.approx(-expected)


def test_unequal_wheel_arcs_integrate_curve_at_midpoint_heading() -> None:
    odom = DifferentialOdometry(CFG)
    odom.update(raw_left=0, raw_right=0, stamp_s=0.0)
    state = odom.update(raw_left=-375, raw_right=749, stamp_s=1.0)
    left_arc = math.pi * 0.065 / 2.0
    right_arc = math.pi * 0.065
    distance = (left_arc + right_arc) / 2.0
    dtheta = (right_arc - left_arc) / CFG.wheel_base_m
    assert state.x == pytest.approx(distance * math.cos(dtheta / 2.0))
    assert state.y == pytest.approx(distance * math.sin(dtheta / 2.0))
    assert state.theta == pytest.approx(dtheta)


def test_first_sample_is_a_zero_velocity_baseline() -> None:
    state = DifferentialOdometry(CFG).update(123, -456, 10.0)
    assert state.x == 0.0
    assert state.linear_mps == 0.0
    assert state.angular_rad_s == 0.0


def test_non_increasing_timestamp_is_rejected() -> None:
    odom = DifferentialOdometry(CFG)
    odom.update(0, 0, 1.0)
    with pytest.raises(ValueError, match="strictly increasing"):
        odom.update(1, 1, 1.0)


def test_signed_32_bit_counter_rollover_is_one_count() -> None:
    config = EncoderConfig(0.165, 0.065, 100.0, 100.0, 1, 1)
    odom = DifferentialOdometry(config)
    odom.update(2**31 - 1, 2**31 - 1, 0.0)
    state = odom.update(-2**31, -2**31, 1.0)
    assert state.x == pytest.approx(math.pi * 0.065 / 100.0)
