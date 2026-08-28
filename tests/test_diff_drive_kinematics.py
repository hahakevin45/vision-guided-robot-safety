import math

import pytest

from vgr_core.motion import DiffDriveParams, twist_to_wheel_counts

PARAMS = DiffDriveParams()  # wheel_base 0.165, diameter 0.065, cpr 750/749, max 900


def test_zero_twist_is_stop():
    assert twist_to_wheel_counts(0.0, 0.0, PARAMS) == (0, 0)


def test_straight_forward_wheels_equal_and_positive():
    left, right = twist_to_wheel_counts(0.1, 0.0, PARAMS)
    assert left > 0 and right > 0
    # 直行時兩輪目標應幾乎相等（僅左右 cpr 差 1 造成 <=1 counts 差）。
    assert abs(left - right) <= 1


def test_straight_reverse_wheels_negative():
    left, right = twist_to_wheel_counts(-0.1, 0.0, PARAMS)
    assert left < 0 and right < 0


def test_spin_in_place_opposite_and_balanced():
    # v=0, +ω 左轉：左輪後退、右輪前進，等量反向。
    left, right = twist_to_wheel_counts(0.0, 1.0, PARAMS)
    assert left < 0 < right
    assert abs(abs(left) - abs(right)) <= 1


def test_left_turn_right_wheel_faster():
    # +ω 左轉：右輪應比左輪快（右輪在外側）。
    left, right = twist_to_wheel_counts(0.2, 0.5, PARAMS)
    assert right > left


def test_expected_counts_straight_matches_formula():
    v = 0.15
    circ = math.pi * PARAMS.wheel_diameter_m
    expected_left = round(v / circ * PARAMS.left_counts_per_rev)
    expected_right = round(v / circ * PARAMS.right_counts_per_rev)
    left, right = twist_to_wheel_counts(v, 0.0, PARAMS)
    assert left == expected_left
    assert right == expected_right


def test_left_cpr_higher_shows_up_at_speed():
    # 直行、無夾制、速度夠大時，左輪(750)略高於右輪(749)。
    left, right = twist_to_wheel_counts(0.2, 0.0, PARAMS)
    assert left >= right


def test_clamp_caps_both_wheels_straight():
    # 遠超上限的直行 → 兩輪都夾到 ~max，且不超過。
    left, right = twist_to_wheel_counts(2.0, 0.0, PARAMS)
    assert abs(left) <= PARAMS.max_counts_per_s
    assert abs(right) <= PARAMS.max_counts_per_s
    assert max(abs(left), abs(right)) == PARAMS.max_counts_per_s


def test_clamp_preserves_turn_ratio():
    # 飽和時等比例縮放：左右比例應與未夾制前一致。
    v, w = 1.0, 2.0
    half = PARAMS.wheel_base_m / 2.0
    circ = math.pi * PARAMS.wheel_diameter_m
    raw_left = (v - w * half) / circ * PARAMS.left_counts_per_rev
    raw_right = (v + w * half) / circ * PARAMS.right_counts_per_rev
    left, right = twist_to_wheel_counts(v, w, PARAMS)
    assert max(abs(left), abs(right)) == PARAMS.max_counts_per_s
    # 比例保留（容許 rounding 誤差）。
    assert left / right == pytest.approx(raw_left / raw_right, rel=0.01)


def test_returns_ints():
    left, right = twist_to_wheel_counts(0.123, 0.456, PARAMS)
    assert isinstance(left, int) and isinstance(right, int)


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        DiffDriveParams(wheel_base_m=0.0)
    with pytest.raises(ValueError):
        DiffDriveParams(max_counts_per_s=0)
