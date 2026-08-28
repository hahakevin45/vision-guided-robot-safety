"""Nav2 `/plan` lookahead target selection for the SAPF goal contract."""
import math

import pytest

from vgr_core.safety.path_target import select_path_lookahead


def test_selects_point_at_lookahead_distance():
    # robot at x=0, path along +x at 0.1 m spacing; nearest point is (0.1, 0)
    # and 0.35 m of forward path accumulation crosses the (0.5, 0) point
    points = tuple((i * 0.1, 0.0) for i in range(1, 21))
    goal = select_path_lookahead(points, (0.0, 0.0), 0.35)
    assert goal == pytest.approx((0.5, 0.0))


def test_short_path_returns_final_point():
    points = ((0.1, 0.0), (0.2, 0.0), (0.3, 0.0))
    assert select_path_lookahead(points, (0.0, 0.0), 0.35) == pytest.approx((0.3, 0.0))


def test_nearest_point_starts_accumulation():
    # robot sits beside the middle of the path; lookahead walks forward from there
    points = ((0.0, 0.0), (0.2, 0.0), (0.4, 0.0), (0.6, 0.0), (0.8, 0.0))
    goal = select_path_lookahead(points, (0.2, 0.05), 0.35)
    assert goal == pytest.approx((0.6, 0.0))  # 0.2 -> 0.4 (0.2) -> 0.6 (0.4)


def test_does_not_walk_backward_before_nearest_point():
    # robot ahead of the path start must not pick a point behind itself
    points = ((0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.3, 0.0), (0.4, 0.0))
    goal = select_path_lookahead(points, (0.35, 0.0), 0.35)
    assert goal == pytest.approx((0.4, 0.0))  # final point, never x < 0.35


def test_empty_path_rejected():
    assert select_path_lookahead((), (0.0, 0.0), 0.35) is None


def test_non_finite_path_rejected():
    points = ((0.1, 0.0), (math.nan, 0.0), (0.3, 0.0))
    assert select_path_lookahead(points, (0.0, 0.0), 0.35) is None


def test_non_positive_lookahead_rejected():
    points = ((0.1, 0.0), (0.2, 0.0))
    assert select_path_lookahead(points, (0.0, 0.0), 0.0) is None
    assert select_path_lookahead(points, (0.0, 0.0), -1.0) is None


def test_single_point_returns_it():
    assert select_path_lookahead(((2.0, 0.0),), (0.0, 0.0), 0.35) == pytest.approx((2.0, 0.0))
