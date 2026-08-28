from dataclasses import replace
from pathlib import Path

import pytest
import nav2_integration.acceptance as acceptance

from nav2_integration.acceptance import (
    TraceSummary,
    _detour_side,
    _footprint_clearance,
    _start_pose_error,
    summarize_localization_errors,
    evaluate,
)


PASSING_TRACE = TraceSummary(
    action_status="SUCCEEDED",
    final_position_error_m=0.08,
    final_yaw_error_rad=0.10,
    min_clearance_m=0.18,
    detour_side="north",
    nav_cmd_count=40,
    safe_cmd_count=40,
    plan_count=2,
)


def test_acceptance_requires_goal_clearance_detour_and_both_command_topics() -> None:
    report = evaluate(PASSING_TRACE)
    assert report["pass"] is True
    assert report["reasons"] == []


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("action_status", "ABORTED", "action status"),
        ("final_position_error_m", 0.13, "position error"),
        ("final_yaw_error_rad", 0.26, "yaw error"),
        ("min_clearance_m", 0.04, "clearance"),
        ("detour_side", None, "detour"),
        ("nav_cmd_count", 0, "/cmd_vel_nav"),
        ("safe_cmd_count", 0, "/cmd_vel_safe"),
        ("plan_count", 0, "/plan"),
    ],
)
def test_each_required_gate_can_fail(field: str, value: object, reason: str) -> None:
    report = evaluate(replace(PASSING_TRACE, **{field: value}))
    assert report["pass"] is False
    assert any(reason in item for item in report["reasons"])


def test_report_preserves_metrics_for_machine_audit() -> None:
    report = evaluate(PASSING_TRACE)
    assert report["metrics"]["final_position_error_m"] == 0.08
    assert report["metrics"]["detour_side"] == "north"
    assert report["thresholds"] == {
        "max_position_error_m": 0.12,
        "max_yaw_error_rad": 0.25,
        "min_clearance_m": 0.05,
    }


def test_footprint_clearance_uses_measured_rectangle_not_outer_circle() -> None:
    # At y=0.7 the 22 cm wide rectangular body has 19 cm to the north wall;
    # a 23 cm circumscribed circle would incorrectly report only 7 cm.
    assert _footprint_clearance(2.0, 0.7, 0.0) == pytest.approx(0.19)


def test_footprint_clearance_detects_obstacle_intersection() -> None:
    assert _footprint_clearance(2.0, 0.4, 0.0) == pytest.approx(0.0)


def test_rotated_footprint_clearance_is_finite_and_symmetric() -> None:
    north = _footprint_clearance(1.5, 0.65, 0.6)
    south = _footprint_clearance(1.5, -0.65, -0.6)
    assert north == pytest.approx(south)
    assert north > 0.0


def test_detour_side_classifies_topology_without_repeating_clearance_gate() -> None:
    assert _detour_side([(2.0, 0.40, 0.0)]) == "north"
    assert _detour_side([(2.0, -0.40, 0.0)]) == "south"


def test_long_running_goal_uses_zero_stamp_for_latest_tf() -> None:
    source = Path("nav2_integration/acceptance.py").read_text(encoding="utf-8")
    assert "goal.pose.header.stamp =" not in source


def test_localization_error_summary_preserves_count_mean_and_max() -> None:
    summary = summarize_localization_errors([(0.1, 0.2), (0.3, 0.4)])
    assert summary == {
        "count": 2,
        "mean_position_error_m": pytest.approx(0.2),
        "max_position_error_m": pytest.approx(0.3),
        "mean_yaw_error_rad": pytest.approx(0.3),
        "max_yaw_error_rad": pytest.approx(0.4),
    }


def test_start_pose_error_detects_gazebo_spawn_drift() -> None:
    assert _start_pose_error((0.50, 0.0, 0.0), 0.50, 0.0) == pytest.approx(0.0)
    assert _start_pose_error((0.02, 0.19, 0.0), 0.50, 0.0) > 0.5


def test_plan_summary_distinguishes_initial_crossing_from_replan() -> None:
    assert hasattr(acceptance, "_summarize_plan")
    straight = acceptance._summarize_plan(
        [(0.7, 0.0), (1.8, 0.0), (2.0, 0.0), (3.5, 0.0)])
    detour = acceptance._summarize_plan([
        (0.7, 0.0), (1.4, 0.0), (1.5, 0.3), (1.6, 0.6),
        (2.4, 0.6), (2.5, 0.3), (2.6, 0.0), (3.5, 0.0),
    ])
    assert straight["crosses_obstacle_envelope"] is True
    assert detour["crosses_obstacle_envelope"] is False
    assert detour["max_abs_y_m"] == pytest.approx(0.6)
