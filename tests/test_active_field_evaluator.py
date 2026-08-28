"""Tests for active ArUco field evaluator (Task 7)."""
import pytest

from gazebo_sim.evaluate_active_aruco_field import (
    compare_active_field_runs, evaluate_active_field,
    wall_clearance,
)

SQUARE = ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0))


def _true(t, x, y):
    return {"topic": "/sim/true_pose", "t": t,
            "true_pose": {"x": x, "y": y, "theta": 0.0}}


def _status(t, **debug):
    return {"topic": "/safety_gate/status", "t": t,
            "mode": "MODIFIED", "debug": debug}


def _ids(t, ids):
    return {"topic": "/aruco/marker_ids", "t": t, "stamp_s": t, "ids": ids}


def _cmd(t, v, omega=0.0):
    return {"topic": "/cmd_vel_safe", "t": t,
            "twist": {"v": v, "omega": omega}}


def _zero_commands(start_t):
    return [_cmd(start_t + 0.1 * index, 0.0) for index in range(11)]


def test_wall_clearance_includes_half_wall_and_robot_radius():
    assert wall_clearance(1.0, 0.30, SQUARE, wall_thickness=0.05,
                          robot_radius=0.23) == pytest.approx(0.045)


def test_status_alignment_never_uses_future_true_pose():
    rows = [
        _true(0.0, 1.0, 0.3),
        _status(1.0, dead_reckoning=1.0, estimated_x=1.1, estimated_y=0.3,
                pose_drift_m=0.2, d_safe_m=0.48, blind_dist_m=0.3),
        _true(2.0, 1.8, 0.3),
    ]
    manifest = _manifest("controlled_adaptive")
    manifest["runtime_failures"] = ["fixture ignores full run validity"]
    out = evaluate_active_field(rows, manifest)
    assert out["max_true_localization_error_m"] == pytest.approx(0.1)


def _pose(topic, t, x, y):
    return {"topic": topic, "t": t, "stamp_s": t,
            "pose": {"x": x, "y": y, "theta": 0.0}}


def _window(t, dropout, x):
    return {"topic": "/aruco/dropout_window", "t": t,
            "event": "dropout_start" if dropout else "dropout_end",
            "dropout": dropout, "applied_t_s": t,
            "pose": {"x": x, "y": 0.3, "theta": 0.0}}


def _manifest(arm):
    return {
        "arm": arm, "repeat": 1,
        "start_pose": {"x": 1.4, "y": 0.3, "yaw": 0.0},
        "goal": {"x": 0.5, "y": 0.3},
        "walls": [list(p) for p in SQUARE],
        "wall_thickness_m": 0.05, "robot_radius_m": 0.23,
        "timeout_sim_s": 90.0, "runtime_failures": [],
        "dropout": {"enabled": arm != "natural_adaptive",
                    "dropout_x": 1.25, "resume_x": 0.70},
    }


def _controlled_rows():
    return [
        _true(0.0, 1.4, 0.3), _ids(0.0, [5]),
        _pose("/aruco/pose_raw", 0.0, 1.4, 0.3),
        _pose("/aruco/pose", 0.0, 1.4, 0.3),
        _cmd(0.5, 0.1), _true(0.5, 1.35, 0.3),
        _window(1.0, True, 1.24), _true(1.0, 1.24, 0.28),
        _status(1.6, dead_reckoning=1.0, estimated_x=1.14,
                estimated_y=0.28, pose_drift_m=0.2, d_safe_m=0.48,
                blind_dist_m=0.3),
        _true(2.0, 0.69, 0.25), _window(2.0, False, 0.69),
        _ids(2.1, [0]), _pose("/aruco/pose_raw", 2.1, 0.66, 0.27),
        _pose("/aruco/pose", 2.1, 0.66, 0.27),
        _status(2.2, dead_reckoning=0.0, estimated_x=0.65,
                estimated_y=0.28, pose_drift_m=0.0, d_safe_m=0.28,
                blind_dist_m=0.0),
        _true(2.2, 0.65, 0.28), _true(3.0, 0.5, 0.3),
        *_zero_commands(3.0), _true(4.0, 0.5, 0.3),
    ]


def _natural_rows():
    return [
        _true(0.0, 1.4, 0.3), _ids(0.0, [5]),
        _pose("/aruco/pose_raw", 0.0, 1.4, 0.3),
        _pose("/aruco/pose", 0.0, 1.4, 0.3), _cmd(0.5, 0.1),
        _true(1.0, 1.1, 0.28), _ids(1.0, []),
        _status(1.5, dead_reckoning=1.0, estimated_x=0.95,
                estimated_y=0.28, pose_drift_m=0.2, d_safe_m=0.48,
                blind_dist_m=0.3),
        _ids(1.6, []), _true(1.7, 0.68, 0.27), _ids(1.7, [0]),
        _pose("/aruco/pose_raw", 1.7, 0.68, 0.27),
        _pose("/aruco/pose", 1.7, 0.68, 0.27),
        _status(1.8, dead_reckoning=0.0, estimated_x=0.67,
                estimated_y=0.28, pose_drift_m=0.0, d_safe_m=0.28,
                blind_dist_m=0.0),
        _true(2.5, 0.5, 0.3), *_zero_commands(2.5),
        _true(3.5, 0.5, 0.3),
    ]


def test_controlled_window_reacquisition_reset_and_goal_are_valid():
    out = evaluate_active_field(
        _controlled_rows(), _manifest("controlled_adaptive"))
    assert out["valid"] is True
    assert out["dropout_duration_s"] == pytest.approx(1.0)
    assert out["marker_0_reacquire_t_s"] == pytest.approx(2.1)
    assert out["recovered"] is True
    assert out["reached_goal"] is True
    assert out["minimum_envelope_excess_m"] == pytest.approx(0.1)


def test_natural_visibility_sequence_is_valid_without_gate_events():
    out = evaluate_active_field(_natural_rows(), _manifest("natural_adaptive"))
    assert out["valid"] is True
    assert out["dropout_duration_s"] >= 0.4
    assert out["marker_0_reacquire_t_s"] == pytest.approx(1.7)
    assert out["recovered"] is True


def test_natural_fixture_with_marker5_no_gap_then_marker0_is_invalid():
    # Marker 5 is accepted and marker 0 reappears later, but there is no
    # empty accepted-ID gap, so the natural dropout evidence is missing.
    rows = [
        _true(0.0, 1.4, 0.3), _ids(0.0, [5]),
        _cmd(0.5, 0.1), _true(1.0, 1.1, 0.28), _ids(1.0, [5]),
        _ids(1.7, [0]), _true(2.5, 0.5, 0.3), _cmd(2.5, 0.0),
    ]
    out = evaluate_active_field(rows, _manifest("natural_adaptive"))
    assert out["valid"] is False
    assert out["dropout_start_t_s"] is None
    assert out["marker_0_reacquire_t_s"] == pytest.approx(1.7)
    assert any("missing natural dropout gap" in reason
               for reason in out["invalid_reasons"])


def test_controlled_fixture_missing_window_end_is_invalid():
    # Dropout start fires but the resume/end transition never arrives.
    rows = [
        _true(0.0, 1.4, 0.3), _ids(0.0, [5]),
        _pose("/aruco/pose_raw", 0.0, 1.4, 0.3),
        _pose("/aruco/pose", 0.0, 1.4, 0.3),
        _cmd(0.5, 0.1), _true(0.5, 1.35, 0.3),
        _window(1.0, True, 1.24), _true(1.0, 1.24, 0.28),
        _true(3.0, 0.5, 0.3), _cmd(3.0, 0.0),
        _true(4.0, 0.5, 0.3), _cmd(4.0, 0.0),
    ]
    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))
    assert out["valid"] is False
    assert out["dropout_start_t_s"] == pytest.approx(1.0)
    assert out["dropout_end_t_s"] is None
    assert any("missing controlled dropout end transition" in reason
               for reason in out["invalid_reasons"])


def test_natural_gap_of_exactly_0_4s_is_invalid():
    # The gap must be strictly greater than 0.4 s; exactly 0.4 s is not
    # enough to count as genuine dropout evidence.
    rows = [
        _true(0.0, 1.4, 0.3), _ids(0.0, [5]),
        _cmd(0.5, 0.1), _true(1.0, 1.1, 0.28), _ids(1.0, []),
        _ids(1.4, [0]), _true(2.5, 0.5, 0.3), _cmd(2.5, 0.0),
    ]
    out = evaluate_active_field(rows, _manifest("natural_adaptive"))
    assert out["valid"] is False
    assert out["dropout_duration_s"] is None
    assert any("not strictly > 0.4s" in reason
               for reason in out["invalid_reasons"])


def test_goal_requires_one_second_of_zero_safe_command():
    rows = _controlled_rows()
    command = next(
        row for row in rows
        if row.get("topic") == "/cmd_vel_safe"
        and row["t"] == pytest.approx(4.0))
    command["twist"]["v"] = 0.1
    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))
    assert out["reached_goal"] is False
    assert out["time_to_goal_s"] is None


def test_nonfinite_required_debug_marks_run_invalid():
    rows = _controlled_rows()
    rows[8]["debug"]["estimated_x"] = float("nan")
    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))
    assert out["valid"] is False
    assert any("non-finite" in reason for reason in out["invalid_reasons"])


def test_startup_missing_pose_status_does_not_invalidate_run():
    # Pre-motion STOP with reason "missing_pose" legitimately lacks
    # blind/dead-reckoning fields (NaN d_safe), but remains chronologically
    # ordered in the trace fixture.
    rows = _controlled_rows()
    rows.insert(4, {
        "topic": "/safety_gate/status", "t": 0.3, "mode": "STOP",
        "debug": {
            "reason": "missing_pose", "dead_reckoning": 0,
            "estimated_x": float("nan"), "estimated_y": float("nan"),
            "pose_drift_m": float("nan"), "d_safe_m": float("nan"),
            "blind_dist_m": float("nan"),
        },
    })
    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))
    assert out["valid"] is True
    assert out["invalid_reasons"] == []


def _result(arm, repeat, clearance, *, reached=True):
    return {
        "arm": arm, "repeat": repeat, "valid": True,
        "min_true_wall_clearance_m": clearance,
        "max_southward_excursion_m": 0.1,
        "path_length_m": 1.0, "time_to_goal_s": 10.0 if reached else None,
        "max_true_localization_error_m": 0.05,
        "collision_envelope_violated": False, "recovered": True,
        "reached_goal": reached,
    }


def test_comparison_requires_three_positive_controlled_pairs_and_natural_runs():
    rows = []
    for repeat in (1, 2, 3):
        rows.append(_result("controlled_adaptive", repeat, 0.20 + repeat / 100))
        rows.append(_result("controlled_fixed_028", repeat, 0.10))
        rows.append(_result("natural_adaptive", repeat, 0.18))
    out = compare_active_field_runs(rows)
    assert len(out["controlled_pairs"]) == 3
    assert all(p["min_clearance_delta_m"] > 0 for p in out["controlled_pairs"])
    assert out["adaptive_clearance_claim"] is True
    assert out["natural_end_to_end_claim"] is True
    assert out["scenario_solved"] is True


def test_pair_time_delta_is_null_when_either_arm_fails_goal():
    rows = [
        _result("controlled_adaptive", 1, 0.2),
        _result("controlled_fixed_028", 1, 0.1, reached=False),
    ]
    out = compare_active_field_runs(rows)
    assert out["controlled_pairs"][0]["time_to_goal_delta_s"] is None


def test_comparison_works_for_repeats_actually_present():
    # One controlled pair + one natural run (no hard-coded 3-repeat gate).
    rows = [
        _result("controlled_adaptive", 1, 0.20),
        _result("controlled_fixed_028", 1, 0.10),
        _result("natural_adaptive", 1, 0.18),
    ]
    out = compare_active_field_runs(rows)
    assert len(out["controlled_pairs"]) == 1
    assert out["controlled_pairs"][0]["repeat"] == 1
    assert out["controlled_pairs"][0]["min_clearance_delta_m"] > 0
    assert out["adaptive_clearance_claim"] is True
    assert out["natural_end_to_end_claim"] is True
    assert out["scenario_solved"] is True



def test_controlled_recovery_requires_reset_after_reacquisition():
    rows = _controlled_rows()
    rows.insert(4, _status(
        0.25, dead_reckoning=0.0, estimated_x=1.4, estimated_y=0.3,
        pose_drift_m=0.0, d_safe_m=0.28, blind_dist_m=0.0))
    rows = [
        row for row in rows
        if not (row.get("topic") == "/safety_gate/status"
                and row.get("t") == 2.2)
    ]

    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))

    assert out["blind_reset_t_s"] is None
    assert out["recovered"] is False


def test_natural_recovery_requires_blind_state_reset_after_marker_zero():
    rows = [
        row for row in _natural_rows()
        if not (row.get("topic") == "/safety_gate/status"
                and row.get("t") == 1.8)
    ]

    out = evaluate_active_field(rows, _manifest("natural_adaptive"))

    assert out["marker_0_reacquire_t_s"] == pytest.approx(1.7)
    assert out["blind_reset_t_s"] is None
    assert out["recovered"] is False


def test_goal_more_than_five_centimetres_away_is_not_reached():
    rows = _controlled_rows()
    for row in rows:
        if row.get("topic") == "/sim/true_pose" and row["t"] >= 3.0:
            row["true_pose"]["x"] = 0.60

    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))

    assert out["final_true_goal_distance_m"] == pytest.approx(0.10)
    assert out["arrive_t_s"] is None
    assert out["reached_goal"] is False


def test_goal_dwell_rejects_turn_in_place_command():
    rows = _controlled_rows()
    for row in rows:
        if row.get("topic") == "/cmd_vel_safe" and row["t"] == 3.0:
            row["twist"]["omega"] = 0.4

    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))

    assert out["reached_goal"] is False


def test_goal_dwell_rejects_intervening_nonzero_command():
    rows = _controlled_rows()
    command = next(
        row for row in rows
        if row.get("topic") == "/cmd_vel_safe"
        and row["t"] == pytest.approx(3.6))
    command["twist"]["v"] = 0.1

    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))

    assert out["reached_goal"] is False


def test_goal_dwell_rejects_missing_command_coverage():
    rows = [
        row for row in _controlled_rows()
        if not (row.get("topic") == "/cmd_vel_safe"
                and 3.0 < row["t"] < 4.0)
    ]

    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))

    assert out["reached_goal"] is False


def test_regressing_trace_timestamp_invalidates_evidence():
    rows = _controlled_rows()
    rows.append(_true(1.0, 1.24, 0.28))

    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))

    assert out["valid"] is False
    assert any("timestamp regression" in reason
               for reason in out["invalid_reasons"])


def test_nonfinite_trace_timestamp_invalidates_evidence():
    rows = _controlled_rows()
    rows.append({"topic": "/diagnostic", "t": float("nan")})

    out = evaluate_active_field(rows, _manifest("controlled_adaptive"))

    assert out["valid"] is False
    assert any("non-finite timestamp" in reason
               for reason in out["invalid_reasons"])