"""R3 虛擬 geofence evaluator 測試（spec 9）。

- signed distance 到虛擬線（法線指向安全側）。
- crossed：footprint 越過線；min true clearance = min(signed) − radius。
- passthrough capture depth（越線 0.20m）。
- aggregate：SAPF-new 10/10 不越線且 clearance ≥ 0.05；passthrough 3/3 越線。
"""
from __future__ import annotations

import math

import pytest

from safety_sim.experiments.r3_geofence import (
    R3StopOutcome,
    R3TraceOutcome,
    VirtualLine,
    aggregate_r3,
    evaluate_r3_stop,
    evaluate_r3_trace,
    signed_distance_to_line,
)
from vgr_safety_gate.sapf_nominal import SapfNominalCore


def _nominal(goal=(3.0, 0.0), stop_radius_m=0.05):
    return SapfNominalCore(
        goal=goal,
        d_g_star=0.30, zeta=1.825741858351,
        v_max=0.15, omega_max=0.25,
        theta_error_max=math.pi / 4.0, k_omega=1.5,
        pose_fresh_s=0.4, stop_radius_m=stop_radius_m,
    )

# 虛擬線：x = 2.0（垂直線），安全側為 −x（法線 (−1, 0)）。
LINE = VirtualLine(p1=(2.0, -1.0), p2=(2.0, 1.0), safe_side_normal=(-1.0, 0.0))
ROBOT_RADIUS = 0.23


def test_signed_distance_positive_on_safe_side():
    assert signed_distance_to_line((1.5, 0.0), LINE) == pytest.approx(0.5)
    assert signed_distance_to_line((1.7, 0.3), LINE) == pytest.approx(0.3)


def test_signed_distance_negative_beyond_line():
    assert signed_distance_to_line((2.5, 0.0), LINE) == pytest.approx(-0.5)


def test_signed_distance_on_line_is_zero():
    assert signed_distance_to_line((2.0, 0.7), LINE) == pytest.approx(0.0)


def test_stop_outcome_not_crossed():
    out = evaluate_r3_stop(LINE, stop_point=(1.72, 0.0),
                           robot_radius=ROBOT_RADIUS)
    assert isinstance(out, R3StopOutcome)
    assert not out.crossed
    # 車心 0.28m → footprint 0.05m（spec 4.1 clearance）
    assert out.min_true_clearance_m == pytest.approx(0.05)


def test_stop_outcome_crossed_with_depth():
    # 車頭前緣越線 0.20m：車心 x = 2.0 − 0.23 + 0.20 = 1.97（車頭朝 +x）
    out = evaluate_r3_stop(LINE, stop_point=(1.97, 0.0),
                           robot_radius=ROBOT_RADIUS)
    assert out.crossed
    assert out.capture_depth_m == pytest.approx(0.20)
    assert out.min_true_clearance_m == pytest.approx(-0.20)


def test_stop_outcome_still_on_safe_side_but_close():
    # 車頭正好貼在線內側：車心 x = 2.0 − 0.23 − 0.01 = 1.76
    out = evaluate_r3_stop(LINE, stop_point=(1.76, 0.0),
                           robot_radius=ROBOT_RADIUS)
    assert not out.crossed
    assert out.min_true_clearance_m == pytest.approx(0.01)


def test_trace_outcome_uses_minimum_over_path():
    pts = [(1.0, 0.0), (1.5, 0.0), (1.76, 0.0), (1.70, 0.0)]
    out = evaluate_r3_trace(LINE, pts, robot_radius=ROBOT_RADIUS)
    assert isinstance(out, R3TraceOutcome)
    assert not out.crossed
    # 最近點 x=1.76 → signed 0.24 → clearance 0.01
    assert out.min_true_clearance_m == pytest.approx(0.01)


def test_trace_outcome_crossed_when_footprint_crosses():
    pts = [(1.0, 0.0), (2.1, 0.0), (2.4, 0.0)]
    out = evaluate_r3_trace(LINE, pts, robot_radius=ROBOT_RADIUS)
    assert out.crossed
    assert out.capture_depth_m == pytest.approx(2.4 - 2.0 - ROBOT_RADIUS)


def test_trace_rejects_empty_or_non_finite_points():
    with pytest.raises(ValueError):
        evaluate_r3_trace(LINE, [], robot_radius=ROBOT_RADIUS)
    with pytest.raises(ValueError):
        evaluate_r3_trace(LINE, [(1.0, float("nan"))], robot_radius=ROBOT_RADIUS)


def test_aggregate_sapf_pass_and_passthrough_cross():
    sapf_ok = [evaluate_r3_stop(LINE, (1.72, 0.0), ROBOT_RADIUS)
               for _ in range(10)]
    passthrough_cross = [evaluate_r3_stop(LINE, (2.43, 0.0), ROBOT_RADIUS)
                         for _ in range(3)]
    agg = aggregate_r3(sapf_ok, passthrough_cross,
                       clearance_requirement_m=0.05)
    assert agg.sapf_passed          # 10/10 不越線且 clearance ≥ 0.05
    assert agg.passthrough_crossed  # 3/3 越線


def test_aggregate_fails_when_sapf_crosses_once():
    sapf = [evaluate_r3_stop(LINE, (1.72, 0.0), ROBOT_RADIUS) for _ in range(9)]
    sapf.append(evaluate_r3_stop(LINE, (2.10, 0.0), ROBOT_RADIUS))
    agg = aggregate_r3(sapf, [], clearance_requirement_m=0.05)
    assert not agg.sapf_passed


def test_aggregate_fails_when_sapf_clearance_too_close():
    # 車心 1.74：signed 0.26 → footprint clearance 0.03 < 0.05，但未越線
    sapf = [evaluate_r3_stop(LINE, (1.74, 0.0), ROBOT_RADIUS)
            for _ in range(10)]
    agg = aggregate_r3(sapf, [], clearance_requirement_m=0.05)
    assert not agg.sapf_passed


def test_aggregate_requires_all_passthrough_cross():
    crossed = [evaluate_r3_stop(LINE, (2.43, 0.0), ROBOT_RADIUS)
               for _ in range(2)]
    stopped = evaluate_r3_stop(LINE, (1.72, 0.0), ROBOT_RADIUS)
    agg = aggregate_r3([], crossed + [stopped], clearance_requirement_m=0.05)
    assert not agg.passthrough_crossed


# --- R3 shared nominal controller（spec 4.2） ---

def test_nominal_core_drives_toward_goal():
    core = _nominal(goal=(3.0, 0.0))
    core.update_pose((1.0, 0.0, 0.0), stamp_s=10.0)
    cmd = core.command(now_s=10.1)
    assert cmd.v > 0.0
    assert abs(cmd.omega) < 1e-9


def test_nominal_core_turns_toward_goal():
    core = _nominal(goal=(3.0, 0.0))
    core.update_pose((1.0, 0.0, math.pi / 2.0), stamp_s=10.0)  # 車頭朝 +y
    cmd = core.command(now_s=10.1)
    assert cmd.omega < 0.0  # 需右轉回到 +x


def test_nominal_core_stops_on_stale_pose():
    core = _nominal(goal=(3.0, 0.0))
    core.update_pose((1.0, 0.0, 0.0), stamp_s=10.0)
    assert core.command(now_s=10.9).v == pytest.approx(0.0)   # age 0.9 > 0.4
    assert core.command(now_s=11.0).v == pytest.approx(0.0)


def test_nominal_core_stops_near_goal():
    core = _nominal(goal=(3.0, 0.0), stop_radius_m=0.05)
    core.update_pose((2.98, 0.0, 0.0), stamp_s=10.0)
    assert core.command(now_s=10.1).v == pytest.approx(0.0)


def test_nominal_core_requires_pose_update_first():
    core = _nominal(goal=(3.0, 0.0))
    with pytest.raises(RuntimeError):
        core.command(now_s=0.0)


def test_nominal_zeta_matches_analytic_gains():
    from safety_sim.sapf_field import compute_analytic_gains

    zeta, _ = compute_analytic_gains(
        d_g_star=0.30, a_max=0.5, v_max=0.15, d_safe=0.28, Q_star=0.80)
    core = _nominal(goal=(3.0, 0.0))
    assert core.zeta == pytest.approx(zeta)
