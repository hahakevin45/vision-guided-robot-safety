"""GS3 evaluator gates: each failure mode must fail independently.

Synthetic JSONL traces exercise the same path the headless runner produces:
true pose events, safe commands, and safety status. Every GS3 gate
(collision, clearance, goal miss, no detour, no safe commands) must be able
to fail the verdict on its own.
"""
import json

import pytest

from gazebo_sim.evaluate_gs_trace import evaluate_trace
from safety_sim.scenarios import get_scenario
from safety_sim.types import Pose


def _write_jsonl(path, rows):
    path.write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in rows) + "\n",
        encoding="utf-8",
    )


def _rows(poses, *, modes=None, gamma=None):
    """Build a GS3 trace: `poses` = list of (t, x, y) true poses."""
    modes = modes or ["MODIFIED"] * len(poses)
    gamma = gamma or [0.0] * len(poses)
    rows = [
        {"topic": "metadata", "scenario_name": "GS3", "filter_name": "safe_apf_new"}
    ]
    for (t, x, y), mode, g in zip(poses, modes, gamma):
        rows.append({
            "topic": "/sim/true_pose", "t": t,
            "true_pose": {"x": x, "y": y, "theta": 0.0},
            "actual_twist": {"v": 0.1, "omega": 0.0},
        })
        rows.append({
            "topic": "/cmd_vel_nav", "t": t,
            "twist": {"v": 0.1, "omega": 0.0},
        })
        rows.append({
            "topic": "/cmd_vel_safe", "t": t,
            "twist": {"v": 0.1, "omega": 0.0},
        })
        rows.append({
            "topic": "/safety_gate/status", "t": t,
            "mode": mode,
            "debug": {"max_abs_gamma_rad": g},
        })
    return rows


def _detour_poses():
    # start (0.5, 0) -> detour up to y=1.0 -> final (3.2, 0)
    # box (2.0, 0) 0.4x0.6 inflated by robot 0.23: y-bound 0.53；detour 1.0 安全
    return [
        (0.0, 0.5, 0.0), (1.0, 1.0, 0.1), (2.0, 1.6, 0.55),
        (3.0, 2.0, 0.60), (4.0, 2.4, 0.60), (5.0, 2.8, 0.55),
        (6.0, 3.2, 0.0),
    ]


def test_gs3_passes_with_clear_detour_and_goal(tmp_path):
    trace_path = tmp_path / "pass.jsonl"
    _write_jsonl(trace_path, _rows(_detour_poses(), gamma=[0.0, 0.1, 0.5, 1.2, 0.8, 0.2, 0.0]))
    report = evaluate_trace(trace_path, "GS3")
    assert report["scenario"] == "GS3"
    assert report["safety_sim_scenario"] == "S8"
    assert report["passed"] is True
    assert report["reasons"] == []
    gs3 = report["gs3"]
    assert gs3["final_goal_distance_m"] == pytest.approx(0.0, abs=1e-9)
    assert gs3["max_lateral_deviation_m"] == pytest.approx(0.6)
    assert gs3["max_abs_gamma_rad"] == pytest.approx(1.2)
    assert gs3["safe_command_samples"] == 7
    assert gs3["pass"] is True


def test_gs3_fails_when_goal_missed(tmp_path):
    trace_path = tmp_path / "miss.jsonl"
    poses = _detour_poses()[:-1] + [(7.0, 3.2, 0.4)]  # final y=0.4 -> dist 0.4
    _write_jsonl(trace_path, _rows(poses))
    report = evaluate_trace(trace_path, "GS3")
    assert report["passed"] is False
    assert any("did not reach goal" in r for r in report["reasons"])


def test_gs3_fails_without_detour(tmp_path):
    trace_path = tmp_path / "nodetour.jsonl"
    poses = [(i, 0.5 + i * 0.45, 0.0) for i in range(7)]  # straight line, y=0
    _write_jsonl(trace_path, _rows(poses))
    report = evaluate_trace(trace_path, "GS3")
    assert report["passed"] is False
    assert any("no detour" in r for r in report["reasons"])
    assert report["gs3"]["max_lateral_deviation_m"] == pytest.approx(0.0)


def test_gs3_fails_on_gazebo_collision(tmp_path):
    trace_path = tmp_path / "collide.jsonl"
    # straight through the cylinder center (2.0, 0.0) r=0.2, y=0
    poses = [(i, 1.6 + i * 0.15, 0.0) for i in range(7)]
    _write_jsonl(trace_path, _rows(poses))
    report = evaluate_trace(trace_path, "GS3")
    assert report["passed"] is False
    assert any("collision threshold" in r for r in report["reasons"])


def test_gs3_fails_without_safe_command_samples(tmp_path):
    trace_path = tmp_path / "nostop.jsonl"
    _write_jsonl(trace_path, _rows(_detour_poses(), modes=["STOP"] * 7))
    report = evaluate_trace(trace_path, "GS3")
    assert report["passed"] is False
    assert any("no MODIFIED" in r for r in report["reasons"])


def test_gs3_empty_trace_fails(tmp_path):
    trace_path = tmp_path / "empty.jsonl"
    _write_jsonl(trace_path, [{"topic": "metadata", "scenario_name": "GS3",
                               "filter_name": "safe_apf_new"}])
    report = evaluate_trace(trace_path, "GS3")
    assert report["passed"] is False
    assert report["gs3"]["reasons"] == ["no trace samples"]


def test_gs3_world_contains_obstacle_used_for_clearance(tmp_path):
    # clearance in the evaluator comes from the S8 world, which carries the
    # same obstacle the filter sees; a pose inside the box must produce
    # negative clearance through the world, not the raw pose.
    from vgr_core.geometry.arena_geometry import Box2D

    world = get_scenario("S8").make_world()
    ob = world.obstacles[0]
    assert isinstance(ob, Box2D)
    assert ob.x == pytest.approx(2.0)
    assert ob.size_x == pytest.approx(0.40)
    assert world.min_clearance(Pose(2.0, 0.0, 0.0)) < 0.0
