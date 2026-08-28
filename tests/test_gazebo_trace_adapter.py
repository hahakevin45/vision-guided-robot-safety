import json

import pytest

from gazebo_sim.trace_adapter import load_trace
from safety_sim.scenarios.basic import _make_arena


def _write_jsonl(path, rows):
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")


def test_load_trace_aligns_samples_to_safe_command_ticks_and_recomputes_clearance(tmp_path):
    path = tmp_path / "gs2.jsonl"
    _write_jsonl(path, [
        {"topic": "/sim/true_pose", "t": 0.00,
         "true_pose": {"x": 0.5, "y": 0.0, "theta": 0.0},
         "actual_twist": {"v": 0.21, "omega": 0.01}},
        {"topic": "/cmd_vel_nav", "t": 0.00, "twist": {"v": 0.15, "omega": 0.0}},
        {"topic": "/aruco/pose", "t": 0.00, "stamp_s": 0.00,
         "pose": {"x": 0.5, "y": 0.0, "theta": 0.0}},
        {"topic": "/safety_gate/status", "t": 0.00, "mode": "PASS", "debug": {"a": 1.0}},
        {"topic": "/cmd_vel_safe", "t": 0.05, "twist": {"v": 0.10, "omega": 0.0}},
        {"topic": "/sim/true_pose", "t": 0.10,
         "true_pose": {"x": 0.51, "y": 0.0, "theta": 0.0},
         "actual_twist": {"v": 0.22, "omega": 0.02}},
        {"topic": "/cmd_vel_nav", "t": 0.10, "twist": {"v": 0.20, "omega": 0.1}},
        {"topic": "/aruco/pose", "t": 0.10, "stamp_s": 0.10,
         "pose": {"x": 0.51, "y": 0.0, "theta": 0.0}},
        {"topic": "/safety_gate/status", "t": 0.10, "mode": "MODIFIED", "debug": {"a": 2.0}},
        {"topic": "/cmd_vel_safe", "t": 0.15, "twist": {"v": 0.12, "omega": 0.1}},
    ])

    world = _make_arena()
    trace = load_trace(path, world)

    assert trace.scenario_name == "gs2"
    assert trace.filter_name == "gazebo"
    assert len(trace.samples) == 2
    first, second = trace.samples
    assert first.t == pytest.approx(0.05)
    assert first.true_pose.x == pytest.approx(0.5)
    assert first.est_pose.x == pytest.approx(0.5)
    assert first.pose_age_s == pytest.approx(0.05)
    assert first.desired.v == pytest.approx(0.15)
    assert first.cmd.v == pytest.approx(0.10)
    assert first.actual_twist.v == pytest.approx(0.21)
    assert first.actual_twist.omega == pytest.approx(0.01)
    assert first.clearance == pytest.approx(world.min_clearance(first.true_pose))
    assert second.mode == "MODIFIED"
    assert second.debug == {"a": 2.0}
    assert second.actual_twist.v == pytest.approx(0.22)
    assert second.actual_twist.omega == pytest.approx(0.02)


def test_load_trace_uses_empty_values_when_optional_topics_have_not_arrived(tmp_path):
    path = tmp_path / "empty_context.jsonl"
    _write_jsonl(path, [
        {"topic": "/sim/true_pose", "t": 0.0,
         "true_pose": {"x": 0.5, "y": 0.0, "theta": 0.0}},
        {"topic": "/cmd_vel_safe", "t": 0.1, "twist": {"v": 0.0, "omega": 0.0}},
    ])

    trace = load_trace(path, _make_arena())
    sample = trace.samples[0]

    assert sample.est_pose is None
    assert sample.pose_age_s == float("inf")
    assert sample.desired.v == 0.0
    assert sample.mode == "UNKNOWN"


def test_load_trace_falls_back_to_safe_command_for_legacy_true_pose_rows(tmp_path):
    path = tmp_path / "legacy.jsonl"
    _write_jsonl(path, [
        {"topic": "/sim/true_pose", "t": 0.0,
         "true_pose": {"x": 0.5, "y": 0.0, "theta": 0.0}},
        {"topic": "/cmd_vel_safe", "t": 0.1, "twist": {"v": 0.12, "omega": 0.3}},
    ])

    trace = load_trace(path, _make_arena())
    sample = trace.samples[0]

    assert sample.actual_twist.v == pytest.approx(0.12)
    assert sample.actual_twist.omega == pytest.approx(0.3)
