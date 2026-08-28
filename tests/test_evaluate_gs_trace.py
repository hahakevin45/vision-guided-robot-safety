import json

import pytest

from gazebo_sim.evaluate_gs_trace import evaluate_trace


def _write_jsonl(path, rows):
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")


def _sample_rows(samples, *, scenario_name="GS", filter_name="clamp_watchdog"):
    rows = [
        {"topic": "metadata", "scenario_name": scenario_name, "filter_name": filter_name},
    ]
    for t, x, v, mode, pose_stamp in samples:
        rows.extend([
            {"topic": "/sim/true_pose", "t": t, "true_pose": {"x": x, "y": 0.0, "theta": 0.0}},
            {"topic": "/cmd_vel_nav", "t": t, "twist": {"v": 0.15, "omega": 0.0}},
            {"topic": "/aruco/pose", "t": t, "stamp_s": pose_stamp,
             "pose": {"x": x, "y": 0.0, "theta": 0.0}},
            {"topic": "/safety_gate/status", "t": t, "mode": mode, "debug": {}},
            {"topic": "/cmd_vel_safe", "t": t, "twist": {"v": v, "omega": 0.0}},
        ])
    return rows


def test_gs1_uses_gazebo_clearance_collision_threshold(tmp_path):
    trace_path = tmp_path / "gs1.jsonl"
    # With the representative 0.23 m radius, x=3.86 has negative clearance.
    # Gazebo acceptance additionally retains a 0.05 m margin.
    _write_jsonl(trace_path, _sample_rows([
        (0.0, 0.50, 0.15, "PASS", 0.0),
        (0.1, 3.86, 0.15, "PASS", 0.1),
    ], scenario_name="GS1"))

    report = evaluate_trace(trace_path, "GS1")

    assert report["scenario"] == "GS1"
    assert report["filter"] == "clamp_watchdog"
    assert report["passed"] is False
    assert report["metrics"]["min_clearance"] == pytest.approx(-0.09)
    assert report["metrics"]["collided"] is True
    assert any("clearance" in reason for reason in report["reasons"])


def test_gs2_allows_three_seconds_to_stop_after_fault(tmp_path):
    trace_path = tmp_path / "gs2.jsonl"
    _write_jsonl(trace_path, _sample_rows([
        (0.0, 0.50, 0.15, "PASS", 0.0),
        (1.0, 0.65, 0.15, "PASS", 1.0),
        (2.0, 0.75, 0.15, "PASS", 1.0),
        (3.5, 0.80, 0.00, "STOP", 1.0),
    ], scenario_name="GS2"))

    report = evaluate_trace(trace_path, "GS2", fault_t0=1.0)

    assert report["scenario"] == "GS2"
    assert report["passed"] is True
    assert report["reasons"] == []
    assert report["metrics"]["time_to_stop_after_fault_s"] == pytest.approx(2.5)
    assert report["metrics"]["collided"] is False


def test_gs2_fault_inference_ignores_clock_offset_ramp(tmp_path):
    """age 偽段（stamp lag ramp）不應被當成 dropout。

    Regression（2026-08-09 GS2 實測）：aruco stamp 與 recorder clock 的
    固定偏移會產生一段 age 0→1.5s 的偽上升；推斷必須落在真正無界的
    dropout 段（最後一段），否則 time_to_stop 從錯誤的 t0 起算。
    """
    trace_path = tmp_path / "gs2_lag.jsonl"
    # t0..10: age 爬升到 1.5 的偽段（lag artifact），之後 age 維持 1.5
    # t=22 起：真 dropout，age 無界上升，t=24 STOP
    _write_jsonl(trace_path, _sample_rows([
        (0.0, 0.50, 0.15, "PASS", 0.0),
        (5.0, 0.60, 0.15, "PASS", 4.0),
        (10.0, 0.70, 0.15, "PASS", 8.5),
        (15.0, 0.80, 0.15, "PASS", 13.5),
        (20.0, 0.90, 0.15, "PASS", 18.5),
        (22.0, 0.95, 0.15, "PASS", 20.0),
        (22.5, 0.96, 0.00, "STOP", 20.0),
        (24.0, 0.96, 0.00, "STOP", 20.0),
    ], scenario_name="GS2"))

    report = evaluate_trace(trace_path, "GS2")

    assert report["passed"] is True
    assert report["metrics"]["time_to_stop_after_fault_s"] == pytest.approx(2.5)
