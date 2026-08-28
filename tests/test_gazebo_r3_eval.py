"""R3 Gazebo trace evaluator tests（spec 9.3）。

trace 解析（只取 /sim/true_pose）、sapf_new/passthrough 判定、main 輸出。
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from gazebo_sim.evaluate_r3_trace import (
    evaluate_r3_file,
    main,
    parse_r3_trace,
)


def _write_trace(path: Path, points: list[tuple[float, float]]) -> None:
    rows = []
    for i, (x, y) in enumerate(points):
        rows.append(json.dumps({
            "t": 1.0 + i * 0.1, "topic": "/sim/true_pose",
            "true_pose": {"x": x, "y": y, "theta": 0.0},
        }))
    rows.append(json.dumps({"t": 9.0, "topic": "/cmd_vel_safe",
                            "twist": {"v": 0.0, "omega": 0.0}}))
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")


def test_parse_r3_trace_only_takes_true_pose(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    _write_trace(p, [(1.0, 0.0), (1.5, 0.0)])
    pts = parse_r3_trace(p)
    assert pts == [(1.0, 0.0), (1.5, 0.0)]


def test_parse_r3_trace_empty_is_empty_list(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    p.write_text('{"topic": "metadata", "scenario_name": "x"}\n', encoding="utf-8")
    assert parse_r3_trace(p) == []


def test_sapf_new_passes_when_stops_before_line(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    # 車心最接近 1.75（footprint 淨空 0.02 < 0.05 → fail）
    _write_trace(p, [(1.0, 0.0), (1.70, 0.0), (1.72, 0.0)])
    r = evaluate_r3_file(p, line_x=2.0, arm="sapf_new", run_id="r1")
    assert r.passed  # 1.72 → clearance 0.05 ≥ 0.05 ✓


def test_sapf_new_fails_when_too_close(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    _write_trace(p, [(1.0, 0.0), (1.74, 0.0)])  # clearance 0.03 < 0.05
    r = evaluate_r3_file(p, line_x=2.0, arm="sapf_new", run_id="r1")
    assert not r.passed
    assert not r.outcome.crossed


def test_sapf_new_fails_when_crosses(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    _write_trace(p, [(1.0, 0.0), (2.1, 0.0)])
    r = evaluate_r3_file(p, line_x=2.0, arm="sapf_new", run_id="r1")
    assert r.outcome.crossed
    assert not r.passed


def test_passthrough_passes_only_when_crosses(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    _write_trace(p, [(1.0, 0.0), (2.05, 0.0), (2.30, 0.0)])
    r = evaluate_r3_file(p, line_x=2.0, arm="passthrough", run_id="p1")
    assert r.passed
    assert r.outcome.crossed
    # capture depth = 車頭前緣越線深度 = 0.30 − 0.23
    assert r.outcome.capture_depth_m == pytest.approx(0.30 - 0.23)


def test_passthrough_fails_when_blocked(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    # 車心 1.76：車頭 1.99 未越線 → passthrough 負對照失敗（沒有穿越）
    _write_trace(p, [(1.0, 0.0), (1.76, 0.0)])
    r = evaluate_r3_file(p, line_x=2.0, arm="passthrough", run_id="p1")
    assert not r.outcome.crossed
    assert not r.passed


def test_main_writes_eval_json_and_returns_status(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    _write_trace(p, [(1.0, 0.0), (1.70, 0.0)])
    out = tmp_path / "eval.json"
    rc = main([str(p), "2.0", "sapf_new", "--run-id", "r9", "--out", str(out)])
    assert rc == 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["run_id"] == "r9"
    assert payload["crossed"] is False


def test_cbf_arm_uses_sapf_semantics(tmp_path: Path):
    """CBF 與 SAPF-new 同一判定：不越線且 true clearance >= 0.05。"""
    p = tmp_path / "t.jsonl"
    _write_trace(p, [(1.0, 0.0), (1.72, 0.0)])  # clearance 0.05
    r = evaluate_r3_file(p, line_x=2.0, arm="cbf", run_id="c1")
    assert r.passed
    assert not r.outcome.crossed


def test_cbf_arm_fails_when_crosses(tmp_path: Path):
    p = tmp_path / "t.jsonl"
    _write_trace(p, [(1.0, 0.0), (2.1, 0.0)])
    r = evaluate_r3_file(p, line_x=2.0, arm="cbf", run_id="c1")
    assert r.outcome.crossed
    assert not r.passed
