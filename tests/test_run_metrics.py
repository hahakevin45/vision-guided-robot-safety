#!/usr/bin/env python3
"""安全濾波器對比實驗指標工具的測試。

extract：用 outputs/media_20260718 真數據跑，斷言關鍵值合理。
compare：用兩份指標驗表格結構（列/欄/最佳粗體）。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import compare_runs  # noqa: E402
import extract_run_metrics as erm  # noqa: E402

BAG = ROOT / "outputs/media_20260718/20260718_090844_bag"
REPORT = ROOT / "outputs/media_20260718/field_goal_20260718_090844.json"


@pytest.fixture(scope="module")
def metrics():
    if not BAG.exists() or not REPORT.exists():
        pytest.skip("真數據不存在")
    return erm.compute_metrics(str(BAG), str(REPORT), label="safe_apf_demo")


def test_min_wall_clearance(metrics):
    assert metrics["min_wall_clearance_m"] is not None
    assert metrics["min_wall_clearance_m"] > 0.2


def test_success(metrics):
    assert metrics["success"] is True


def test_fence_no_violation(metrics):
    assert metrics["fence_violations"] == 0
    assert metrics["fence_min_signed_m"] > 0  # 全程在圍欄內


def test_blind_short(metrics):
    assert metrics["blind_total_s"] < 1.0


def test_no_stop_mode(metrics):
    assert metrics["mode_fractions"]["STOP"] == 0
    total = sum(metrics["mode_fractions"].values())
    assert abs(total - 1.0) < 1e-9


def test_intervention_and_jerk_present(metrics):
    assert metrics["intervention_l1"] is not None
    assert metrics["intervention_l1"] >= 0
    assert metrics["smoothness_jerk"] is not None
    assert metrics["smoothness_jerk"] >= 0


def test_odom_sane(metrics):
    assert metrics["odom_path_m"] >= metrics["odom_net_m"] >= 0
    assert metrics["time_to_goal_s"] > 0
    assert metrics["goal_error_m"] >= 0


# --- 純函式單元測試（不依賴真數據） ---

def test_signed_dist_inside_outside():
    poly = erm.FENCE_POLYGON
    inside = erm._signed_dist_to_poly(1.0, 0.5, poly)
    outside = erm._signed_dist_to_poly(-1.0, 0.5, poly)
    assert inside > 0
    assert outside < 0


def test_intervention_l1_pairing():
    # nav 與 safe 完全一致 -> L1 = 0
    nav = [(0.0, 0.1, 0.2), (0.05, 0.1, 0.2)]
    safe = [(0.0, 0.1, 0.2), (0.05, 0.1, 0.2)]
    assert erm._intervention_l1(nav, safe) == 0.0
    # safe 全被壓成 0 -> L1 = |0.1| + |0.2|
    safe0 = [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)]
    assert abs(erm._intervention_l1(nav, safe0) - 0.3) < 1e-9


def test_intervention_l1_tolerance():
    # 間隔超過容忍 -> 無配對
    nav = [(0.0, 0.1, 0.0)]
    safe = [(5.0, 0.0, 0.0)]
    assert erm._intervention_l1(nav, safe) is None


def test_jerk_constant_is_zero():
    xs = [(i * 0.05, 0.08, 0.0) for i in range(10)]
    assert erm._smoothness_jerk(xs) == 0.0


# --- compare_runs 表格結構 ---

def test_compare_table_structure(tmp_path):
    m1 = {
        "label": "run_a", "success": True, "min_wall_clearance_m": 0.5,
        "fence_min_signed_m": 0.3, "fence_violations": 0, "time_to_goal_s": 20.0,
        "goal_error_m": 0.1, "intervention_l1": 0.02, "smoothness_jerk": 0.5,
        "blind_total_s": 0.0, "blind_max_dist_m": 0.0, "odom_net_m": 1.0,
        "odom_path_m": 1.2,
        "mode_fractions": {"PASS": 1.0, "MODIFIED": 0.0, "STOP": 0.0},
    }
    m2 = dict(m1)
    m2 = {**m1, "label": "run_b", "min_wall_clearance_m": 0.3,
          "time_to_goal_s": 25.0, "intervention_l1": 0.05,
          "mode_fractions": {"PASS": 0.9, "MODIFIED": 0.1, "STOP": 0.0}}

    table = compare_runs.build_table([m1, m2])
    lines = table.strip().splitlines()
    # 表頭 + 分隔 + 2 資料列
    assert len(lines) == 4
    assert lines[0].startswith("| label |")
    # 每列欄數一致
    ncol = lines[0].count("|")
    assert all(ln.count("|") == ncol for ln in lines)
    # 欄位改版（2026-07-19）：主淨空=clearance（fence 有號距離）、位置制到點
    assert "clearance(m)" in lines[0] and "reached" in lines[0]
    # 標籤都在
    assert "run_a" in table and "run_b" in table


def test_compare_handles_missing(tmp_path):
    m1 = {"label": "x", "success": True, "min_wall_clearance_m": None,
          "mode_fractions": {}}
    table = compare_runs.build_table([m1])
    assert "| x |" in table
    assert "-" in table  # None 以 - 呈現
