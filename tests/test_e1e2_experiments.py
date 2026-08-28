"""E1/E2 實驗最小測試：情境幾何 + 校準收斂 + 濾波器行為 + 盲段分岔驗證。"""
from __future__ import annotations

import math
import statistics

from safety_sim.experiments.e1e2_scenarios import (BLIND_APPROACH,
                                                    REVERSE_INTO_WALL,
                                                    E1E2EpisodeConfig)
from safety_sim.experiments.run_e1e2 import (_distance_to_wall, aggregate,
                                              calibrate_cbf, run_e1,
                                              run_episode)


def test_blind_approach_geometry():
    cfg = BLIND_APPROACH
    assert cfg.name == "blind_approach"
    assert cfg.target_wall_x == 5.0
    assert cfg.blind_at_distance_m == 0.6
    # 車起始距牆 1.2m
    assert math.isclose(_distance_to_wall(cfg.start_pose.x, cfg.target_wall_x), 1.2)
    # 盲段觸發點距牆 0.6m：車需前進 0.6m 才觸發
    trigger_x = 3.8 + (1.2 - 0.6)  # start + distance traveled = 4.4
    assert math.isclose(_distance_to_wall(trigger_x, cfg.target_wall_x), 0.6)
    # goal 距牆 5cm
    assert math.isclose(_distance_to_wall(cfg.goal[0], cfg.target_wall_x), 0.05)
    # 車朝 +x 前方
    assert math.isclose(cfg.start_pose.theta, 0.0)


def test_reverse_into_wall_geometry():
    cfg = REVERSE_INTO_WALL
    assert cfg.name == "reverse_into_wall"
    assert cfg.target_wall_x == 0.0
    # 車尾對牆 0.8m（牆在 x=0，車在 x=0.8，朝 +x）
    assert math.isclose(_distance_to_wall(cfg.start_pose.x, cfg.target_wall_x), 0.8)
    assert math.isclose(cfg.start_pose.theta, 0.0)


def test_calibration_convergence():
    """無盲段 CBF 停距校準：最佳組合與 safe_apf 預設差 <2cm。"""
    cal = calibrate_cbf(BLIND_APPROACH, seed=0)
    assert cal["best_diff_m"] < 0.02, (
        f"calibration failed: best_diff_m={cal['best_diff_m']:.4f} >= 0.02")
    cbf_cal = cal["calibrated"]
    assert cbf_cal is not None
    # 校準結果合理範圍
    assert 0.01 <= cbf_cal["buffer_m"] <= 0.20
    assert 0.1 <= cbf_cal["alpha"] <= 10.0


def test_passthrough_collides_on_blind_approach():
    """baseline（passthrough）在盲段衝牆情境必碰撞（證明危險）。"""
    r = run_episode(BLIND_APPROACH, "passthrough", seed=0)
    assert r.collided, "passthrough should collide on blind_approach"


def test_safe_apf_avoids_collision_blind_approach():
    r = run_episode(BLIND_APPROACH, "safe_apf", seed=0)
    assert not r.collided


def test_cbf_avoids_collision_blind_approach_no_blind():
    r = run_episode(BLIND_APPROACH, "cbf", seed=0, blind_enabled=False)
    assert not r.collided


def test_blind_approach_deterministic():
    a = run_episode(BLIND_APPROACH, "safe_apf", seed=7)
    b = run_episode(BLIND_APPROACH, "safe_apf", seed=7)
    assert a == b


def test_blind_approach_calibrated_stops_similar_to_safe_apf():
    """校準 CBF 與 safe_apf 的無盲段停距差應 <2cm。"""
    cal = calibrate_cbf(BLIND_APPROACH, seed=0)
    cbf_cal = cal["calibrated"]
    r_safe = run_episode(BLIND_APPROACH, "safe_apf", seed=0, blind_enabled=False)
    r_cbf = run_episode(BLIND_APPROACH, "cbf", seed=0, blind_enabled=False,
                        filter_kwargs={"buffer_m": cbf_cal["buffer_m"],
                                       "alpha": cbf_cal["alpha"]})
    diff = abs(r_cbf.true_stop_dist_m - r_safe.true_stop_dist_m)
    assert diff < 0.02, f"calibrated CBF stop dist differs from safe_apf by {diff:.4f}m"
    assert not r_safe.collided
    assert not r_cbf.collided


def test_aggregate_statistics():
    """驗證 aggregate 函式產出正確統計形狀。"""
    from safety_sim.experiments.run_e1e2 import E1E2EpisodeResult
    r1 = E1E2EpisodeResult(collided=False, final_clearance=0.3,
                           true_stop_dist_m=0.5, belief_stop_dist_m=0.5,
                           belief_vs_true_diff_m=0.0, min_clearance=0.3,
                           max_speed_mps=0.15)
    r2 = E1E2EpisodeResult(collided=True, final_clearance=-0.1,
                           true_stop_dist_m=0.3, belief_stop_dist_m=0.4,
                           belief_vs_true_diff_m=0.1, min_clearance=-0.1,
                           max_speed_mps=0.15)
    r3 = E1E2EpisodeResult(collided=False, final_clearance=0.4,
                           true_stop_dist_m=0.6, belief_stop_dist_m=0.6,
                           belief_vs_true_diff_m=0.0, min_clearance=0.4,
                           max_speed_mps=0.15)
    agg = aggregate([r1, r2, r3])
    assert agg["n"] == 3
    assert math.isclose(agg["collision_rate"], 1/3)
    assert math.isclose(agg["true_stop_dist_median_m"], 0.5)
    assert agg["true_stop_dist_min_m"] == 0.3
    assert agg["true_stop_dist_max_m"] == 0.6


def test_reverse_into_wall_passthrough_collides():
    """倒車情境 baseline 應碰撞（無濾波直直倒進牆）。"""
    r = run_episode(REVERSE_INTO_WALL, "passthrough", seed=0)
    assert r.collided


def test_reverse_into_wall_safe_apf_avoids_collision():
    r = run_episode(REVERSE_INTO_WALL, "safe_apf", seed=0)
    assert not r.collided


def test_e1e2_filter_names_valid():
    """確認使用的 filter name 都在 available_filters 內。"""
    from safety_sim.filters import available_filters
    valid = set(available_filters())
    for name in ["passthrough", "safe_apf", "cbf"]:
        assert name in valid, f"filter {name!r} not available"


def test_blind_bifurcation_cbf_vs_safe_apf_diverge():
    """有盲段時 cbf 與 safe_apf 真實停距必須不同（分岔）。"""
    cal = calibrate_cbf(BLIND_APPROACH, seed=0)
    cbf_cal = cal["calibrated"]
    r_safe = run_episode(BLIND_APPROACH, "safe_apf", seed=7, blind_enabled=True)
    r_cbf = run_episode(BLIND_APPROACH, "cbf", seed=7, blind_enabled=True,
                        filter_kwargs={"buffer_m": cbf_cal["buffer_m"],
                                       "alpha": cbf_cal["alpha"]})
    diff = abs(r_safe.true_stop_dist_m - r_cbf.true_stop_dist_m)
    assert diff > 0.005, (
        f"blind bifurcation failed: safe_apf={r_safe.true_stop_dist_m:.4f} vs "
        f"cbf={r_cbf.true_stop_dist_m:.4f}, diff={diff:.4f}m (both filters "
        f"produce near-identical blind stop distances)")


def test_blind_bifurcation_no_blind_match():
    """無盲段時校準後兩濾波器停距差 <2cm（校準保持有效）。"""
    cal = calibrate_cbf(BLIND_APPROACH, seed=0)
    cbf_cal = cal["calibrated"]
    r_safe = run_episode(BLIND_APPROACH, "safe_apf", seed=7, blind_enabled=False)
    r_cbf = run_episode(BLIND_APPROACH, "cbf", seed=7, blind_enabled=False,
                        filter_kwargs={"buffer_m": cbf_cal["buffer_m"],
                                       "alpha": cbf_cal["alpha"]})
    diff = abs(r_safe.true_stop_dist_m - r_cbf.true_stop_dist_m)
    assert diff < 0.02, (
        f"no-blind calibration mismatch: safe_apf={r_safe.true_stop_dist_m:.4f} vs "
        f"cbf={r_cbf.true_stop_dist_m:.4f}, diff={diff:.4f}m")


def test_blind_bifurcation_belief_vs_true_diff():
    """有盲段時信念 vs 真實停距差應顯著（信念被 odom 誤差偏移）。"""
    r = run_episode(BLIND_APPROACH, "safe_apf", seed=7, blind_enabled=True)
    assert not math.isnan(r.belief_vs_true_diff_m)
    assert abs(r.belief_vs_true_diff_m) > 0.001, (
        f"belief-vs-true diff too small: {r.belief_vs_true_diff_m:.4f}m")


def test_blind_bifurcation_field_localizer_not_frozen():
    """盲段中信念位姿不應凍結（pose_age_s 為 0）。"""
    from safety_sim.experiments.field_localizer import FieldLocalizer
    from vgr_core.safety import Pose
    loc = FieldLocalizer(seed=42)
    # 先給一筆正常視覺
    loc.observe(Pose(0.0, 0.0, 0.0), 1.0, dropout=False)
    # 進盲段並移動
    _, age, drift = loc.observe(Pose(0.1, 0.0, 0.0), 1.1, dropout=True)
    assert age == 0.0, f"pose_age_s should be 0 during blind (not frozen), got {age}"
    assert drift > 0.08, f"pose_drift_m should grow during blind, got {drift}"


def test_blind_bifurcation_pose_drift_m_formula():
    """pose_drift_m 應為 0.10 + 0.30×盲走里程。"""
    from safety_sim.experiments.field_localizer import FieldLocalizer
    from vgr_core.safety import Pose
    loc = FieldLocalizer(seed=99, drift_rate_per_m=0.24)
    loc.observe(Pose(0.0, 0.0, 0.0), 0.0, dropout=False)
    # 進盲段，行走 0.5m
    _, _, d1 = loc.observe(Pose(0.5, 0.0, 0.0), 0.1, dropout=True)
    expected = 0.10 + 0.30 * 0.5  # = 0.25
    assert abs(d1 - expected) < 0.05, (
        f"pose_drift_m formula: got {d1:.4f}, expected ≈{expected:.4f}")


def test_blind_bifurcation_e1_results_shape():
    """E1 結果兩「有盲段」列不可相同（核心交付條件）。"""
    cal = calibrate_cbf(BLIND_APPROACH, seed=0)
    cbf_cal = cal["calibrated"]
    e1_data = run_e1(BLIND_APPROACH, cbf_cal, seeds=5)
    safe_blind = e1_data["safe_apf"]["blind"]
    cbf_blind = e1_data["cbf_calibrated"]["blind"]
    assert safe_blind["true_stop_dist_median_m"] != cbf_blind["true_stop_dist_median_m"], (
        "E1 blind rows are identical — bifurcation not working. "
        f"safe_apf median={safe_blind['true_stop_dist_median_m']:.4f}, "
        f"cbf median={cbf_blind['true_stop_dist_median_m']:.4f}")


def test_e1e2_episode_config_has_blind_budget():
    """E1E2EpisodeConfig 支援盲走預算欄位（場地政策）。"""
    cfg = BLIND_APPROACH
    assert cfg.blind_max_s == 60.0
    assert cfg.blind_max_dist_m == 2.0
