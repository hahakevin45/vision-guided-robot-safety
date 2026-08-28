"""大場地 × 定位誤差實驗 harness 的煙霧測試（wrapper，不碰核心）。

只驗證：(1) FieldLocalizer 的誤差語意、(2) 梯形場地被 World 原生吃下、
(3) run_episode 產出合理指標且對抗情境真的危險（baseline 會撞）、
(4) 決定性（同 seed 同結果）。跑很快，不做大樣本。
"""
from __future__ import annotations

import math

from safety_sim.experiments.field_localizer import FieldLocalizer
from safety_sim.experiments.field_scenarios import ARENA, all_episodes, make_arena
from safety_sim.experiments.run_field import run_episode
from vgr_core.safety import Pose


def test_arena_is_polygon_and_nonrectangular():
    # 梯形：四頂點、非軸對齊矩形（World 原生吃多邊形 geofence）。
    w = make_arena()
    assert len(w.geofence) == 4
    xs = {round(x, 3) for x, _ in ARENA}
    ys = {round(y, 3) for _, y in ARENA}
    assert len(xs) == 4 and len(ys) == 4  # 沒有共線的軸對齊邊


def test_localizer_injects_bias_and_drift():
    loc = FieldLocalizer(noise_xy_std=0.0, systematic_bias_m=0.04, seed=1)
    true = Pose(1.0, 0.5, 0.0)
    est, age, drift = loc.observe(true, 0.0, dropout=False)
    # 無噪聲時，est 與 true 的距離應等於系統偏差量值。
    assert est is not None
    assert math.isclose(math.hypot(est.x - true.x, est.y - true.y), 0.04, abs_tol=1e-9)
    # 有視覺時 drift 上界 = 系統偏差量值。
    assert math.isclose(drift, 0.04, abs_tol=1e-9)


def test_localizer_blackout_grows_drift():
    """盲段期間信念基於 odom 積分（非凍結），漂移依合約公式成長。"""
    loc = FieldLocalizer(noise_xy_std=0.0, systematic_bias_m=0.04,
                         drift_rate_per_m=0.24, seed=2)
    est0, _, _ = loc.observe(Pose(0.0, 0.0, 0.0), 0.0, dropout=False)
    # 進盲段：true 前進 0.5m，信念會基於錨點＋位移＋誤差成長（非凍結）。
    loc.observe(Pose(0.25, 0.0, 0.0), 0.1, dropout=True)
    est_b, age_b, drift_b = loc.observe(Pose(0.5, 0.0, 0.0), 0.2, dropout=True)
    # 盲段中信念跟隨 odom 積分前進（不凍結）。
    assert est_b != est0
    # 盲段內 age 恆為 0（信念由 odom 即時更新）。
    assert age_b == 0.0
    # 漂移合約公式：0.10 + 0.30 * blind_path_m（非 systematic_bias + drift_rate * path）。
    assert math.isclose(drift_b, 0.10 + 0.30 * 0.5, abs_tol=1e-10)


def test_adversarial_goal_is_dangerous_for_baseline():
    # baseline（passthrough）在對抗 goal 情境必撞——證明情境真的危險。
    cfg = all_episodes()["adversarial_goal"]
    r = run_episode(cfg, "passthrough", seed=0)
    assert r.collided
    assert not r.success


def test_safe_filter_avoids_collision_on_adversarial_goal():
    cfg = all_episodes()["adversarial_goal"]
    r = run_episode(cfg, "cbf", seed=0)
    assert not r.collided


def test_run_episode_is_deterministic():
    cfg = all_episodes()["corridor"]
    a = run_episode(cfg, "safe_apf", seed=7)
    b = run_episode(cfg, "safe_apf", seed=7)
    assert a == b
