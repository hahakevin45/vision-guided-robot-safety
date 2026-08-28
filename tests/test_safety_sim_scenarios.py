"""S1–S3 標準情境的端到端整合測試（M1/M2 驗收門檻）。

門檻：
- S1 直衝牆：passthrough 必撞（驗證情境有效）。clamp_watchdog 沒有幾何
  知識，也會撞——S1 是幾何感知方法（M4）的基準，這裡只固定住這個事實。
- S2 行進中 marker 全丟：passthrough 撞牆；clamp_watchdog 須在
  pose_age 超限後快速停下，不碰牆。
- S3 Nav 失控：passthrough 超速；clamp_watchdog 全程限速且不碰撞。
"""
import math

import pytest

from safety_sim import metrics
from safety_sim.filters import make_filter
from safety_sim.runner import run_scenario
from safety_sim.scenario import DEFAULT_ROBOT_RADIUS_M
from safety_sim.scenarios import all_scenario_names, get_scenario


def run(scenario_name: str, filter_name: str):
    scenario = get_scenario(scenario_name)
    return scenario, run_scenario(scenario, make_filter(filter_name))


# --- 情境庫本身 ---

def test_registry_has_s1_to_s3():
    names = all_scenario_names()
    for name in ("S1", "S2", "S3"):
        assert name in names


def test_unknown_scenario_raises():
    with pytest.raises(ValueError):
        get_scenario("S99")


def test_run_is_deterministic():
    _, trace_a = run("S2", "clamp_watchdog")
    _, trace_b = run("S2", "clamp_watchdog")
    assert trace_a.samples[-1].true_pose == trace_b.samples[-1].true_pose


def test_scenarios_use_representative_robot_radius_in_world_and_static_info():
    for scenario_name in all_scenario_names():
        scenario = get_scenario(scenario_name)
        world = scenario.make_world()
        assert scenario.robot_radius_m == pytest.approx(DEFAULT_ROBOT_RADIUS_M)
        assert world.robot_radius_m == pytest.approx(DEFAULT_ROBOT_RADIUS_M)


# --- S1：全速直衝牆 ---

def test_s1_passthrough_collides():
    scenario, trace = run("S1", "passthrough")
    assert metrics.collided(trace)
    passed, reasons = scenario.evaluate(trace)
    assert not passed and reasons


def test_s1_clamp_watchdog_also_collides_geometry_needed():
    # 固定住「無幾何知識的基準無法通過 S1」這個事實；
    # 若未來這個測試失敗，代表情境變太鬆或 filter 偷看了世界。
    scenario, trace = run("S1", "clamp_watchdog")
    assert metrics.collided(trace)


# --- S2：行進中 marker 全丟 ---

def test_s2_passthrough_collides_during_blackout():
    scenario, trace = run("S2", "passthrough")
    assert metrics.collided(trace)
    assert not scenario.evaluate(trace)[0]


def test_s2_clamp_watchdog_stops_safely():
    scenario, trace = run("S2", "clamp_watchdog")
    assert not metrics.collided(trace)
    # 故障後必須在 1.5 秒內完全停住（含馬達滑行）。
    stop_time = metrics.time_to_stop_after(trace, scenario.fault_t0)
    assert stop_time < 1.5
    # With a representative 0.23 m radius, the body retains >0.25 m clearance.
    assert metrics.min_clearance(trace) > 0.25
    passed, reasons = scenario.evaluate(trace)
    assert passed, reasons


# --- S3：Nav 失控（超速 + 高頻振盪） ---

def test_s3_passthrough_exceeds_speed_limit():
    scenario, trace = run("S3", "passthrough")
    assert metrics.max_speed(trace) > scenario.max_v_mps * 1.2
    assert not scenario.evaluate(trace)[0]


def test_s3_clamp_watchdog_keeps_limits_and_survives():
    scenario, trace = run("S3", "clamp_watchdog")
    assert metrics.max_speed(trace) <= scenario.max_v_mps * 1.05
    assert not metrics.collided(trace)
    passed, reasons = scenario.evaluate(trace)
    assert passed, reasons


# --- 活性指標：filter 不能用「永遠 STOP」蒙混 ---

def test_s2_clamp_watchdog_actually_drives_before_fault():
    scenario, trace = run("S2", "clamp_watchdog")
    pre_fault = [s for s in trace.samples if s.t < scenario.fault_t0]
    assert max(abs(s.actual_twist.v) for s in pre_fault) > 0.1


def test_metrics_report_fields():
    scenario, trace = run("S2", "clamp_watchdog")
    report = metrics.summarize(trace, fault_t0=scenario.fault_t0)
    assert report.collided is False
    assert 0.0 <= report.intervention_ratio <= 1.0
    assert report.cmd_distortion >= 0.0
    assert report.min_clearance > 0.0
