"""M3：WaypointNav 與 S4–S7 標準情境。

門檻設計（幾何感知的分水嶺）：
- S4 間歇性位姿黑洞：passthrough 與 clamp_watchdog 都會在恢復期間
  持續逼近牆而最終撞上；只有幾何感知方法能過。
- S5 輪速不對稱漂移：直行命令實際走弧線，無幾何知識者撞側牆。
- S6 斜角衝 geofence 角落：同上。
- S7 正常 waypoint（活性）：所有 filter 都必須通過且抵達目標——
  「永遠 STOP」在這裡會被抓出來。
"""
import math

from safety_sim import metrics
from safety_sim.filters import make_filter
from safety_sim.nav import WaypointNav
from safety_sim.runner import run_scenario
from safety_sim.scenarios import all_scenario_names, get_scenario
from vgr_core.safety import Observation, Pose, Twist


def obs_at(pose: Pose | None) -> Observation:
    return Observation(pose=pose, pose_age_s=0.0 if pose else math.inf,
                       wheel_feedback=(0.0, 0.0))


def run(scenario_name: str, filter_name: str):
    scenario = get_scenario(scenario_name)
    return scenario, run_scenario(scenario, make_filter(filter_name))


# --- WaypointNav 單元 ---

def test_waypoint_nav_drives_toward_goal():
    nav = WaypointNav(goal=(2.0, 0.0), max_v_mps=0.15)
    cmd = nav.command(obs_at(Pose(0.0, 0.0, 0.0)), t=0.0)
    assert cmd.v > 0.1
    assert abs(cmd.omega) < 0.1     # 已對準目標，不需要轉


def test_waypoint_nav_turns_toward_offset_goal():
    nav = WaypointNav(goal=(0.0, 2.0), max_v_mps=0.15)
    cmd = nav.command(obs_at(Pose(0.0, 0.0, 0.0)), t=0.0)
    assert cmd.omega > 0.5          # 目標在左邊，+ω 左轉


def test_waypoint_nav_stops_at_goal_and_without_pose():
    nav = WaypointNav(goal=(1.0, 0.0), max_v_mps=0.15)
    assert nav.command(obs_at(Pose(1.0, 0.01, 0.0)), t=0.0) == Twist.stop()
    assert nav.command(obs_at(None), t=0.0) == Twist.stop()


# --- 情境庫 ---

def test_registry_has_s4_to_s7():
    names = all_scenario_names()
    for name in ("S4", "S5", "S6", "S7"):
        assert name in names


# --- S4：間歇性位姿黑洞 ---

def test_s4_passthrough_collides():
    scenario, trace = run("S4", "passthrough")
    assert metrics.collided(trace)


def test_s4_clamp_watchdog_stop_and_resume_is_not_enough():
    # clamp 在每次黑洞期間會停，但恢復後繼續逼近牆——沒有幾何知識，
    # 時間夠長還是撞。這固定住「S4 需要幾何感知」的設計事實。
    scenario, trace = run("S4", "clamp_watchdog")
    assert metrics.collided(trace)
    # 但它確實有在黑洞期間介入停車（不是行為跟 passthrough 一樣）。
    assert metrics.intervention_ratio(trace) > 0.1


# --- S5：輪速不對稱漂移 ---

def test_s5_uncorrected_drift_hits_side_wall():
    for filter_name in ("passthrough", "clamp_watchdog"):
        scenario, trace = run("S5", filter_name)
        assert metrics.collided(trace), filter_name
        # 撞的是側牆（y 方向），不是正前方的牆。
        first_hit = next(s for s in trace.samples if s.clearance < 0)
        assert abs(first_hit.true_pose.y) > 0.5


# --- S6：斜角衝角落 ---

def test_s6_no_geometry_filters_collide():
    for filter_name in ("passthrough", "clamp_watchdog"):
        scenario, trace = run("S6", filter_name)
        assert metrics.collided(trace), filter_name


# --- S7：正常 waypoint（活性） ---

def test_s7_all_baseline_filters_reach_goal():
    for filter_name in ("passthrough", "clamp_watchdog"):
        scenario, trace = run("S7", filter_name)
        passed, reasons = scenario.evaluate(trace)
        assert passed, (filter_name, reasons)
        goal = trace.world.goal
        final = trace.samples[-1].true_pose
        assert math.hypot(final.x - goal[0], final.y - goal[1]) < 0.15


def test_s7_evaluate_catches_always_stop():
    # 用 clamp_watchdog 但把速度上限設為 0 等效不可行——這裡直接驗證
    # evaluate 對「沒到目標」會 FAIL：跑一個永遠 STOP 的 filter。
    class AlwaysStop:
        name = "always_stop"

        def reset(self, static_info):
            pass

        def filter(self, desired, obs, t, dt):
            from vgr_core.safety import SafetyDecision
            return SafetyDecision(cmd=Twist.stop(), mode="STOP")

    scenario = get_scenario("S7")
    trace = run_scenario(scenario, AlwaysStop())
    passed, reasons = scenario.evaluate(trace)
    assert not passed
    assert any("goal" in r for r in reasons)
