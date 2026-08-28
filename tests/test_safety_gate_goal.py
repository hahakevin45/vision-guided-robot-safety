"""SafetyGateCore goal/plan/obstacle plumbing for the SAPF filter.

The core supplies the SAPF goal contract: a fixed goal (GS experiments) or a
stamped Nav2 `/plan` lookahead target, plus static obstacle circles. A fixed
goal does not age; a plan does, and a stale, empty, or cleared plan yields no
goal so the filter fails closed.
"""
import math

import pytest
from vgr_core.safety import Pose, Twist

from gazebo_sim.nodes.safety_gate import SafetyGateCore
from vgr_core.safety import Circle, Pose

from tests.test_gazebo_nodes_core import RecordingFilter

OBSTACLE = Circle(2.0, 0.0, 0.20)
PATH = tuple((0.1 + i * 0.1, 0.0) for i in range(20))


def _core(**kw):
    return SafetyGateCore(RecordingFilter(), **kw)


def test_fixed_goal_and_obstacles_flow_into_observation():
    core = _core(fixed_goal=(3.2, 0.0), obstacles=(OBSTACLE,))
    core.update_aruco_pose(Pose(0.5, 0.0, 0.0), stamp_s=0.0)
    obs, dbg = core.build_observation(now_s=5.0)  # fixed goal never ages
    assert obs.goal == (3.2, 0.0)
    assert obs.goal_age_s == 0.0
    assert obs.obstacles == (OBSTACLE,)


def test_no_goal_without_fixed_goal_or_plan():
    core = _core()
    core.update_aruco_pose(Pose(0.5, 0.0, 0.0), stamp_s=0.0)
    obs, _dbg = core.build_observation(now_s=0.1)
    assert obs.goal is None
    assert obs.goal_age_s == math.inf


def test_fresh_plan_provides_lookahead_goal():
    core = _core(plan_lookahead_m=0.35)
    core.update_aruco_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_plan(PATH, stamp_s=0.0)
    obs, _dbg = core.build_observation(now_s=0.1)
    # nearest path point (0.1, 0); 0.35 m forward crosses (0.5, 0)
    assert obs.goal == pytest.approx((0.5, 0.0))
    assert obs.goal_age_s == pytest.approx(0.1)


def test_stale_plan_yields_no_goal():
    core = _core(plan_timeout_s=0.5)
    core.update_aruco_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_plan(PATH, stamp_s=0.0)
    obs, _dbg = core.build_observation(now_s=1.0)
    assert obs.goal is None
    assert obs.goal_age_s == math.inf


def test_cleared_plan_yields_no_goal():
    core = _core()
    core.update_aruco_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_plan(PATH, stamp_s=0.0)
    core.update_plan_clear()
    obs, _dbg = core.build_observation(now_s=0.1)
    assert obs.goal is None
    assert obs.goal_age_s == math.inf


def test_empty_plan_yields_no_goal():
    core = _core()
    core.update_aruco_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_plan((), stamp_s=0.0)
    obs, _dbg = core.build_observation(now_s=0.1)
    assert obs.goal is None
    assert obs.goal_age_s == math.inf


def test_fixed_goal_wins_over_plan():
    core = _core(fixed_goal=(3.2, 0.0))
    core.update_aruco_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_plan(PATH, stamp_s=0.0)
    obs, _dbg = core.build_observation(now_s=0.1)
    assert obs.goal == (3.2, 0.0)


def test_plan_requires_pose():
    core = _core()
    core.update_plan(PATH, stamp_s=0.0)
    obs, _dbg = core.build_observation(now_s=0.1)
    assert obs.goal is None
    assert obs.goal_age_s == math.inf


def test_frozen_aruco_restamp_does_not_reset_blind():
    """pseudo_aruco 凍結後仍重發舊 pose（同 stamp）：不得重置盲走累積。

    dead-reckoning 中 blind_dist_m 必須持續累積（odom 增量），
    直到 budget 超額 fail-closed；重發凍結 pose 是 dropout 語意的一部分。
    """
    from safety_sim.filters import make_filter
    from vgr_core.safety.safety_gate import SafetyGateCore

    core = SafetyGateCore(make_filter("safe_apf_new"),
                          max_v_mps=0.15, max_omega_rad_s=1.5)
    core.update_nav(Twist(0.15, 0.0), stamp_s=0.0)
    # 新鮮修正
    core.update_aruco_pose(Pose(1.2, 0.0, 0.0), stamp_s=10.0)
    core.update_odom_pose(Pose(1.2, 0.0, 0.0), stamp_s=10.0)
    # odom 前進 0.3m
    core.update_odom_pose(Pose(1.5, 0.0, 0.0), stamp_s=10.5)
    blind_after_odom = core.build_observation(10.5)[1]["blind_dist_m"]
    assert blind_after_odom == pytest.approx(0.3)
    # 凍結重發（同 stamp）：不得重置 blind
    core.update_aruco_pose(Pose(1.2, 0.0, 0.0), stamp_s=10.0)
    blind_after_restamp = core.build_observation(10.6)[1]["blind_dist_m"]
    assert blind_after_restamp == pytest.approx(0.3)
    # 新的新鮮修正：重置
    core.update_aruco_pose(Pose(1.8, 0.0, 0.0), stamp_s=12.0)
    assert core.build_observation(12.1)[1]["blind_dist_m"] == pytest.approx(0.0)


def test_build_observation_debug_has_estimated_pose():
    """core_debug 暴露 estimated_x/estimated_y：盲走（within_budget）時為
    dead-reckoned 估測位姿，非盲走時為視覺位姿。

    要走到估測分支需同時滿足盲走預算：aruco 錨點必須在 odom 之後才擷取得到，
    末筆 odom 距 now 必須 ≤ aruco_fresh_s，且盲走距離 ≤ blind_max_dist_m。
    """
    from safety_sim.filters import make_filter
    from vgr_core.safety.safety_gate import SafetyGateCore

    core = SafetyGateCore(make_filter("safe_apf_new"),
                          max_v_mps=0.15, max_omega_rad_s=1.5,
                          blind_max_dist_m=1.0)
    core.update_nav(Twist(0.15, 0.0), stamp_s=0.0)
    # 先建立 odom，讓 aruco 更新可擷取錨點（anchor = 當下 odom）
    core.update_odom_pose(Pose(1.0, 0.0, 0.0), stamp_s=9.0)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=10.0)
    core.update_odom_pose(Pose(1.0, 0.0, 0.0), stamp_s=10.0)
    # 盲走 0.583m；stamp 11.2 距 now 11.5 = 0.3s ≤ aruco_fresh_s
    core.update_odom_pose(Pose(1.5, 0.3, 0.0), stamp_s=11.2)
    obs, debug = core.build_observation(11.5)
    assert obs.pose == Pose(1.5, 0.3, 0.0)  # dead-reckoned 估測位姿
    assert debug["estimated_x"] == pytest.approx(1.5)
    assert debug["estimated_y"] == pytest.approx(0.3)
