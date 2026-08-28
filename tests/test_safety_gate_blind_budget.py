"""SafetyGateCore 盲走預算（dead-reckoning budget）單元測試。

策略（2026-07-14 檢討）：ArUco 丟失後改用「最後視覺錨點＋odom 增量」推算
位姿，直到盲走距離 >0.5m 或時間 >5s 才讓 pose_age 過期 fail-closed。
"""
import math

import pytest

from gazebo_sim.nodes.safety_gate import SafetyGateCore
from vgr_core.safety import Pose

from tests.test_gazebo_nodes_core import RecordingFilter


def _core(**kw):
    return SafetyGateCore(RecordingFilter(), **kw)


def test_fresh_aruco_uses_vision_pose():
    core = _core()
    core.update_odom_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_aruco_pose(Pose(1.0, 0.5, 0.1), stamp_s=0.0)
    obs, dbg = core.build_observation(now_s=0.1)
    assert obs.pose == Pose(1.0, 0.5, 0.1)
    assert obs.pose_age_s == pytest.approx(0.1)
    assert dbg["dead_reckoning"] == 0.0


def test_stale_aruco_dead_reckons_from_odom_delta():
    core = _core()
    core.update_odom_pose(Pose(5.0, 5.0, 0.0), stamp_s=0.0)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=0.0)  # 錨點
    # 視覺斷 1 秒，odom 前進 0.2m
    core.update_odom_pose(Pose(5.2, 5.0, 0.0), stamp_s=1.0)
    obs, dbg = core.build_observation(now_s=1.0)
    assert dbg["dead_reckoning"] == 1.0
    assert obs.pose.x == pytest.approx(1.2)
    assert obs.pose.y == pytest.approx(0.0)
    assert obs.pose_age_s == pytest.approx(0.0)  # 估計是新鮮的（odom age）
    assert dbg["blind_dist_m"] == pytest.approx(0.2)


def test_dead_reckoning_respects_anchor_rotation():
    core = _core()
    # 錨點時 odom 朝 +y（theta=90°），之後 odom 沿自己車頭走 0.3
    core.update_odom_pose(Pose(0.0, 0.0, math.pi / 2), stamp_s=0.0)
    core.update_aruco_pose(Pose(2.0, 1.0, 0.0), stamp_s=0.0)  # map 中車朝 +x
    core.update_odom_pose(Pose(0.0, 0.3, math.pi / 2), stamp_s=1.0)
    obs, dbg = core.build_observation(now_s=1.0)
    # odom 前進 0.3（車體前向）→ map 中沿 +x 前進 0.3
    assert obs.pose.x == pytest.approx(2.3)
    assert obs.pose.y == pytest.approx(1.0)
    assert obs.pose.theta == pytest.approx(0.0)


def test_blind_distance_budget_exhausted_goes_stale():
    core = _core(blind_max_dist_m=0.5)
    core.update_odom_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=0.0)
    core.update_odom_pose(Pose(0.6, 0.0, 0.0), stamp_s=1.0)  # 盲走 0.6 > 0.5
    obs, dbg = core.build_observation(now_s=1.0)
    assert dbg["dead_reckoning"] == 0.0
    assert obs.pose_age_s == pytest.approx(1.0)  # 真實視覺 age → filter 會停


def test_blind_time_budget_exhausted_goes_stale():
    core = _core(blind_max_s=5.0)
    core.update_odom_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=0.0)
    core.update_odom_pose(Pose(0.1, 0.0, 0.0), stamp_s=6.0)
    obs, dbg = core.build_observation(now_s=6.0)
    assert dbg["dead_reckoning"] == 0.0
    assert obs.pose_age_s == pytest.approx(6.0)


def test_stale_odom_does_not_dead_reckon():
    """odom 自己也斷（serial 問題）就不能推算——回報真實視覺 age。"""
    core = _core()
    core.update_odom_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=0.0)
    core.update_odom_pose(Pose(0.1, 0.0, 0.0), stamp_s=0.2)  # 最後 odom 在 0.2s
    obs, dbg = core.build_observation(now_s=2.0)  # odom age 1.8 > fresh 0.4
    assert dbg["dead_reckoning"] == 0.0
    assert obs.pose_age_s == pytest.approx(2.0)


def test_zero_budget_disables_dead_reckoning():
    core = _core(blind_max_dist_m=0.0)
    core.update_odom_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=0.0)
    core.update_odom_pose(Pose(0.1, 0.0, 0.0), stamp_s=1.0)
    obs, dbg = core.build_observation(now_s=1.0)
    assert dbg["dead_reckoning"] == 0.0
    assert obs.pose_age_s == pytest.approx(1.0)


def test_no_odom_falls_back_to_legacy():
    core = _core()
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=0.0)
    obs, dbg = core.build_observation(now_s=1.0)
    assert dbg["dead_reckoning"] == 0.0
    assert obs.pose_age_s == pytest.approx(1.0)


def test_aruco_update_resets_blind_distance():
    core = _core()
    core.update_odom_pose(Pose(0.0, 0.0, 0.0), stamp_s=0.0)
    core.update_aruco_pose(Pose(1.0, 0.0, 0.0), stamp_s=0.0)
    core.update_odom_pose(Pose(0.4, 0.0, 0.0), stamp_s=1.0)
    core.update_aruco_pose(Pose(1.4, 0.0, 0.0), stamp_s=1.0)  # 視覺回來
    core.update_odom_pose(Pose(0.7, 0.0, 0.0), stamp_s=2.0)  # 再盲走 0.3
    obs, dbg = core.build_observation(now_s=2.0)
    assert dbg["dead_reckoning"] == 1.0
    assert dbg["blind_dist_m"] == pytest.approx(0.3)
    assert obs.pose.x == pytest.approx(1.7)
