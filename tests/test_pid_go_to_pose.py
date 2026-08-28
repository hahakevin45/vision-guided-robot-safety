"""ρ-α-β go-to-pose 控制律的單元＋閉環收斂測試。"""
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws/src/vgr_safety_gate"))

from vgr_safety_gate.pid_go_to_pose import (  # noqa: E402
    PidParams,
    PidState,
    compose_blind_pose,
    pid_step,
)

P = PidParams()


def test_never_reverses():
    """任何相對位置（含 goal 在正後方）v 都不為負。"""
    state = PidState()
    for gx, gy in [(1, 0), (-1, 0), (0, 1), (0, -1), (-0.3, -0.2)]:
        v, _, _ = pid_step((0, 0, 0), (gx, gy, 0), state, P)
        assert v >= 0.0


def test_large_alpha_rotates_in_place():
    """goal 在正後方：先原地轉，不前進。"""
    v, omega, state = pid_step((0, 0, 0), (-1.0, 0.1, 0), PidState(), P)
    assert v == 0.0
    assert abs(omega) >= P.omega_min_rad_s
    assert state.aligning


def test_aligned_drives_forward_with_speed_floor():
    v, omega, state = pid_step((0, 0, 0), (0.06, 0.0, 0), PidState(), P)
    assert v >= P.v_min_mps  # 地板高於馬達靜摩擦
    assert v <= P.v_max_mps
    assert not state.aligning


def test_speed_clamped_far_away():
    v, _, _ = pid_step((0, 0, 0), (5.0, 0.0, 0), PidState(), P)
    assert v == pytest.approx(P.v_max_mps)


def test_at_goal_aligns_final_yaw_then_done():
    # 位置已到但朝向差 90°：只轉不走
    v, omega, state = pid_step((1, 1, 0), (1, 1, math.pi / 2), PidState(), P)
    assert v == 0.0
    assert omega > 0.0
    assert not state.done
    # 朝向也到：done、全零
    v, omega, state = pid_step((1, 1, math.pi / 2 - 0.05),
                               (1, 1, math.pi / 2), PidState(), P)
    assert (v, omega) == (0.0, 0.0)
    assert state.done
    # done 之後恆為零
    v, omega, state = pid_step((0, 0, 0), (9, 9, 0), state, P)
    assert (v, omega) == (0.0, 0.0)


def test_align_hysteresis():
    """一旦進入對準模式，要壓到 exit 閾值以下才恢復前進。"""
    state = PidState(aligning=True)
    alpha_mid = (P.align_exit_rad + P.align_enter_rad) / 2.0
    goal = (math.cos(alpha_mid), math.sin(alpha_mid), 0.0)
    v, _, state = pid_step((0, 0, 0), goal, state, P)
    assert v == 0.0 and state.aligning
    goal_small = (math.cos(0.1), math.sin(0.1), 0.0)
    v, _, state = pid_step((0, 0, 0), goal_small, state, P)
    assert v > 0.0 and not state.aligning


def _simulate(start, goal, params=P, dt=0.05, max_steps=4000):
    """單車運動學閉環：回傳（最終位姿, 步數, 是否曾倒車）。"""
    x, y, th = start
    state = PidState()
    reversed_ever = False
    for step in range(max_steps):
        v, omega, state = pid_step((x, y, th), goal, state, params)
        if state.done:
            return (x, y, th), step, reversed_ever
        if v < 0:
            reversed_ever = True
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = math.atan2(math.sin(th + omega * dt), math.cos(th + omega * dt))
    return (x, y, th), max_steps, reversed_ever


@pytest.mark.parametrize("goal", [
    (0.30, -0.20, 0.0),     # 本次實驗規格：前 30cm 右 20cm 朝向不變
    (0.30, 0.0, 0.0),       # 直線
    (0.0, 0.40, math.pi/2),  # 純側向＋轉 90°
    (-0.30, -0.20, 0.0),    # goal 在後方（先掉頭，不倒車）
])
def test_converges_forward_only(goal):
    final, steps, reversed_ever = _simulate((0.0, 0.0, 0.0), goal)
    assert not reversed_ever
    assert math.hypot(final[0] - goal[0], final[1] - goal[1]) <= P.pos_tol_m + 0.02
    yaw_err = abs(math.atan2(math.sin(final[2] - goal[2]),
                             math.cos(final[2] - goal[2])))
    assert yaw_err <= P.yaw_tol_rad + 0.05
    assert steps * 0.05 < 60.0  # 60 秒內收斂


def test_compose_blind_pose_matches_gate_semantics():
    # 錨點時 odom 朝 +y，之後 odom 沿車頭走 0.3 → map 中沿錨點朝向前進 0.3
    est = compose_blind_pose(
        (2.0, 1.0, 0.0), (0.0, 0.0, math.pi / 2), (0.0, 0.3, math.pi / 2))
    assert est[0] == pytest.approx(2.3)
    assert est[1] == pytest.approx(1.0)
    assert est[2] == pytest.approx(0.0)
