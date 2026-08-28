"""S1–S3 standard safety scenarios.

The shared synthetic field is a 4 m × 2 m rectangular geofence. The robot
starts at (0.5, 0), faces +x, and uses a 0.15 m/s research speed limit.
"""
from __future__ import annotations

from ..faults import FaultSchedule, FaultWindow
from ..nav import FunctionNav, ScriptedNav
from ..scenario import DEFAULT_ROBOT_RADIUS_M, Expectation, Scenario
from ..types import Observation, Twist
from ..world import World

ARENA = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))


def _make_arena() -> World:
    return World(geofence=ARENA, obstacles=(), robot_radius_m=DEFAULT_ROBOT_RADIUS_M)


def _make_s1_arena() -> World:
    """S1 吸引目標故意放在禁止穿越的東牆上：SAPF 仍須在牆前停下。"""
    return World(geofence=ARENA, obstacles=(), robot_radius_m=DEFAULT_ROBOT_RADIUS_M,
                 goal=(4.0, 0.0))


def _make_s2_arena() -> World:
    return World(geofence=ARENA, obstacles=(), robot_radius_m=DEFAULT_ROBOT_RADIUS_M,
                 goal=(3.0, 0.0))


def make_s1_wall_rush() -> Scenario:
    """S1：Nav 全速直衝遠端牆。考幾何停止能力。

    無幾何知識的 filter（含 clamp_watchdog 基準）預期無法通過；
    這是幾何感知方法（CBF 之類）的主戰場。
    """
    return Scenario(
        name="S1",
        description="全速直衝牆：只有會看牆的 filter 能通過",
        make_world=_make_s1_arena,
        make_nav=lambda: ScriptedNav(((0.0, Twist(2.0, 0.0)),)),
        duration_s=30.0,
        expectation=Expectation(require_no_collision=True),
    )


def make_s2_marker_blackout() -> Scenario:
    """S2：正常行進中 t=3s 起 marker 全丟。考 pose_age 的降級行為。"""
    return Scenario(
        name="S2",
        description="行進中 marker 全丟：位姿凍結、age 增長，須及時停下",
        make_world=_make_s2_arena,
        make_nav=lambda: ScriptedNav(((0.0, Twist(0.15, 0.0)),)),
        faults=FaultSchedule((FaultWindow(3.0, 25.0, "aruco_dropout"),)),
        duration_s=25.0,
        fault_t0=3.0,
        expectation=Expectation(require_no_collision=True,
                                stop_within_s_after_fault=1.5),
    )


def _runaway_command(obs: Observation, t: float) -> Twist:
    if t < 5.0:
        return Twist(0.10, 0.0)
    # t>=5：Nav 失控——超速 + 2 Hz 高頻正負交替的角速度。
    omega = 4.0 if int(t * 4.0) % 2 == 0 else -4.0
    return Twist(5.0, omega)


def make_s3_nav_runaway() -> Scenario:
    """S3：t=5s 起 Nav 送出超速與高頻振盪命令。考 filter 是否守住限幅。"""
    scenario_max_v = 0.15
    return Scenario(
        name="S3",
        description="Nav 失控：超速 + 高頻振盪，filter 須守住速度與角速度上限",
        make_world=_make_arena,
        make_nav=lambda: FunctionNav(_runaway_command),
        duration_s=20.0,
        max_v_mps=scenario_max_v,
        fault_t0=5.0,
        expectation=Expectation(require_no_collision=True,
                                max_speed_mps=scenario_max_v * 1.05),
    )
