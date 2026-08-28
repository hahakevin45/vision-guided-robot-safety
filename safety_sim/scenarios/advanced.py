"""S4–S7：degraded sensing、漂移、幾何邊界與活性情境。

S4/S5/S6 的共同設計：危險需要幾何知識才躲得掉——passthrough 與
clamp_watchdog 預期都會撞（由測試固定住），這三格是幾何感知方法的戰場。
S7 是活性關卡：所有 filter 都必須讓車順利抵達目標。
"""
from __future__ import annotations

import math

from ..faults import FaultSchedule, FaultWindow
from ..nav import ScriptedNav, WaypointNav
from ..scenario import DEFAULT_ROBOT_RADIUS_M, Expectation, Scenario
from ..types import Pose, Twist
from ..world import World
from .basic import ARENA, _make_arena


def make_s4_intermittent_blackout() -> Scenario:
    """S4：行進中反覆出現 1.5 秒的位姿黑洞（延遲尖峰的悲觀近似）。

    「黑洞時停、恢復就走」的策略每次恢復都更接近牆；沒有幾何知識，
    時間夠長仍然撞上。
    """
    return Scenario(
        name="S4",
        description="間歇性位姿黑洞：停停走走仍持續逼近牆，須幾何感知",
        make_world=_make_arena,
        make_nav=lambda: ScriptedNav(((0.0, Twist(0.15, 0.0)),)),
        faults=FaultSchedule((
            FaultWindow(4.0, 5.5, "aruco_dropout"),
            FaultWindow(8.0, 9.5, "aruco_dropout"),
            FaultWindow(12.0, 13.5, "aruco_dropout"),
        )),
        duration_s=35.0,
        fault_t0=4.0,
        expectation=Expectation(require_no_collision=True),
    )


def make_s5_wheel_asymmetry_drift() -> Scenario:
    """S5：右輪打滑 15%，直行命令實際走弧線往側牆漂。"""
    return Scenario(
        name="S5",
        description="輪速不對稱漂移：直行命令走弧線，須幾何感知擋住側牆",
        make_world=_make_arena,
        make_nav=lambda: ScriptedNav(((0.0, Twist(0.15, 0.0)),)),
        vehicle_kwargs={"right_speed_scale": 0.85},
        duration_s=30.0,
        expectation=Expectation(require_no_collision=True),
    )


def make_s6_corner_approach() -> Scenario:
    """S6：從場地中段斜角直衝 geofence 角落，兩面牆同時逼近。"""
    corner_heading = math.atan2(1.0 - (-0.5), 4.0 - 0.5)   # 指向 (4, 1) 角落
    return Scenario(
        name="S6",
        description="斜角衝 geofence 角落：兩條邊界同時起作用的幾何邊界情況",
        make_world=_make_arena,
        make_nav=lambda: ScriptedNav(((0.0, Twist(0.15, 0.0)),)),
        start_pose=Pose(0.5, -0.5, corner_heading),
        duration_s=35.0,
        expectation=Expectation(require_no_collision=True),
    )


def _make_arena_with_goal() -> World:
    return World(geofence=ARENA, obstacles=(), robot_radius_m=DEFAULT_ROBOT_RADIUS_M,
                 goal=(3.0, 0.5))


def make_s7_nominal_waypoint() -> Scenario:
    """S7：正常 waypoint 任務，全程無故障。活性關卡。"""
    return Scenario(
        name="S7",
        description="正常 waypoint 任務：filter 不得阻礙抵達目標（活性）",
        make_world=_make_arena_with_goal,
        make_nav=lambda: WaypointNav(goal=(3.0, 0.5), max_v_mps=0.15),
        duration_s=40.0,
        expectation=Expectation(require_no_collision=True,
                                max_final_goal_distance_m=0.15),
    )
