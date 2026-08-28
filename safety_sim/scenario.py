"""Scenario：世界 + 名目命令 + 故障排程 + 時長 + 判定門檻。

用 factory（make_world / make_nav）而不是實例，讓同一個 Scenario 可以
被多個 filter 重複跑而互不汙染狀態。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable

from .faults import FaultSchedule
from .nav import NavSource
from .types import Pose
from .world import World

if TYPE_CHECKING:
    from .runner import Trace


# Representative 0.40 m × 0.22 m envelope; circumscribed radius ≈ 0.228 m.
DEFAULT_ROBOT_RADIUS_M = 0.23


@dataclass(frozen=True)
class Expectation:
    """情境的 pass/fail 門檻。None = 該項不檢查。"""

    require_no_collision: bool = True
    max_speed_mps: float | None = None
    stop_within_s_after_fault: float | None = None
    max_final_goal_distance_m: float | None = None   # 需 world.goal；活性門檻


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    make_world: Callable[[], World]
    make_nav: Callable[[], NavSource]
    faults: FaultSchedule = FaultSchedule()
    duration_s: float = 20.0
    start_pose: Pose = Pose(0.5, 0.0, 0.0)
    control_hz: float = 20.0
    plant_hz: float = 100.0
    max_v_mps: float = 0.15
    max_omega_rad_s: float = 1.5
    robot_radius_m: float = DEFAULT_ROBOT_RADIUS_M
    link_timeout_s: float = 0.5
    localizer_kwargs: dict = field(default_factory=dict)
    vehicle_kwargs: dict = field(default_factory=dict)
    fault_t0: float | None = None       # 主要故障起點，給 time_to_stop 類指標
    expectation: Expectation = Expectation()

    def evaluate(self, trace: "Trace") -> tuple[bool, list[str]]:
        from . import metrics

        reasons: list[str] = []
        e = self.expectation
        if e.require_no_collision and metrics.collided(trace):
            reasons.append("collided (or left geofence)")
        if e.max_speed_mps is not None:
            top = metrics.max_speed(trace)
            if top > e.max_speed_mps:
                reasons.append(f"max speed {top:.3f} m/s > limit {e.max_speed_mps:.3f}")
        if e.max_final_goal_distance_m is not None:
            goal = trace.world.goal
            final = trace.samples[-1].true_pose
            dist = math.hypot(final.x - goal[0], final.y - goal[1])
            if dist > e.max_final_goal_distance_m:
                reasons.append(
                    f"did not reach goal: final distance {dist:.2f} m > {e.max_final_goal_distance_m} m")
        if e.stop_within_s_after_fault is not None:
            tts = metrics.time_to_stop_after(trace, self.fault_t0)
            if tts > e.stop_within_s_after_fault:
                reasons.append(
                    f"did not stop within {e.stop_within_s_after_fault}s after fault (took {tts:.2f}s)")
        return (not reasons, reasons)
