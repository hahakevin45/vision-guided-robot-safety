"""Synthetic trapezoid field and three public experiment scenarios.

The public geometry is a generated fixture, not a measured physical site.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Callable

from ..faults import FaultSchedule, FaultWindow
from ..nav import NavSource, WaypointNav
from ..scenario import DEFAULT_ROBOT_RADIUS_M
from ..types import Pose
from ..world import World

# Synthetic trapezoid; intentionally unrelated to the private lab map.
ARENA = ((0.0, -0.7), (2.5, -0.6), (2.4, 1.9), (0.2, 1.8))

# Common start pose with comfortable wall clearance, facing +x.
START_POSE = Pose(0.6, 0.3, 0.0)


def make_arena(goal: tuple[float, float] | None = None) -> World:
    return World(
        geofence=ARENA,
        obstacles=(),
        robot_radius_m=DEFAULT_ROBOT_RADIUS_M,
        goal=goal,
    )


@dataclass(frozen=True)
class EpisodeConfig:
    name: str
    description: str
    goal: tuple[float, float]
    success_radius_m: float
    make_nav: Callable[[], NavSource]
    duration_s: float = 25.0
    max_v_mps: float = 0.15
    max_omega_rad_s: float = 1.5
    start_pose: Pose = START_POSE
    # Synthetic localization-error model parameters.
    noise_xy_std: float = 0.02
    systematic_bias_m: float = 0.04
    update_hz: float = 15.0
    drift_rate_per_m: float = 0.24
    # 盲段：True 時每個 seed 隨機生成 1–4.2s 的盲段（aruco_dropout）
    blackout: bool = False
    blackout_min_s: float = 1.0
    blackout_max_s: float = 4.2
    blackout_count: int = 3
    vehicle_kwargs: dict = field(default_factory=dict)

    def make_faults(self, seed: int) -> FaultSchedule:
        """每個 seed 產生固定但隨機的盲段排程（非盲段情境回傳空排程）。"""
        if not self.blackout:
            return FaultSchedule()
        rng = random.Random(seed * 7919 + 13)
        windows = []
        # 在 [3, duration-1] 內鋪 blackout_count 個不重疊盲段。
        span_start, span_end = 3.0, self.duration_s - 1.0
        slot = (span_end - span_start) / self.blackout_count
        for k in range(self.blackout_count):
            dur = rng.uniform(self.blackout_min_s, self.blackout_max_s)
            slot_lo = span_start + k * slot
            slot_hi = slot_lo + max(0.0, slot - dur)
            t0 = rng.uniform(slot_lo, max(slot_lo, slot_hi))
            windows.append(FaultWindow(t0, t0 + dur, "aruco_dropout"))
        return FaultSchedule(tuple(windows))

    def fault_t0(self, seed: int) -> float | None:
        sched = self.make_faults(seed)
        if not sched.windows:
            return None
        return min(w.t0 for w in sched.windows)


def _waypoint(goal, max_v, max_omega) -> Callable[[], NavSource]:
    return lambda: WaypointNav(goal=goal, max_v_mps=max_v)


# --- 三類情境 ---

# Goal 25 cm from the right wall (about 2 cm body clearance).
_CORRIDOR_GOAL = (2.15, 0.5)
# Adversarial goal 5 cm in front of the right wall; the robot body cannot fit.
_ADV_GOAL = (2.35, 0.5)
# Diagonal traverse toward the synthetic upper-right corner.
_BLACKOUT_GOAL = (2.12, 1.5)


def all_episodes() -> dict[str, EpisodeConfig]:
    eps = {
        "corridor": EpisodeConfig(
            name="corridor",
            description="貼牆走廊：目標距牆 25cm，考在系統偏差下能否貼牆不撞",
            goal=_CORRIDOR_GOAL,
            success_radius_m=0.12,
            make_nav=_waypoint(_CORRIDOR_GOAL, 0.15, 1.5),
            duration_s=25.0,
        ),
        "adversarial_goal": EpisodeConfig(
            name="adversarial_goal",
            description="對抗 goal：目標在牆前 5cm（牆內），filter 須煞停不得抵達",
            goal=_ADV_GOAL,
            success_radius_m=0.12,
            make_nav=_waypoint(_ADV_GOAL, 0.15, 1.5),
            duration_s=25.0,
        ),
        "blackout_traverse": EpisodeConfig(
            name="blackout_traverse",
            description="盲段穿越：斜穿場地，途中 1–4.2s 隨機視覺斷訊（凍結+drift 上界）",
            goal=_BLACKOUT_GOAL,
            success_radius_m=0.15,
            make_nav=_waypoint(_BLACKOUT_GOAL, 0.15, 1.5),
            duration_s=35.0,
            blackout=True,
        ),
    }
    return eps
