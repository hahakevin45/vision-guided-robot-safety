"""E1/E2 實驗情境：盲段衝牆 × 倒車盲區（wrapper，不改核心）。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from ..nav import NavSource, ScriptedNav, WaypointNav
from ..scenario import DEFAULT_ROBOT_RADIUS_M
from ..types import Pose, Twist
from ..world import World

ARENA_RECT = ((0.0, -2.0), (5.0, -2.0), (5.0, 2.0), (0.0, 2.0))


def make_rect_arena(goal=None):
    return World(
        geofence=ARENA_RECT,
        obstacles=(),
        robot_radius_m=DEFAULT_ROBOT_RADIUS_M,
        goal=goal,
    )


@dataclass(frozen=True)
class E1E2EpisodeConfig:
    name: str
    description: str
    goal: tuple[float, float]
    success_radius_m: float
    make_nav: Callable[[], NavSource]
    duration_s: float
    max_v_mps: float
    max_omega_rad_s: float
    start_pose: Pose
    target_wall_x: float | None = None
    blind_at_distance_m: float | None = None
    noise_xy_std: float = 0.02
    systematic_bias_m: float = 0.04
    update_hz: float = 15.0
    drift_rate_per_m: float = 0.24
    blind_max_s: float = 60.0
    blind_max_dist_m: float = 2.0
    vehicle_kwargs: dict = field(default_factory=dict)

    def make_world(self):
        return make_rect_arena(goal=self.goal)


BLIND_APPROACH = E1E2EpisodeConfig(
    name="blind_approach",
    description="盲段衝牆：車距牆 1.2m 正對衝牆，行進至距牆 0.6m 時視覺斷訊（之後全盲）",
    goal=(4.95, 0.0),
    success_radius_m=0.05,
    make_nav=lambda: WaypointNav(goal=(4.95, 0.0), max_v_mps=0.15),
    duration_s=20.0,
    max_v_mps=0.15,
    max_omega_rad_s=1.5,
    start_pose=Pose(3.8, 0.0, 0.0),
    target_wall_x=5.0,
    blind_at_distance_m=0.6,
    blind_max_s=60.0,
    blind_max_dist_m=2.0,
)

REVERSE_INTO_WALL = E1E2EpisodeConfig(
    name="reverse_into_wall",
    description="倒車盲區：車尾對牆 0.8m，命令=常速倒車 v=-0.05，考 CBF lookahead 盲區",
    goal=(0.0, 0.0),
    success_radius_m=0.05,
    make_nav=lambda: ScriptedNav(segments=((0.0, Twist(-0.05, 0.0)),)),
    duration_s=25.0,
    max_v_mps=0.15,
    max_omega_rad_s=1.5,
    start_pose=Pose(0.8, 0.0, 0.0),
    target_wall_x=0.0,
)
