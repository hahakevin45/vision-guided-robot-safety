"""共用型別：Twist / Pose / Observation / SafetyDecision / StaticInfo。

慣例與 vgr_core.motion 一致：+v 前進、+ω 左轉。
Observation 只放實車拿得到的資訊；ground truth 不進這裡（只給 metrics 用）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Protocol

from vgr_core.geometry.arena_geometry import Box2D
from vgr_core.motion import DiffDriveParams


@dataclass(frozen=True)
class Twist:
    v: float        # m/s
    omega: float    # rad/s

    @staticmethod
    def stop() -> "Twist":
        return Twist(0.0, 0.0)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float

    def distance_to(self, other: "Pose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class Circle:
    """圓形障礙，世界座標。"""
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class Observation:
    """安全層在某個控制 tick 看得到的一切。"""

    pose: Pose | None            # ArUco 估計位姿；從未定位或完全丟失時為 None
    pose_age_s: float            # 這筆位姿距現在多久；沒有位姿時為 math.inf
    wheel_feedback: tuple[float, float]   # 左右輪實際 counts/s（encoder 回報）
    obstacles: tuple[Circle | Box2D, ...] = ()    # 已知障礙（地圖）
    link_age_s: float = 0.0      # 距上次成功下行多久
    # Position-uncertainty bound (m). During dead-reckoning this combines the
    # accepted anchor bound with accumulated odometry drift; filters add it to
    # their safety distance.
    pose_drift_m: float = 0.0
    # SAPF 吸引目標與其年齡。goal=None 表示目前沒有有效目標；goal_age_s
    # 超過 filter 門檻時視同缺目標（Nav2 /plan 情境）。既有 filter 忽略這兩個欄位。
    goal: tuple[float, float] | None = None
    goal_age_s: float = math.inf


@dataclass(frozen=True)
class StaticInfo:
    """filter 初始化一次就好的靜態資訊。"""

    params: DiffDriveParams
    robot_radius_m: float
    geofence: tuple[tuple[float, float], ...] = ()   # 多邊形頂點，空 = 無界
    max_v_mps: float = 0.5
    max_omega_rad_s: float = 3.0


@dataclass(frozen=True)
class SafetyDecision:
    cmd: Twist
    mode: str                                  # "PASS" | "MODIFIED" | "STOP"
    debug: dict[str, float] = field(default_factory=dict)


class SafetyFilter(Protocol):
    name: str

    def reset(self, static_info: StaticInfo) -> None: ...

    def filter(self, desired: Twist, obs: Observation,
               t: float, dt: float) -> SafetyDecision: ...
