"""共用 safety 型別：Twist / Pose / Observation / SafetyDecision / StaticInfo。

慣例與 vgr_core.motion.diff_drive_kinematics 一致：+v 前進、+ω 左轉。
Observation 只放實車拿得到的資訊；ground truth 不進這裡（只給 metrics 用）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Protocol

from vgr_core.motion import DiffDriveParams


@dataclass(frozen=True)
class Twist:
    v: float        # m/s
    omega: float    # rad/s

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Twist):
            if hasattr(other, 'v') and hasattr(other, 'omega'):
                return self.v == other.v and self.omega == other.omega
            return NotImplemented
        return self.v == other.v and self.omega == other.omega

    def __hash__(self) -> int:
        return hash((self.v, self.omega))

    @staticmethod
    def stop() -> "Twist":
        return Twist(0.0, 0.0)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Pose):
            if hasattr(other, 'x') and hasattr(other, 'y') and hasattr(other, 'theta'):
                return self.x == other.x and self.y == other.y and self.theta == other.theta
            return NotImplemented
        return self.x == other.x and self.y == other.y and self.theta == other.theta

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
    obstacles: tuple[Circle, ...] = ()    # 已知障礙（地圖）
    link_age_s: float = 0.0      # 距上次成功下行多久
    # 位姿不確定度上界（m）。盲走（dead-reckoning）時 = 錨點誤差上界＋
    # odom 漂移；filter 應把它加進安全距離。
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
