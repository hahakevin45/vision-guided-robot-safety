"""Small SE(2) helpers for landmark-based map-to-odom localization."""
from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float

    @classmethod
    def identity(cls) -> "Pose2D":
        return cls(0.0, 0.0, 0.0)

    def compose(self, other: "Pose2D") -> "Pose2D":
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return Pose2D(
            x=self.x + c * other.x - s * other.y,
            y=self.y + s * other.x + c * other.y,
            theta=wrap_angle(self.theta + other.theta),
        )

    def inverse(self) -> "Pose2D":
        c = math.cos(self.theta)
        s = math.sin(self.theta)
        return Pose2D(
            x=-c * self.x - s * self.y,
            y=s * self.x - c * self.y,
            theta=wrap_angle(-self.theta),
        )


def map_to_odom(map_base: Pose2D, odom_base: Pose2D) -> Pose2D:
    return map_base.compose(odom_base.inverse())
