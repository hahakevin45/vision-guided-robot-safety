"""ROS-independent differential-drive odometry from absolute encoder counts."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EncoderConfig:
    wheel_base_m: float
    wheel_diameter_m: float
    left_counts_per_rev: float
    right_counts_per_rev: float
    left_sign: int = 1
    right_sign: int = 1

    def __post_init__(self) -> None:
        if self.wheel_base_m <= 0 or self.wheel_diameter_m <= 0:
            raise ValueError("wheel dimensions must be positive")
        if self.left_counts_per_rev <= 0 or self.right_counts_per_rev <= 0:
            raise ValueError("counts per revolution must be positive")
        if self.left_sign not in (-1, 1) or self.right_sign not in (-1, 1):
            raise ValueError("encoder signs must be -1 or 1")


@dataclass(frozen=True)
class OdomState:
    x: float
    y: float
    theta: float
    linear_mps: float
    angular_rad_s: float
    stamp_s: float


def _signed_delta_32(current: int, previous: int) -> int:
    return ((current - previous + 2**31) % 2**32) - 2**31


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DifferentialOdometry:
    def __init__(self, config: EncoderConfig) -> None:
        self.config = config
        self._left: int | None = None
        self._right: int | None = None
        self._stamp_s: float | None = None
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

    def update(self, raw_left: int, raw_right: int, stamp_s: float) -> OdomState:
        if self._stamp_s is None:
            self._left = raw_left
            self._right = raw_right
            self._stamp_s = stamp_s
            return self._state(0.0, 0.0, stamp_s)
        if stamp_s <= self._stamp_s:
            raise ValueError("encoder timestamps must be strictly increasing")
        assert self._left is not None and self._right is not None
        dt = stamp_s - self._stamp_s
        left_counts = _signed_delta_32(raw_left, self._left) * self.config.left_sign
        right_counts = _signed_delta_32(raw_right, self._right) * self.config.right_sign
        circumference = math.pi * self.config.wheel_diameter_m
        left_arc = left_counts * circumference / self.config.left_counts_per_rev
        right_arc = right_counts * circumference / self.config.right_counts_per_rev
        distance = (left_arc + right_arc) / 2.0
        dtheta = (right_arc - left_arc) / self.config.wheel_base_m
        midpoint = self._theta + dtheta / 2.0
        self._x += distance * math.cos(midpoint)
        self._y += distance * math.sin(midpoint)
        self._theta = _wrap(self._theta + dtheta)
        self._left = raw_left
        self._right = raw_right
        self._stamp_s = stamp_s
        return self._state(distance / dt, dtheta / dt, stamp_s)

    def _state(self, linear_mps: float, angular_rad_s: float, stamp_s: float) -> OdomState:
        return OdomState(
            x=self._x,
            y=self._y,
            theta=self._theta,
            linear_mps=linear_mps,
            angular_rad_s=angular_rad_s,
            stamp_s=stamp_s,
        )
