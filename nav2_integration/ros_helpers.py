"""Small ROS-adjacent conversions kept importable without ROS installed."""
from __future__ import annotations

import math


def joint_radians_to_counts(
    *,
    left_rad: float,
    right_rad: float,
    left_counts_per_rev: float,
    right_counts_per_rev: float,
    left_joint_sign: int,
    right_joint_sign: int,
) -> tuple[int, int]:
    if left_joint_sign not in (-1, 1) or right_joint_sign not in (-1, 1):
        raise ValueError("joint signs must be -1 or 1")
    scale = 2.0 * math.pi
    return (
        round(left_rad * left_joint_sign * left_counts_per_rev / scale),
        round(right_rad * right_joint_sign * right_counts_per_rev / scale),
    )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)
