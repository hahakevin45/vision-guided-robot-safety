"""差動輪逆運動學：把 cmd_vel (v, ω) 換成左右輪目標 counts/s。

這是「用 cmd_vel 控車」的核心，且完全不依賴 ROS。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# firmware protocol.h 的 VGR_MAX_TARGET_COUNTS_PER_S
DEFAULT_MAX_COUNTS_PER_S = 900


@dataclass(frozen=True)
class DiffDriveParams:
    """車體幾何 + 編碼器參數。距離單位公尺。"""

    wheel_base_m: float = 0.165          # 兩輪接地中心線間距 (量測值)
    wheel_diameter_m: float = 0.065      # 輪徑
    left_counts_per_rev: float = 750.0
    right_counts_per_rev: float = 749.0
    max_counts_per_s: int = DEFAULT_MAX_COUNTS_PER_S

    def __post_init__(self) -> None:
        for name in ("wheel_base_m", "wheel_diameter_m",
                     "left_counts_per_rev", "right_counts_per_rev"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
        if self.max_counts_per_s <= 0:
            raise ValueError("max_counts_per_s must be positive, got {self.max_counts_per_s}")


def _mps_to_counts_per_s(v_wheel_mps: float, circumference_m: float, counts_per_rev: float) -> float:
    rev_per_s = v_wheel_mps / circumference_m
    return rev_per_s * counts_per_rev


def twist_to_wheel_counts(
    linear_x_mps: float,
    angular_z_rad_s: float,
    params: DiffDriveParams,
) -> tuple[int, int]:
    """cmd_vel (linear.x, angular.z) → (left_counts_per_s, right_counts_per_s)。

    超過 max_counts_per_s 時等比例縮放兩輪，保留轉彎曲率。回傳整數，可直接餵 SET_WHEEL_SPEED (int16)。
    """
    half_base = params.wheel_base_m / 2.0
    v_left_mps = linear_x_mps - angular_z_rad_s * half_base
    v_right_mps = linear_x_mps + angular_z_rad_s * half_base

    circumference_m = math.pi * params.wheel_diameter_m
    left_cps = _mps_to_counts_per_s(v_left_mps, circumference_m, params.left_counts_per_rev)
    right_cps = _mps_to_counts_per_s(v_right_mps, circumference_m, params.right_counts_per_rev)

    # 等比例夾制
    peak = max(abs(left_cps), abs(right_cps))
    if peak > params.max_counts_per_s:
        scale = params.max_counts_per_s / peak
        left_cps *= scale
        right_cps *= scale

    return int(round(left_cps)), int(round(right_cps))
