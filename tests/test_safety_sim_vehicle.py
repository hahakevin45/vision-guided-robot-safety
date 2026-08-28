"""safety_sim.vehicle：差速運動學 plant 的行為測試。

plant 是 ground truth 世界的一部分：吃 Twist 命令（經 twist_to_wheel_counts
量化/飽和），一階馬達延遲趨近目標輪速，積分出真實位姿。
"""
import math

import pytest

from vgr_core.motion import DiffDriveParams
from vgr_core.safety import Pose, Twist
from safety_sim.vehicle import DiffDriveVehicle

PARAMS = DiffDriveParams()


def run_for(vehicle: DiffDriveVehicle, seconds: float, dt: float = 0.01) -> None:
    steps = int(round(seconds / dt))
    for _ in range(steps):
        vehicle.step(dt)


def test_initial_state_is_at_rest():
    v = DiffDriveVehicle(PARAMS)
    assert v.pose == Pose(0.0, 0.0, 0.0)
    assert v.wheel_counts_per_s == (0.0, 0.0)


def test_forward_command_moves_along_x():
    v = DiffDriveVehicle(PARAMS)
    v.set_command(Twist(v=0.1, omega=0.0))
    run_for(v, 2.0)
    assert v.pose.x > 0.05          # 有實際往前走
    assert abs(v.pose.y) < 1e-3     # 直行不應偏移
    assert abs(v.pose.theta) < 1e-2


def test_spin_in_place_changes_heading_not_position():
    v = DiffDriveVehicle(PARAMS)
    v.set_command(Twist(v=0.0, omega=1.0))
    run_for(v, 1.0)
    assert v.pose.theta > 0.3       # +ω 左轉，theta 增加
    assert math.hypot(v.pose.x, v.pose.y) < 5e-3


def test_speed_saturates_at_firmware_limit():
    v = DiffDriveVehicle(PARAMS)
    v.set_command(Twist(v=10.0, omega=0.0))   # 遠超上限
    run_for(v, 3.0)
    circumference = math.pi * PARAMS.wheel_diameter_m
    max_mps = PARAMS.max_counts_per_s / PARAMS.left_counts_per_rev * circumference
    left, right = v.wheel_counts_per_s
    assert left <= PARAMS.max_counts_per_s + 1
    assert right <= PARAMS.max_counts_per_s + 1
    # 穩態實際速度不應超過 firmware 上限對應的線速度
    assert v.twist_actual.v <= max_mps * 1.01


def test_motor_lag_wheel_speed_not_instant():
    v = DiffDriveVehicle(PARAMS, motor_time_constant_s=0.2)
    v.set_command(Twist(v=0.2, omega=0.0))
    v.step(0.01)
    left_early, _ = v.wheel_counts_per_s
    run_for(v, 2.0)
    left_settled, _ = v.wheel_counts_per_s
    assert left_early < 0.3 * left_settled   # 一開始遠低於穩態
    assert left_settled > 0


def test_default_motor_coast_reaches_expected_stop_decay():
    v = DiffDriveVehicle(PARAMS)
    v.set_command(Twist(v=0.2, omega=0.0))
    run_for(v, 1.0)
    settled_left, settled_right = v.wheel_counts_per_s
    v.stop()
    run_for(v, 0.25)
    left, right = v.wheel_counts_per_s
    # Default tau=0.08 s should decay below 5% about 0.25 s after STOP.
    assert abs(left) < abs(settled_left) * 0.05
    assert abs(right) < abs(settled_right) * 0.05


def test_wheel_asymmetry_causes_heading_drift():
    # 右輪打滑（實際速度只有命令的 85%）→ 直行命令下往右偏（theta 變負）。
    v = DiffDriveVehicle(PARAMS, right_speed_scale=0.85)
    v.set_command(Twist(v=0.15, omega=0.0))
    run_for(v, 3.0)
    assert v.pose.theta < -0.05


def test_stop_decays_wheels_to_zero():
    v = DiffDriveVehicle(PARAMS)
    v.set_command(Twist(v=0.2, omega=0.0))
    run_for(v, 1.0)
    v.stop()
    run_for(v, 2.0)
    left, right = v.wheel_counts_per_s
    assert abs(left) < 1.0 and abs(right) < 1.0
    assert abs(v.twist_actual.v) < 1e-3


def test_pose_dataclasses_are_frozen():
    with pytest.raises(Exception):
        Twist(0.0, 0.0).v = 1.0  # type: ignore[misc]
    with pytest.raises(Exception):
        Pose(0.0, 0.0, 0.0).x = 1.0  # type: ignore[misc]
