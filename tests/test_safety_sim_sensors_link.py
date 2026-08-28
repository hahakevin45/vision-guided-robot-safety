"""safety_sim.sensors / link / faults：感測與鏈路的故障行為。

這三個模組決定 Observation 的輸入資料——安全層透過它們取得狀態。
"""
import math

from vgr_core.motion import DiffDriveParams
from safety_sim.faults import FaultSchedule, FaultWindow
from safety_sim.link import CommandLink
from safety_sim.sensors import ArucoLocalizer
from vgr_core.safety import Pose, Twist
from safety_sim.vehicle import DiffDriveVehicle

TRUE_POSE = Pose(1.0, 0.5, 0.3)


# --- faults ---

def test_fault_schedule_windows():
    sched = FaultSchedule((
        FaultWindow(1.0, 2.0, "aruco_dropout"),
        FaultWindow(3.0, 4.0, "link_drop"),
    ))
    assert not sched.active("aruco_dropout", 0.5)
    assert sched.active("aruco_dropout", 1.5)
    assert not sched.active("aruco_dropout", 2.5)
    assert sched.active("link_drop", 3.0)      # 邊界含 t0
    assert not sched.active("link_drop", 4.0)  # 不含 t1
    assert not sched.active("nonexistent", 1.5)


# --- sensors ---

def test_localizer_before_first_fix_returns_none():
    loc = ArucoLocalizer(update_hz=10.0)
    pose, age = loc.observe(TRUE_POSE, t=0.0, dropout=True)
    assert pose is None
    assert age == math.inf


def test_localizer_tracks_pose_and_age():
    loc = ArucoLocalizer(update_hz=10.0)
    pose, age = loc.observe(TRUE_POSE, t=0.0, dropout=False)
    assert pose is not None and age == 0.0
    # 下一個 tick 還沒到更新週期：回傳同一筆，age 增加。
    pose2, age2 = loc.observe(Pose(9.9, 9.9, 0.0), t=0.05, dropout=False)
    assert pose2 == pose
    assert math.isclose(age2, 0.05)


def test_localizer_defaults_use_backfilled_gazebo_vision_characteristics():
    loc = ArucoLocalizer(seed=42)
    pose, age = loc.observe(TRUE_POSE, t=0.0, dropout=False)
    assert pose is not None and age == 0.0
    assert pose != TRUE_POSE
    assert abs(pose.x - TRUE_POSE.x) < 0.4
    assert abs(pose.y - TRUE_POSE.y) < 0.4
    assert abs(pose.theta - TRUE_POSE.theta) < 0.2

    pose2, age2 = loc.observe(Pose(9.9, 9.9, 0.0), t=0.05, dropout=False)
    assert pose2 == pose
    assert math.isclose(age2, 0.05)

    pose3, age3 = loc.observe(Pose(9.9, 9.9, 0.0), t=1.0 / 15.0, dropout=False)
    assert pose3 is not None
    assert pose3 != pose
    assert math.isclose(age3, 0.0)


def test_localizer_dropout_freezes_last_fix_and_age_grows():
    loc = ArucoLocalizer(update_hz=10.0)
    last_fix, _ = loc.observe(TRUE_POSE, t=0.0, dropout=False)
    pose, age = loc.observe(Pose(5.0, 5.0, 0.0), t=2.0, dropout=True)
    assert pose is not None
    assert pose == last_fix                         # 凍結在最後一筆
    assert math.isclose(age, 2.0)


def test_localizer_noise_is_seeded_and_bounded():
    loc_a = ArucoLocalizer(update_hz=10.0, noise_xy_std=0.01, seed=42)
    loc_b = ArucoLocalizer(update_hz=10.0, noise_xy_std=0.01, seed=42)
    pa, _ = loc_a.observe(TRUE_POSE, t=0.0, dropout=False)
    pb, _ = loc_b.observe(TRUE_POSE, t=0.0, dropout=False)
    assert pa == pb                              # 同種子完全重現
    assert abs(pa.x - TRUE_POSE.x) < 0.1         # 噪聲量級合理


# --- link ---

def make_vehicle():
    return DiffDriveVehicle(DiffDriveParams(), motor_time_constant_s=0.01)


def run_vehicle(vehicle, seconds, dt=0.01):
    for _ in range(int(round(seconds / dt))):
        vehicle.step(dt)


def test_link_delivers_command_to_vehicle():
    v = make_vehicle()
    link = CommandLink(timeout_s=0.5)
    link.send(Twist(0.2, 0.0), t=0.0, dropped=False)
    link.poll(v, t=0.0)
    run_vehicle(v, 0.5)
    assert v.twist_actual.v > 0.15
    assert link.age_s(t=0.3) == 0.3


def test_link_drop_blocks_command_and_watchdog_stops_vehicle():
    v = make_vehicle()
    link = CommandLink(timeout_s=0.5)
    link.send(Twist(0.2, 0.0), t=0.0, dropped=False)
    link.poll(v, t=0.0)
    run_vehicle(v, 0.5)
    assert v.twist_actual.v > 0.15

    # 之後所有下行都被丟掉：超過 timeout 板端自動 STOP。
    t = 0.5
    for _ in range(200):
        link.send(Twist(0.2, 0.0), t=t, dropped=True)
        link.poll(v, t=t)
        v.step(0.01)
        t += 0.01
    assert abs(v.twist_actual.v) < 1e-3          # 車停了
    assert link.age_s(t=t) > 1.5                 # host 看得到鏈路斷了多久
