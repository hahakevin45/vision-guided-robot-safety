"""safety_sim.filters：passthrough（基準 0）與 clamp_watchdog（基準 1）。

clamp_watchdog 是所有論文方法都必須贏過的樸素基準：
限幅 + 加速度 ramp + pose_age/link_age watchdog。
"""
import math

from vgr_core.motion import DiffDriveParams
from safety_sim.filters.clamp_watchdog import ClampWatchdogFilter
from safety_sim.filters.passthrough import PassthroughFilter
from vgr_core.safety import Observation, Pose, StaticInfo, Twist

STATIC = StaticInfo(
    params=DiffDriveParams(),
    robot_radius_m=0.10,
    max_v_mps=0.3,
    max_omega_rad_s=1.5,
)


def obs(pose=Pose(0.0, 0.0, 0.0), pose_age=0.0, link_age=0.0):
    return Observation(pose=pose, pose_age_s=pose_age,
                       wheel_feedback=(0.0, 0.0), link_age_s=link_age)


def make_watchdog(**kwargs):
    f = ClampWatchdogFilter(**kwargs)
    f.reset(STATIC)
    return f


# --- passthrough ---

def test_passthrough_never_touches_command():
    f = PassthroughFilter()
    f.reset(STATIC)
    wild = Twist(10.0, 9.0)
    d = f.filter(wild, obs(pose=None, pose_age=math.inf, link_age=99.0), t=0.0, dt=0.05)
    assert d.cmd == wild
    assert d.mode == "PASS"


# --- clamp ---

def test_within_limits_passes_unchanged():
    f = make_watchdog(max_accel_mps2=100.0)   # ramp 放寬，單測 clamp
    d = f.filter(Twist(0.2, 1.0), obs(), t=0.0, dt=0.05)
    assert math.isclose(d.cmd.v, 0.2) and math.isclose(d.cmd.omega, 1.0)
    assert d.mode == "PASS"


def test_overspeed_is_clamped():
    f = make_watchdog(max_accel_mps2=100.0)
    d = f.filter(Twist(2.0, -8.0), obs(), t=0.0, dt=0.05)
    assert d.mode == "MODIFIED"
    assert math.isclose(d.cmd.v, 0.3)
    assert math.isclose(d.cmd.omega, -1.5)


def test_accel_ramp_limits_step_change():
    # 0 → 0.3 m/s 的跳變，在 max_accel=0.5 m/s²、dt=0.05 下
    # 單 tick 最多加 0.025 m/s。
    f = make_watchdog(max_accel_mps2=0.5)
    d = f.filter(Twist(0.3, 0.0), obs(), t=0.0, dt=0.05)
    assert d.mode == "MODIFIED"
    assert math.isclose(d.cmd.v, 0.025, abs_tol=1e-9)
    # 連續餵同樣命令，應逐步爬升到 0.3。
    t = 0.05
    for _ in range(30):
        d = f.filter(Twist(0.3, 0.0), obs(), t=t, dt=0.05)
        t += 0.05
    assert math.isclose(d.cmd.v, 0.3, abs_tol=1e-6)


# --- watchdog ---

def test_stale_pose_forces_stop():
    f = make_watchdog(pose_age_limit_s=0.5, max_accel_mps2=100.0)
    d = f.filter(Twist(0.2, 0.0), obs(pose_age=1.0), t=0.0, dt=0.05)
    assert d.mode == "STOP"
    assert math.isclose(d.cmd.v, 0.0) and math.isclose(d.cmd.omega, 0.0)


def test_no_pose_forces_stop():
    f = make_watchdog(max_accel_mps2=100.0)
    d = f.filter(Twist(0.2, 0.0), obs(pose=None, pose_age=math.inf), t=0.0, dt=0.05)
    assert d.mode == "STOP"


def test_stale_link_forces_stop():
    f = make_watchdog(link_age_limit_s=0.4, max_accel_mps2=100.0)
    d = f.filter(Twist(0.2, 0.0), obs(link_age=0.6), t=0.0, dt=0.05)
    assert d.mode == "STOP"


def test_recovers_after_pose_returns():
    f = make_watchdog(pose_age_limit_s=0.5, max_accel_mps2=100.0)
    f.filter(Twist(0.2, 0.0), obs(pose_age=1.0), t=0.0, dt=0.05)
    d = f.filter(Twist(0.2, 0.0), obs(pose_age=0.0), t=0.05, dt=0.05)
    assert d.mode in ("PASS", "MODIFIED")
    assert d.cmd.v > 0.0


def test_debug_channels_exported():
    f = make_watchdog()
    d = f.filter(Twist(0.2, 0.0), obs(pose_age=0.1, link_age=0.2), t=0.0, dt=0.05)
    assert "pose_age_s" in d.debug and "link_age_s" in d.debug
