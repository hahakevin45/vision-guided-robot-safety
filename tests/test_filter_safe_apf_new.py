"""Contract tests for SafeApfNewFilter.

`/cmd_vel_nav` is motion authorization and liveness only; the attractive
target comes from Observation.goal. Every authorized tick with fresh pose,
goal, and link computes the full SAPF field and outputs MODIFIED. Any invalid
input, reverse request, reached goal, or non-finite math fails closed with
STOP. The existing `safe_apf` filter is not invoked anywhere in this path.
"""
import math

import pytest

from vgr_core.motion import DiffDriveParams
from vgr_core.safety import Circle, Observation, Pose, StaticInfo, Twist

from safety_sim.filters import available_filters, make_filter
from safety_sim.filters.safe_apf_new import SafeApfNewFilter

FENCE = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))
STATIC = StaticInfo(params=DiffDriveParams(), robot_radius_m=0.23,
                    geofence=FENCE, max_v_mps=0.15, max_omega_rad_s=1.5)
GOAL = (3.2, 0.0)
OBSTACLE = Circle(2.0, 0.0, 0.20)


def make_filter_under_test():
    filt = make_filter("safe_apf_new")
    filt.reset(STATIC)
    return filt


def obs(pose=Pose(1.0, 0.0, 0.0), pose_age=0.0, link_age=0.0, goal=GOAL,
        goal_age=0.0, drift=0.0, obstacles=()):
    return Observation(pose=pose, pose_age_s=pose_age,
                       wheel_feedback=(0.0, 0.0), link_age_s=link_age,
                       obstacles=obstacles, goal=goal, goal_age_s=goal_age,
                       pose_drift_m=drift)


def test_registered_in_filter_registry():
    assert "safe_apf_new" in available_filters()
    assert make_filter("safe_apf_new").name == "safe_apf_new"


def test_uses_goal_not_desired_heading_for_attraction():
    # desired pushes +x, goal is at +y; output must aim at the goal
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0),
                           obs(Pose(0.5, 0.0, 0.0), goal=(0.0, 3.0)),
                           t=0.0, dt=0.05)
    assert decision.mode == "MODIFIED"
    assert decision.cmd.omega > 0.0  # turn left toward +y goal


def test_goal_ahead_gives_forward_command():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0), obs(Pose(1.0, 0.0, 0.0)),
                           t=0.0, dt=0.05)
    assert decision.mode == "MODIFIED"
    assert decision.cmd.v > 0.0
    assert abs(decision.cmd.omega) < 0.1  # roughly straight ahead


def test_missing_goal_stops():
    filt = make_filter_under_test()
    assert filt.filter(Twist(0.15, 0.0), obs(goal=None),
                       t=0.0, dt=0.05).mode == "STOP"


def test_stale_goal_stops():
    filt = make_filter_under_test()
    assert filt.filter(Twist(0.15, 0.0), obs(goal_age=1.0),
                       t=0.0, dt=0.05).mode == "STOP"


def test_missing_pose_stops():
    filt = make_filter_under_test()
    assert filt.filter(Twist(0.15, 0.0), obs(pose=None, pose_age=math.inf),
                       t=0.0, dt=0.05).mode == "STOP"


def test_stale_pose_stops():
    filt = make_filter_under_test()
    assert filt.filter(Twist(0.15, 0.0), obs(pose_age=1.0),
                       t=0.0, dt=0.05).mode == "STOP"


def test_stale_link_stops():
    filt = make_filter_under_test()
    assert filt.filter(Twist(0.15, 0.0), obs(link_age=0.6),
                       t=0.0, dt=0.05).mode == "STOP"


def test_zero_desired_is_ignored_autonomous_planner():
    """B 版：cmd_vel 不再授權——SAPF 是自治局部規劃器，desired.v=0 不停止。

    任務活性由 goal 生命週期表達（stale_goal / goal_reached），不是 cmd_vel。
    車仍應朝 goal 運動。
    """
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.0, 0.0), obs(),
                           t=0.0, dt=0.05)
    assert decision.mode == "MODIFIED"
    assert decision.cmd.v > 0.0


def test_reverse_authorization_stops_with_reason():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(-0.1, 0.0), obs(), t=0.0, dt=0.05)
    assert decision.mode == "STOP"
    assert decision.debug.get("reason") == "unsupported_reverse"


def test_goal_reached_stops():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0), obs(Pose(3.21, 0.0, 0.0)),
                           t=0.0, dt=0.05)
    assert decision.mode == "STOP"
    assert decision.debug.get("reason") == "goal_reached"


def test_active_tick_reports_modified_not_pass():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0), obs(), t=0.0, dt=0.05)
    assert decision.mode == "MODIFIED"


def test_debug_carries_field_diagnostics():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0), obs(), t=0.0, dt=0.05)
    for key in ("goal_x", "goal_y", "gradient_x", "gradient_y",
                "min_obstacle_distance_m", "max_abs_gamma_rad",
                "zeta", "eta", "d_safe_m"):
        assert key in decision.debug, key


def test_pose_drift_reduces_effective_clearance_only():
    filt = make_filter_under_test()
    kwargs = dict(pose=Pose(2.3, 0.0, 0.0), obstacles=(OBSTACLE,))
    d0 = filt.filter(Twist(0.15, 0.0), obs(drift=0.0, **kwargs), t=0.0, dt=0.05)
    d1 = filt.filter(Twist(0.15, 0.0), obs(drift=0.05, **kwargs), t=0.0, dt=0.05)
    assert d0.mode == "MODIFIED" and d1.mode == "MODIFIED"
    # circle at (2, 0) r=0.2: robot at x=2.3 -> boundary distance 0.1
    assert d0.debug["min_obstacle_distance_m"] == pytest.approx(0.1)
    assert d1.debug["min_obstacle_distance_m"] == pytest.approx(0.05)


def test_outside_geofence_stops():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0), obs(Pose(4.5, 0.0, 0.0)),
                           t=0.0, dt=0.05)
    assert decision.mode == "STOP"
    assert decision.debug.get("reason") == "outside_geofence"


def test_inside_obstacle_stops():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0),
                           obs(Pose(2.05, 0.0, 0.0), obstacles=(OBSTACLE,)),
                           t=0.0, dt=0.05)
    assert decision.mode == "STOP"
    assert "obstacle" in decision.debug.get("reason", "")


def test_output_bounded_by_vehicle_limits():
    filt = make_filter_under_test()
    decision = filt.filter(Twist(0.15, 0.0), obs(Pose(0.5, 0.0, 0.0)),
                           t=0.0, dt=0.05)
    assert abs(decision.cmd.v) <= STATIC.max_v_mps + 1e-9
    assert abs(decision.cmd.omega) <= STATIC.max_omega_rad_s + 1e-9


def test_ignore_pose_drift_keeps_fixed_radius():
    # Robot at (1.7, 0) faces OBSTACLE (2,0,r=0.2): boundary distance 0.1.
    # pose_drift=0.5 would erode effective clearance to negative -> STOP.
    kwargs = dict(pose=Pose(1.7, 0.0, 0.0), drift=0.5, obstacles=(OBSTACLE,))
    filt = make_filter_under_test()
    assert filt.filter(Twist(0.15, 0.0), obs(**kwargs),
                       t=0.0, dt=0.05).mode == "STOP"
    # With ignore_pose_drift the true face radius (0.1) is kept -> MODIFIED.
    filt2 = SafeApfNewFilter(ignore_pose_drift=True)
    filt2.reset(STATIC)
    dec = filt2.filter(Twist(0.15, 0.0), obs(**kwargs), t=0.0, dt=0.05)
    assert dec.mode == "MODIFIED"
    assert dec.debug["min_obstacle_distance_m"] == pytest.approx(0.1)


def test_fixed_d_safe_overrides_radius():
    filt = SafeApfNewFilter(ignore_pose_drift=True, fixed_d_safe_m=0.77)
    filt.reset(STATIC)
    assert filt._d_safe == pytest.approx(0.77)
    assert filt._d_vort == pytest.approx(0.77 + 0.12)
    assert filt._Q_star == pytest.approx(2.0 * (0.77 + 0.12) - 0.77 + 0.5)
    # Robot at (1.3, 0): boundary distance 0.5 < 0.77 (inside overridden d_safe).
    dec = filt.filter(Twist(0.15, 0.0),
                      obs(Pose(1.3, 0.0, 0.0), obstacles=(OBSTACLE,)),
                      t=0.0, dt=0.05)
    assert dec.debug["d_safe_m"] == pytest.approx(0.77)


def test_fixed_d_safe_rejects_non_positive():
    with pytest.raises(ValueError):
        SafeApfNewFilter(fixed_d_safe_m=0.0)


def test_drift_expands_field_parameters():
    """a/c 臂：drift 膨脹必須撐開場參數（d_safe/Q*/增益），不只 STOP 閾值。"""
    filt = make_filter_under_test()
    filt.reset(STATIC)
    d0 = filt.filter(Twist(0.15, 0.0), obs(Pose(0.5, 0.0, 0.0), drift=0.0),
                     t=0.0, dt=0.05)
    d1 = filt.filter(Twist(0.15, 0.0), obs(Pose(0.5, 0.0, 0.0), drift=0.25),
                     t=0.0, dt=0.05)
    assert d0.debug["d_safe_m"] == pytest.approx(0.28)
    # drift 0.25 -> d_safe 0.53、d_vort 0.65、Q* 1.27
    assert d1.debug["d_safe_m"] == pytest.approx(0.28 + 0.25)
    # zeta 公式不依賴 d_safe（Eq 23）；eta（Eq 29）隨 Q* 外移放大
    assert d1.debug["eta"] > d0.debug["eta"]


def test_fixed_d_safe_not_expanded_by_drift():
    """d 臂（fixed+ignore）：fixed 0.77 時 drift 不膨脹場參數（恆定大圈）。"""
    filt = SafeApfNewFilter(ignore_pose_drift=True, fixed_d_safe_m=0.77)
    filt.reset(STATIC)
    dec = filt.filter(Twist(0.15, 0.0), obs(Pose(0.5, 0.0, 0.0), drift=0.5),
                      t=0.0, dt=0.05)
    assert dec.debug["d_safe_m"] == pytest.approx(0.77)
