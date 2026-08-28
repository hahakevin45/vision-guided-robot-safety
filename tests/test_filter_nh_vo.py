import math

from vgr_core.motion import DiffDriveParams, twist_to_wheel_counts
from safety_sim.filters import available_filters, make_filter
from safety_sim.filters.nh_vo import NhVoFilter
from vgr_core.safety import Observation, Pose, StaticInfo, Twist


FENCE = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))
STATIC = StaticInfo(params=DiffDriveParams(), robot_radius_m=0.10,
                    geofence=FENCE, max_v_mps=0.15, max_omega_rad_s=1.5)


def make_filter_under_test(**kwargs):
    filt = NhVoFilter(**kwargs)
    filt.reset(STATIC)
    return filt


def feedback_for(twist: Twist):
    return twist_to_wheel_counts(twist.v, twist.omega, STATIC.params)


def obs(pose=Pose(1.0, 0.0, 0.0), pose_age=0.0, link_age=0.0,
        wheel_feedback=(0.0, 0.0)):
    return Observation(pose=pose, pose_age_s=pose_age,
                       wheel_feedback=wheel_feedback, link_age_s=link_age)


def test_registered_in_filter_registry():
    assert "nh_vo" in available_filters()
    assert make_filter("nh_vo").name == "nh_vo"


def test_open_space_straight_command_passes_unchanged():
    filt = make_filter_under_test()
    desired = Twist(0.15, 0.0)
    decision = filt.filter(desired, obs(Pose(1.0, 0.0, 0.0),
                                        wheel_feedback=feedback_for(desired)),
                           t=0.0, dt=0.05)
    assert decision.mode == "PASS"
    assert decision.cmd == desired


def test_near_wall_full_speed_command_is_blocked():
    filt = make_filter_under_test()
    desired = Twist(0.15, 0.0)
    decision = filt.filter(desired, obs(Pose(3.86, 0.0, 0.0),
                                        wheel_feedback=feedback_for(desired)),
                           t=0.0, dt=0.05)
    assert decision.mode in ("MODIFIED", "STOP")
    assert decision.cmd.v < desired.v


def test_stale_pose_forces_stop():
    filt = make_filter_under_test(pose_age_limit_s=0.4)
    decision = filt.filter(Twist(0.15, 0.0),
                           obs(Pose(1.0, 0.0, 0.0), pose_age=0.41),
                           t=0.0, dt=0.05)
    assert decision.mode == "STOP"
    assert decision.cmd == Twist.stop()


def test_missing_pose_and_stale_link_force_stop():
    filt = make_filter_under_test(link_age_limit_s=0.4)
    no_pose = filt.filter(Twist(0.15, 0.0), obs(pose=None, pose_age=math.inf),
                          t=0.0, dt=0.05)
    stale_link = filt.filter(Twist(0.15, 0.0), obs(link_age=0.41),
                             t=0.05, dt=0.05)
    assert no_pose.mode == "STOP"
    assert no_pose.cmd == Twist.stop()
    assert stale_link.mode == "STOP"
    assert stale_link.cmd == Twist.stop()
