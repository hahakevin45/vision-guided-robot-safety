import math

import pytest

from vgr_core.motion import Pose2D, map_to_odom, wrap_angle


def assert_pose(actual: Pose2D, expected: Pose2D) -> None:
    assert actual.x == pytest.approx(expected.x)
    assert actual.y == pytest.approx(expected.y)
    assert wrap_angle(actual.theta - expected.theta) == pytest.approx(0.0)


def test_map_to_odom_reconstructs_landmark_pose() -> None:
    map_base = Pose2D(2.0, 1.0, math.pi / 2)
    odom_base = Pose2D(0.5, 0.0, math.pi / 4)
    correction = map_to_odom(map_base, odom_base)
    assert_pose(correction.compose(odom_base), map_base)


def test_identity_local_odom_keeps_map_pose() -> None:
    map_base = Pose2D(1.2, -0.4, -0.3)
    assert_pose(map_to_odom(map_base, Pose2D.identity()), map_base)


def test_inverse_undoes_translation_and_rotation() -> None:
    pose = Pose2D(1.0, 2.0, 1.2)
    assert_pose(pose.compose(pose.inverse()), Pose2D.identity())


def test_angles_are_wrapped_to_canonical_interval() -> None:
    assert wrap_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    assert wrap_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
