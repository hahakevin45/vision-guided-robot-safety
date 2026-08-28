import math

from vgr_driver.cli.aruco_bearing_stream import bearing_and_turn


def test_bearing_and_turn_reports_left_marker():
    bearing_deg, side, turn = bearing_and_turn(-1.0, 1.0)

    assert math.isclose(bearing_deg, -45.0)
    assert side == "左"
    assert turn == "TURN_LEFT"


def test_bearing_and_turn_reports_right_marker():
    bearing_deg, side, turn = bearing_and_turn(1.0, 1.0)

    assert math.isclose(bearing_deg, 45.0)
    assert side == "右"
    assert turn == "TURN_RIGHT"


def test_bearing_and_turn_uses_deadband_for_centered_marker():
    bearing_deg, side, turn = bearing_and_turn(0.05, 1.0, deadband_deg=5.0)

    assert 0.0 < bearing_deg < 5.0
    assert side == "正前方"
    assert turn == "FORWARD"


def test_bearing_and_turn_reports_straight_ahead_when_x_is_zero():
    bearing_deg, side, turn = bearing_and_turn(0.0, 2.0)

    assert bearing_deg == 0.0
    assert side == "正前方"
    assert turn == "FORWARD"
