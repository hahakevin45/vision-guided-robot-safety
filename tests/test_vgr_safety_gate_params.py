import sys
from pathlib import Path

# Add the package path to sys.path to enable imports without installing first
pkg_path = Path(__file__).resolve().parent.parent / "ros2_ws/src/vgr_safety_gate"
if str(pkg_path) not in sys.path:
    sys.path.insert(0, str(pkg_path))

import pytest
from vgr_safety_gate.safety_gate_node import parse_geofence


def test_parse_geofence_valid():
    # 4 points (8 elements)
    inputs = [0.0, -1.0, 4.0, -1.0, 4.0, 1.0, 0.0, 1.0]
    expected = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))
    assert parse_geofence(inputs) == expected

    # 3 points (6 elements)
    inputs_3 = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    expected_3 = ((1.0, 2.0), (3.0, 4.0), (5.0, 6.0))
    assert parse_geofence(inputs_3) == expected_3

    # String format with brackets
    str_input = "[0.0, -1.0, 4.0, -1.0, 4.0, 1.0, 0.0, 1.0]"
    assert parse_geofence(str_input) == expected

    # String format without brackets
    str_input_no_brackets = "0.0, -1.0, 4.0, -1.0, 4.0, 1.0, 0.0, 1.0"
    assert parse_geofence(str_input_no_brackets) == expected


def test_parse_geofence_odd_length():
    # 7 elements
    inputs = [0.0, -1.0, 4.0, -1.0, 4.0, 1.0, 0.0]
    with pytest.raises(ValueError) as excinfo:
        parse_geofence(inputs)
    assert "even length" in str(excinfo.value)


def test_parse_geofence_too_few_points():
    # 2 points (4 elements)
    inputs = [0.0, -1.0, 4.0, -1.0]
    with pytest.raises(ValueError) as excinfo:
        parse_geofence(inputs)
    assert "at least 3 points" in str(excinfo.value)

    # 0 elements
    with pytest.raises(ValueError) as excinfo:
        parse_geofence([])
    assert "at least 3 points" in str(excinfo.value)
