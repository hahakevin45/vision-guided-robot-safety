from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_pi_safe_check_script_contains_only_safe_serial_commands():
    script = (ROOT / "scripts" / "pi_safe_check.sh").read_text(encoding="utf-8")

    assert "certify_camera" in script
    assert "ros2_smoke_test" in script
    assert "certify_ros2_topics --controller mock" in script
    assert "certify_ros2_safe_serial" in script
    assert "FORWARD" not in script
    assert "TURN_LEFT" not in script
    assert "TURN_RIGHT" not in script
