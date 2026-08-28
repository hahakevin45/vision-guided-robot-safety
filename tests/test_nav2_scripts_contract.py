from pathlib import Path


HEADLESS = Path("gazebo_sim/scripts/run_nav2_scenario.sh")
GUI = Path("gazebo_sim/scripts/run_nav2_gui.sh")


def test_headless_runner_has_isolation_timeout_and_evidence() -> None:
    text = HEADLESS.read_text(encoding="utf-8")
    for required in (
        "ROS_DOMAIN_ID",
        "IGN_PARTITION",
        "timeout",
        "vgr_nav2.world",
        "/sim/true_pose_raw",
        "/cmd_vel_safe",
        "safe_apf",
        "nav2_integration.acceptance",
        "NAV2_PASS",
        "cleanup",
        "setsid",
        'kill -- "-$pid"',
    ):
        assert required in text


def test_headless_runner_supports_both_odom_modes_and_pose_sources() -> None:
    text = HEADLESS.read_text(encoding="utf-8")
    for value in ("ground_truth", "wheel_odom", "pseudo", "vision"):
        assert value in text
    assert "odom_mode:=" in text
    assert "nav2_marker_map.json" in text


def test_headless_runner_checks_spawn_pose_before_navigation() -> None:
    text = HEADLESS.read_text(encoding="utf-8")
    assert "--start-x 0.7" in text
    assert "--start-y 0.0" in text


def test_gui_runner_uses_same_launch_and_opens_rviz() -> None:
    text = GUI.read_text(encoding="utf-8")
    assert "vgr_nav2_bringup" in text
    assert "navigation.launch.py" in text
    assert "rviz2" in text
    assert "/cmd_vel_safe" in text
    assert "vgr_nav2.world" in text
    assert "nav2_marker_map.json" in text
    assert "setsid" in text
    assert 'kill -- "-$pid"' in text


def test_colcon_setup_is_sourced_with_nounset_temporarily_disabled() -> None:
    for script in (HEADLESS, GUI):
        text = script.read_text(encoding="utf-8")
        assert 'set +u\nsource "$TMP_DIR/install/setup.bash"\nset -u' in text
