import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import vgr_runtime.cli.pi_nav2_goal_bench as goal_bench
from vgr_runtime.cli.pi_nav2_goal_bench import (
    CommandLimiter,
    GoalEvidence,
    _validate_safe_publishers,
    atomic_json,
    evaluate_goal,
)


def test_limiter_allows_only_bounded_forward_planar_command():
    limiter = CommandLimiter(
        max_linear_mps=0.03,
        max_angular_rad_s=0.25,
        stale_s=0.20,
    )

    limited = limiter.limit(
        linear_x=0.08,
        angular_z=-0.50,
        command_stamp_s=1.0,
        now_s=1.10,
    )

    assert limited.linear_x == 0.03
    assert limited.angular_z == -0.25
    assert limited.was_clamped is True
    assert limited.was_stale is False


def test_limiter_rejects_reverse_and_stops_stale_command():
    limiter = CommandLimiter(0.03, 0.25, 0.20)

    reverse = limiter.limit(-0.01, 0.1, 1.0, 1.05)
    stale = limiter.limit(0.02, 0.1, 1.0, 1.21)

    assert (reverse.linear_x, reverse.angular_z) == (0.0, 0.1)
    assert reverse.was_clamped is True
    assert (stale.linear_x, stale.angular_z) == (0.0, 0.0)
    assert stale.was_stale is True


@pytest.mark.parametrize(
    ("limits", "message"),
    [
        ((0.0, 0.25, 0.20), "positive"),
        ((0.03, 0.0, 0.20), "positive"),
        ((0.03, 0.25, 0.0), "positive"),
    ],
)
def test_limiter_rejects_nonpositive_limits(limits, message):
    with pytest.raises(ValueError, match=message):
        CommandLimiter(*limits)


def passing_evidence(**changes):
    values = dict(
        goal_count=1,
        goal_accepted=True,
        action_status="SUCCEEDED",
        initial_pose=(0.0, 0.0, 0.0),
        final_pose=(0.10, 0.0, 0.0),
        raw_encoder_delta=(36, 35),
        nav_cmd_count=80,
        safe_cmd_count=80,
        plan_count=1,
        max_nav_linear_mps=0.03,
        max_nav_angular_rad_s=0.05,
        max_safe_linear_mps=0.03,
        max_safe_angular_rad_s=0.05,
        max_abs_target_cps=111,
        final_targets=(0, 0),
        zero_target_observation_s=2.1,
        hardware_faults=(),
        safe_publisher_count=1,
        clamp_count=0,
        stale_count=0,
        goal_elapsed_s=5.2,
    )
    values.update(changes)
    return GoalEvidence(**values)


def test_goal_evidence_accepts_bounded_10cm_closed_loop():
    report = evaluate_goal(passing_evidence())

    assert report["pass"] is True
    assert report["metrics"]["delta_x_m"] == 0.10
    assert report["metrics"]["normalized_encoder_delta"] == [36, 35]


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"goal_count": 2}, "exactly one goal"),
        ({"goal_accepted": False}, "not accepted"),
        ({"action_status": "TIMEOUT"}, "action status"),
        ({"goal_elapsed_s": 20.01}, "deadline"),
        ({"final_pose": (0.079, 0.0, 0.0)}, "odometry x"),
        ({"final_pose": (0.121, 0.0, 0.0)}, "odometry x"),
        ({"final_pose": (0.10, 0.031, 0.0)}, "lateral drift"),
        ({"final_pose": (0.10, 0.0, 0.251)}, "yaw error"),
        ({"raw_encoder_delta": (-1, 35)}, "encoder direction"),
        ({"nav_cmd_count": 0}, "/cmd_vel_nav"),
        ({"safe_cmd_count": 0}, "/cmd_vel_safe"),
        ({"plan_count": 0}, "/plan"),
        ({"max_safe_linear_mps": 0.031}, "linear limit"),
        ({"max_safe_angular_rad_s": 0.251}, "angular limit"),
        ({"max_abs_target_cps": 121}, "hardware target"),
        ({"final_targets": (1, 0)}, "final targets"),
        ({"zero_target_observation_s": 1.99}, "zero-target observation"),
        ({"hardware_faults": ("timeout",)}, "hardware fault"),
        ({"safe_publisher_count": 2}, "publisher"),
    ],
)
def test_goal_evidence_rejects_each_unsafe_condition(change, reason):
    report = evaluate_goal(passing_evidence(**change))

    assert report["pass"] is False
    assert any(reason in item for item in report["reasons"])


def test_atomic_json_replaces_temporary_file(tmp_path):
    path = tmp_path / "nested" / "report.json"

    atomic_json(path, {"pass": True})

    assert json.loads(path.read_text(encoding="utf-8")) == {"pass": True}
    assert not path.with_suffix(".json.tmp").exists()


def test_external_relay_requires_safety_gate_as_only_safe_publisher():
    publishers = [SimpleNamespace(node_name="safety_gate", node_namespace="/")]

    assert _validate_safe_publishers(publishers, external_relay=True) == 1


@pytest.mark.parametrize(
    "publishers",
    [
        [],
        [SimpleNamespace(node_name="unexpected_relay", node_namespace="/")],
        [
            SimpleNamespace(node_name="safety_gate", node_namespace="/"),
            SimpleNamespace(node_name="other", node_namespace="/"),
        ],
    ],
)
def test_external_relay_rejects_missing_wrong_or_duplicate_safe_publisher(publishers):
    with pytest.raises(RuntimeError, match="safety_gate"):
        _validate_safe_publishers(publishers, external_relay=True)


def test_internal_relay_only_requires_exactly_one_safe_publisher():
    publishers = [SimpleNamespace(node_name="vgr_pi_nav2_goal_bench")]

    assert _validate_safe_publishers(publishers, external_relay=False) == 1


def test_runtime_supports_internal_and_external_safe_relays():
    source = Path("ros2_ws/src/vgr_runtime/vgr_runtime/cli/pi_nav2_goal_bench.py").read_text(encoding="utf-8")

    assert 'ActionClient(node, NavigateToPose, "/navigate_to_pose")' in source
    assert '"/cmd_vel_nav"' in source
    assert '"/cmd_vel_safe"' in source
    assert '"/odom"' in source
    assert '"/hardware/status"' in source
    assert '"/plan"' in source
    assert "cancel_goal_async" in source
    assert "get_publishers_info_by_topic" in source
    assert 'goal.pose.header.frame_id = "map"' in source
    assert "goal.pose.pose.position.x = 0.10" in source
    assert '"/aruco/pose"' not in source
    assert "external_relay" in source


def test_cli_rejects_any_broader_envelope_before_ros(tmp_path, monkeypatch):
    called = False

    def unexpected_run():
        nonlocal called
        called = True

    monkeypatch.setattr(goal_bench, "run_ros_goal", unexpected_run)
    report = tmp_path / "report.json"

    exit_code = goal_bench.main([
        "--goal-x", "0.11",
        "--wheels-raised", "YES",
        "--report", str(report),
    ])

    assert exit_code == 1
    assert called is False
    assert json.loads(report.read_text(encoding="utf-8"))["pass"] is False


def test_cli_forwards_external_relay_without_changing_legacy_default(tmp_path, monkeypatch):
    calls = []

    def fake_run(*, external_relay=False):
        calls.append(external_relay)
        return {"pass": True}

    monkeypatch.setattr(goal_bench, "run_ros_goal", fake_run)

    external_report = tmp_path / "external.json"
    legacy_report = tmp_path / "legacy.json"
    assert goal_bench.main([
        "--external-relay",
        "--wheels-raised", "YES",
        "--report", str(external_report),
    ]) == 0
    assert goal_bench.main([
        "--wheels-raised", "YES",
        "--report", str(legacy_report),
    ]) == 0

    assert calls == [True, False]


def test_runner_has_bounded_goal10cm_safety_gate_mode():
    source = Path("scripts/run_pi_nav2_bench.sh").read_text(encoding="utf-8")

    assert "goal10cm_gate)" in source
    assert "goal10cm_gate requires VGR_WHEELS_RAISED=YES" in source
    assert "require_fresh_pass stationary" in source
    assert "require_fresh_pass nav2" in source
    assert "start_hardware true 120" in source
    assert "bench_pseudo_pose" in source
    assert "safety_gate_node" in source
    assert "max_v_mps:=0.03" in source
    assert "max_omega_rad_s:=0.25" in source
    assert "nav_timeout_s:=0.2" in source
    assert "[-2.5,-2.5, 2.5,-2.5, 2.5,2.5, -2.5,2.5]" in source
    assert "goal10cm_gate_safety_status.log" in source
    assert "--external-relay" in source
    assert 'CLEANUP_TOPIC="/cmd_vel_nav"' in source
