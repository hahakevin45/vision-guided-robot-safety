"""Bounded raised-wheel one-metre NavigateToPose performance harness."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Sequence

from vgr_runtime.cli.pi_nav2_goal_bench import CommandLimiter, atomic_json


@dataclass(frozen=True)
class OneMeterEvidence:
    goal_count: int
    goal_accepted: bool
    action_status: str
    initial_pose: tuple[float, float, float]
    final_pose: tuple[float, float, float]
    raw_encoder_delta: tuple[int, int]
    nav_cmd_count: int
    safe_cmd_count: int
    plan_count: int
    max_nav_linear_mps: float
    max_nav_angular_rad_s: float
    max_safe_linear_mps: float
    max_safe_angular_rad_s: float
    max_abs_target_cps: int
    final_targets: tuple[int, int]
    zero_target_observation_s: float
    hardware_faults: tuple[str, ...]
    safe_publisher_count: int
    clamp_count: int
    stale_count: int
    goal_elapsed_s: float


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def evaluate_one_meter(evidence: OneMeterEvidence) -> dict[str, object]:
    delta_x = evidence.final_pose[0] - evidence.initial_pose[0]
    delta_y = evidence.final_pose[1] - evidence.initial_pose[1]
    delta_yaw = _wrap_angle(evidence.final_pose[2] - evidence.initial_pose[2])
    reasons: list[str] = []
    if evidence.goal_count != 1:
        reasons.append("exactly one goal was not sent")
    if not evidence.goal_accepted:
        reasons.append("goal was not accepted")
    if evidence.action_status != "SUCCEEDED":
        reasons.append(
            f"action status is {evidence.action_status}, expected SUCCEEDED"
        )
    if evidence.goal_elapsed_s > 12.0:
        reasons.append("goal exceeded the 12 second deadline")
    if not 0.95 <= delta_x <= 1.02:
        reasons.append(f"odometry x delta {delta_x:.4f} m is outside [0.95, 1.02]")
    if abs(delta_y) > 0.08:
        reasons.append(f"lateral drift {delta_y:.4f} m exceeds 0.08 m")
    if abs(delta_yaw) > 0.25:
        reasons.append(f"yaw error {delta_yaw:.4f} rad exceeds 0.25 rad")
    if evidence.raw_encoder_delta[0] <= 0 or evidence.raw_encoder_delta[1] <= 0:
        reasons.append("encoder direction is not forward on both wheels")
    if evidence.nav_cmd_count <= 0:
        reasons.append("no /cmd_vel_nav command was observed")
    if evidence.safe_cmd_count <= 0:
        reasons.append("no /cmd_vel_safe command was relayed")
    if evidence.plan_count <= 0:
        reasons.append("no /plan was observed")
    if evidence.max_safe_linear_mps > 0.20:
        reasons.append("relayed linear command exceeded 0.20 m/s")
    if evidence.max_safe_angular_rad_s > 0.25:
        reasons.append("relayed angular command exceeded 0.25 rad/s")
    if evidence.max_abs_target_cps > 900:
        reasons.append("hardware target exceeded 900 counts/s")
    if evidence.final_targets != (0, 0):
        reasons.append("final targets are nonzero")
    if evidence.zero_target_observation_s < 2.0:
        reasons.append("zero-target observation is shorter than 2 seconds")
    if evidence.hardware_faults:
        reasons.append("hardware fault was reported: " + "; ".join(evidence.hardware_faults))
    if evidence.safe_publisher_count != 1:
        reasons.append(
            f"/cmd_vel_safe publisher count is {evidence.safe_publisher_count}, expected 1"
        )
    closed_loop_pass = not reasons
    performance_target_met = evidence.goal_elapsed_s <= 7.0
    all_reasons = list(reasons)
    if not performance_target_met:
        all_reasons.append("nominal seven second performance target was not met")
    metrics = asdict(evidence)
    metrics.update({
        "delta_x_m": delta_x,
        "delta_y_m": delta_y,
        "delta_yaw_rad": delta_yaw,
        "normalized_encoder_delta": list(evidence.raw_encoder_delta),
        "final_targets": list(evidence.final_targets),
    })
    return {
        "pass": closed_loop_pass and performance_target_met,
        "closed_loop_pass": closed_loop_pass,
        "performance_target_met": performance_target_met,
        "reasons": all_reasons,
        "metrics": metrics,
        "thresholds": {
            "min_delta_x_m": 0.95,
            "max_delta_x_m": 1.02,
            "max_abs_delta_y_m": 0.08,
            "max_abs_delta_yaw_rad": 0.25,
            "max_linear_mps": 0.20,
            "max_angular_rad_s": 0.25,
            "max_abs_target_cps": 900,
            "min_zero_target_observation_s": 2.0,
            "action_deadline_s": 12.0,
            "performance_target_s": 7.0,
        },
    }


def _yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def require_raised_confirmation(value: str) -> None:
    if value != "YES":
        raise ValueError("speed1m requires VGR_WHEELS_RAISED=YES")


def run_ros_goal() -> dict[str, object]:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String

    limiter = CommandLimiter(0.20, 0.25, 0.20)

    class OneMeterNode(Node):
        def __init__(self) -> None:
            super().__init__("vgr_pi_nav2_1m_bench")
            self.latest_nav: tuple[float, float, float] | None = None
            self.latest_pose: tuple[float, float, float] | None = None
            self.latest_status: dict[str, object] | None = None
            self.latest_status_stamp_s: float | None = None
            self.last_nonzero_target_s: float | None = None
            self.relay_enabled = True
            self.nav_cmd_count = 0
            self.safe_cmd_count = 0
            self.plan_count = 0
            self.status_count = 0
            self.max_nav_linear_mps = 0.0
            self.max_nav_angular_rad_s = 0.0
            self.max_safe_linear_mps = 0.0
            self.max_safe_angular_rad_s = 0.0
            self.max_abs_target_cps = 0
            self.clamp_count = 0
            self.stale_count = 0
            self.hardware_faults: list[str] = []
            self.safe_publisher = self.create_publisher(
                Twist,
                "/cmd_vel_safe",
                10,
            )
            self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav, 20)
            self.create_subscription(Odometry, "/odom", self._on_odom, 20)
            self.create_subscription(
                String,
                "/hardware/status",
                self._on_status,
                20,
            )
            self.create_subscription(RosPath, "/plan", self._on_plan, 10)
            self.create_timer(0.05, self._on_relay_timer)

        def _on_nav(self, msg: Twist) -> None:
            now_s = time.monotonic()
            linear = float(msg.linear.x)
            angular = float(msg.angular.z)
            self.latest_nav = (linear, angular, now_s)
            self.nav_cmd_count += 1
            self.max_nav_linear_mps = max(self.max_nav_linear_mps, abs(linear))
            self.max_nav_angular_rad_s = max(
                self.max_nav_angular_rad_s,
                abs(angular),
            )

        def _on_odom(self, msg: Odometry) -> None:
            self.latest_pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                _yaw_from_quaternion(msg.pose.pose.orientation),
            )

        def _on_status(self, msg: String) -> None:
            now_s = time.monotonic()
            try:
                status = json.loads(msg.data)
            except (TypeError, json.JSONDecodeError):
                fault = "invalid hardware status JSON"
                if fault not in self.hardware_faults:
                    self.hardware_faults.append(fault)
                return
            self.latest_status = status
            self.latest_status_stamp_s = now_s
            self.status_count += 1
            left_target = int(status.get("left_target_cps", 0))
            right_target = int(status.get("right_target_cps", 0))
            self.max_abs_target_cps = max(
                self.max_abs_target_cps,
                abs(left_target),
                abs(right_target),
            )
            if left_target or right_target:
                self.last_nonzero_target_s = now_s
            fault = status.get("fault")
            if fault and str(fault) not in self.hardware_faults:
                self.hardware_faults.append(str(fault))

        def _on_plan(self, _msg: RosPath) -> None:
            self.plan_count += 1

        def _publish(self, linear: float, angular: float) -> None:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.safe_publisher.publish(msg)
            self.safe_cmd_count += 1
            self.max_safe_linear_mps = max(
                self.max_safe_linear_mps,
                abs(linear),
            )
            self.max_safe_angular_rad_s = max(
                self.max_safe_angular_rad_s,
                abs(angular),
            )

        def publish_zero(self) -> None:
            self._publish(0.0, 0.0)

        def _on_relay_timer(self) -> None:
            now_s = time.monotonic()
            if not self.relay_enabled or self.latest_nav is None:
                self.publish_zero()
                return
            linear, angular, stamp_s = self.latest_nav
            command = limiter.limit(linear, angular, stamp_s, now_s)
            if command.was_clamped:
                self.clamp_count += 1
            if command.was_stale:
                self.stale_count += 1
            self._publish(command.linear_x, command.angular_z)

    def spin_until(predicate, deadline_s: float) -> bool:
        while rclpy.ok() and time.monotonic() < deadline_s:
            if predicate():
                return True
            rclpy.spin_once(node, timeout_sec=0.05)
        return bool(predicate())

    def require_preconditions() -> None:
        ready = spin_until(
            lambda: node.latest_pose is not None
            and node.latest_status is not None,
            time.monotonic() + 5.0,
        )
        if not ready:
            raise RuntimeError("timed out waiting for /odom and /hardware/status")
        assert node.latest_pose is not None
        assert node.latest_status is not None
        x, y, yaw = node.latest_pose
        if math.hypot(x, y) > 0.01 or abs(yaw) > 0.05:
            raise RuntimeError(
                f"initial odometry is not centered: x={x:.4f} y={y:.4f} yaw={yaw:.4f}"
            )
        if not bool(node.latest_status.get("allow_motion")):
            raise RuntimeError("hardware bridge does not allow motion")
        initial_targets = (
            int(node.latest_status.get("left_target_cps", 0)),
            int(node.latest_status.get("right_target_cps", 0)),
        )
        if initial_targets != (0, 0):
            raise RuntimeError(f"initial hardware targets are {initial_targets}")
        if node.hardware_faults:
            raise RuntimeError(
                "hardware fault before goal: " + "; ".join(node.hardware_faults)
            )

    rclpy.init()
    node = OneMeterNode()
    action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    goal_handle = None
    goal_count = 0
    goal_accepted = False
    action_status = "NOT_SENT"
    goal_elapsed_s = 0.0
    initial_pose = (0.0, 0.0, 0.0)
    initial_raw = (0, 0)
    safe_publisher_count = 0
    try:
        require_preconditions()
        if not action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/navigate_to_pose action server is unavailable")
        spin_until(
            lambda: len(node.get_publishers_info_by_topic("/cmd_vel_safe")) >= 1,
            time.monotonic() + 3.0,
        )
        safe_publisher_count = len(
            node.get_publishers_info_by_topic("/cmd_vel_safe")
        )
        if safe_publisher_count != 1:
            raise RuntimeError(
                f"/cmd_vel_safe publisher count is {safe_publisher_count}, expected 1"
            )
        assert node.latest_pose is not None
        assert node.latest_status is not None
        initial_pose = node.latest_pose
        initial_raw = (
            int(node.latest_status.get("raw_left", 0)),
            int(node.latest_status.get("raw_right", 0)),
        )

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.00
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0
        goal_count += 1
        started_s = time.monotonic()
        send_future = action.send_goal_async(goal)
        if not spin_until(send_future.done, started_s + 5.0):
            action_status = "SEND_TIMEOUT"
        else:
            goal_handle = send_future.result()
            goal_accepted = bool(goal_handle and goal_handle.accepted)
            if not goal_accepted:
                action_status = "REJECTED"
            else:
                result_future = goal_handle.get_result_async()
                if spin_until(result_future.done, started_s + 12.0):
                    status = result_future.result().status
                    statuses = {
                        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                        GoalStatus.STATUS_ABORTED: "ABORTED",
                        GoalStatus.STATUS_CANCELED: "CANCELED",
                    }
                    action_status = statuses.get(status, f"STATUS_{status}")
                else:
                    action_status = "TIMEOUT"
                    cancel_future = goal_handle.cancel_goal_async()
                    spin_until(cancel_future.done, time.monotonic() + 2.0)
        goal_elapsed_s = time.monotonic() - started_s
        node.relay_enabled = False
        node.latest_nav = None
        stop_started_s = time.monotonic()
        spin_until(lambda: False, stop_started_s + 2.20)

        if node.latest_pose is None or node.latest_status is None:
            raise RuntimeError("odometry or hardware status disappeared")
        final_targets = (
            int(node.latest_status.get("left_target_cps", 0)),
            int(node.latest_status.get("right_target_cps", 0)),
        )
        latest_raw = (
            int(node.latest_status.get("raw_left", 0)),
            int(node.latest_status.get("raw_right", 0)),
        )
        zero_start_s = max(
            stop_started_s,
            node.last_nonzero_target_s or stop_started_s,
        )
        zero_observation_s = max(
            0.0,
            (node.latest_status_stamp_s or zero_start_s) - zero_start_s,
        )
        evidence = OneMeterEvidence(
            goal_count=goal_count,
            goal_accepted=goal_accepted,
            action_status=action_status,
            initial_pose=initial_pose,
            final_pose=node.latest_pose,
            raw_encoder_delta=(
                latest_raw[0] - initial_raw[0],
                latest_raw[1] - initial_raw[1],
            ),
            nav_cmd_count=node.nav_cmd_count,
            safe_cmd_count=node.safe_cmd_count,
            plan_count=node.plan_count,
            max_nav_linear_mps=node.max_nav_linear_mps,
            max_nav_angular_rad_s=node.max_nav_angular_rad_s,
            max_safe_linear_mps=node.max_safe_linear_mps,
            max_safe_angular_rad_s=node.max_safe_angular_rad_s,
            max_abs_target_cps=node.max_abs_target_cps,
            final_targets=final_targets,
            zero_target_observation_s=zero_observation_s,
            hardware_faults=tuple(node.hardware_faults),
            safe_publisher_count=safe_publisher_count,
            clamp_count=node.clamp_count,
            stale_count=node.stale_count,
            goal_elapsed_s=goal_elapsed_s,
        )
        report = evaluate_one_meter(evidence)
        report.update({
            "mode": "goal1m",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "status_sample_count": node.status_count,
        })
        return report
    finally:
        node.relay_enabled = False
        try:
            if goal_handle is not None and action_status not in {
                "SUCCEEDED",
                "ABORTED",
                "CANCELED",
            }:
                cancel_future = goal_handle.cancel_goal_async()
                spin_until(cancel_future.done, time.monotonic() + 1.0)
        finally:
            for _ in range(10):
                node.publish_zero()
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.shutdown()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, default=1.00)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--max-linear-mps", type=float, default=0.20)
    parser.add_argument("--max-angular-rad-s", type=float, default=0.25)
    parser.add_argument("--stale-s", type=float, default=0.20)
    parser.add_argument("--wheels-raised", required=True)
    parser.add_argument("--report", type=Path, required=True)
    return parser


def _require_fixed_envelope(args: argparse.Namespace) -> None:
    requested = (
        args.goal_x,
        args.goal_y,
        args.goal_yaw,
        args.timeout_s,
        args.max_linear_mps,
        args.max_angular_rad_s,
        args.stale_s,
    )
    approved = (1.00, 0.0, 0.0, 12.0, 0.20, 0.25, 0.20)
    if requested != approved:
        raise ValueError(
            "goal and command envelope are fixed at "
            "x=1.00 y=0 yaw=0 timeout=12 linear=0.20 angular=0.25 stale=0.20"
        )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        require_raised_confirmation(args.wheels_raised)
        _require_fixed_envelope(args)
        report = run_ros_goal()
    except Exception as exc:
        report = {
            "mode": "goal1m",
            "pass": False,
            "reasons": [f"{type(exc).__name__}: {exc}"],
            "generated_at": datetime.now(timezone.utc).isoformat(),
        }
    atomic_json(args.report, report)
    passed = bool(report.get("pass"))
    print("PI_NAV2_1M_PASS" if passed else "PI_NAV2_1M_FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
