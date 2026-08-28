"""Bounded raised-wheel NavigateToPose acceptance harness for the real Pi."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Sequence


@dataclass(frozen=True)
class LimitedCommand:
    linear_x: float
    angular_z: float
    was_clamped: bool
    was_stale: bool


class CommandLimiter:
    """Restrict test-only Nav2 output to the approved raised-wheel envelope."""

    def __init__(
        self,
        max_linear_mps: float,
        max_angular_rad_s: float,
        stale_s: float,
    ) -> None:
        if min(max_linear_mps, max_angular_rad_s, stale_s) <= 0.0:
            raise ValueError("command limits must be positive")
        self.max_linear_mps = max_linear_mps
        self.max_angular_rad_s = max_angular_rad_s
        self.stale_s = stale_s

    def limit(
        self,
        linear_x: float,
        angular_z: float,
        command_stamp_s: float,
        now_s: float,
    ) -> LimitedCommand:
        if now_s - command_stamp_s > self.stale_s:
            return LimitedCommand(0.0, 0.0, False, True)
        linear = min(self.max_linear_mps, max(0.0, linear_x))
        angular = min(
            self.max_angular_rad_s,
            max(-self.max_angular_rad_s, angular_z),
        )
        return LimitedCommand(
            linear_x=linear,
            angular_z=angular,
            was_clamped=linear != linear_x or angular != angular_z,
            was_stale=False,
        )


@dataclass(frozen=True)
class GoalEvidence:
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


def evaluate_goal(evidence: GoalEvidence) -> dict[str, object]:
    """Evaluate independent action, motion-limit, odometry, and STOP proof."""
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
    if evidence.goal_elapsed_s > 20.0:
        reasons.append("goal exceeded the 20 s deadline")
    if not 0.08 <= delta_x <= 0.12:
        reasons.append(f"odometry x delta {delta_x:.4f} m is outside [0.08, 0.12]")
    if abs(delta_y) > 0.03:
        reasons.append(f"lateral drift {delta_y:.4f} m exceeds 0.03 m")
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
    if evidence.max_safe_linear_mps > 0.03:
        reasons.append("relayed command exceeded the 0.03 m/s linear limit")
    if evidence.max_safe_angular_rad_s > 0.25:
        reasons.append("relayed command exceeded the 0.25 rad/s angular limit")
    if evidence.max_abs_target_cps > 120:
        reasons.append("hardware target exceeded 120 counts/s")
    if evidence.final_targets != (0, 0):
        reasons.append("final targets are nonzero")
    if evidence.zero_target_observation_s < 2.0:
        reasons.append("zero-target observation is shorter than 2 s")
    if evidence.hardware_faults:
        reasons.append("hardware fault was reported: " + "; ".join(evidence.hardware_faults))
    if evidence.safe_publisher_count != 1:
        reasons.append(
            f"/cmd_vel_safe publisher count is {evidence.safe_publisher_count}, expected 1"
        )
    metrics = asdict(evidence)
    metrics.update({
        "delta_x_m": delta_x,
        "delta_y_m": delta_y,
        "delta_yaw_rad": delta_yaw,
        "normalized_encoder_delta": list(evidence.raw_encoder_delta),
        "final_targets": list(evidence.final_targets),
    })
    return {
        "pass": not reasons,
        "reasons": reasons,
        "metrics": metrics,
        "thresholds": {
            "min_delta_x_m": 0.08,
            "max_delta_x_m": 0.12,
            "max_abs_delta_y_m": 0.03,
            "max_abs_delta_yaw_rad": 0.25,
            "max_linear_mps": 0.03,
            "max_angular_rad_s": 0.25,
            "max_abs_target_cps": 120,
            "min_zero_target_observation_s": 2.0,
            "goal_deadline_s": 20.0,
        },
    }


def atomic_json(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


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
        raise ValueError("goal10cm requires VGR_WHEELS_RAISED=YES")


def _validate_safe_publishers(
    publisher_infos: Sequence[object],
    *,
    external_relay: bool,
) -> int:
    count = len(publisher_infos)
    if external_relay:
        names = [str(getattr(info, "node_name", "")) for info in publisher_infos]
        if count != 1 or names != ["safety_gate"]:
            raise RuntimeError(
                "/cmd_vel_safe must have exactly one safety_gate publisher; "
                f"found count={count} names={names}"
            )
    elif count != 1:
        raise RuntimeError(
            f"/cmd_vel_safe publisher count is {count}, expected 1"
        )
    return count


def run_ros_goal(*, external_relay: bool = False) -> dict[str, object]:
    """Run the fixed 10 cm action while relaying only bounded commands."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path as RosPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String

    limiter = CommandLimiter(0.03, 0.25, 0.20)

    class GoalBenchNode(Node):
        def __init__(self) -> None:
            super().__init__("vgr_pi_nav2_goal_bench")
            self.external_relay = external_relay
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
            self.safe_publisher = None
            if self.external_relay:
                self.create_subscription(Twist, "/cmd_vel_safe", self._on_safe, 20)
            else:
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
            if not self.external_relay:
                self.create_timer(0.05, self._on_relay_timer)

        def _on_nav(self, msg: Twist) -> None:
            now_s = time.monotonic()
            linear = float(msg.linear.x)
            angular = float(msg.angular.z)
            self.latest_nav = (linear, angular, now_s)
            self.nav_cmd_count += 1
            self.max_nav_linear_mps = max(
                self.max_nav_linear_mps,
                abs(linear),
            )
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

        def _on_safe(self, msg: Twist) -> None:
            self._record_safe(float(msg.linear.x), float(msg.angular.z))

        def _record_safe(self, linear: float, angular: float) -> None:
            self.safe_cmd_count += 1
            self.max_safe_linear_mps = max(self.max_safe_linear_mps, abs(linear))
            self.max_safe_angular_rad_s = max(
                self.max_safe_angular_rad_s,
                abs(angular),
            )

        def _publish(self, linear: float, angular: float) -> None:
            if self.safe_publisher is None:
                raise RuntimeError("external relay mode cannot publish /cmd_vel_safe")
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.safe_publisher.publish(msg)
            self._record_safe(linear, angular)

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
            raise RuntimeError("hardware fault before goal: " + "; ".join(node.hardware_faults))

    rclpy.init()
    node = GoalBenchNode()
    action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    goal_handle = None
    goal_count = 0
    goal_accepted = False
    action_status = "NOT_SENT"
    goal_elapsed_s = 0.0
    stop_started_s = time.monotonic()
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
        publisher_infos = node.get_publishers_info_by_topic("/cmd_vel_safe")
        safe_publisher_count = _validate_safe_publishers(
            publisher_infos,
            external_relay=external_relay,
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
        goal.pose.pose.position.x = 0.10
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
                if spin_until(result_future.done, started_s + 20.0):
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
        evidence = GoalEvidence(
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
        report = evaluate_goal(evidence)
        report.update({
            "mode": "goal10cm_gate" if external_relay else "goal10cm",
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
                if not external_relay:
                    node.publish_zero()
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.shutdown()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, default=0.10)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--max-linear-mps", type=float, default=0.03)
    parser.add_argument("--max-angular-rad-s", type=float, default=0.25)
    parser.add_argument("--stale-s", type=float, default=0.20)
    parser.add_argument("--external-relay", action="store_true")
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
    approved = (0.10, 0.0, 0.0, 20.0, 0.03, 0.25, 0.20)
    if requested != approved:
        raise ValueError(
            "goal and command envelope are fixed at "
            "x=0.10 y=0 yaw=0 timeout=20 linear=0.03 angular=0.25 stale=0.20"
        )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        require_raised_confirmation(args.wheels_raised)
        _require_fixed_envelope(args)
        report = run_ros_goal(external_relay=args.external_relay)
    except Exception as exc:
        report = {
            "mode": "goal10cm_gate" if args.external_relay else "goal10cm",
            "pass": False,
            "reasons": [f"{type(exc).__name__}: {exc}"],
            "generated_at": datetime.now(timezone.utc).isoformat(),
        }
    atomic_json(args.report, report)
    passed = bool(report.get("pass"))
    print("PI_NAV2_GOAL_PASS" if passed else "PI_NAV2_GOAL_FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
