"""Deterministic NavigateToPose runner and machine-readable acceptance verdict."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time

from vgr_core.geometry import ARENA_BOUNDS, NAV_OBSTACLE
from nav2_integration.ros_helpers import quaternion_from_yaw, yaw_from_quaternion
from vgr_core.geometry.arena_geometry import box_distance_to_point
from vgr_core.motion import wrap_angle


MAX_POSITION_ERROR_M = 0.12
MAX_YAW_ERROR_RAD = 0.25
MIN_CLEARANCE_M = 0.05
ROBOT_HALF_LENGTH_M = 0.20
ROBOT_HALF_WIDTH_M = 0.11
ROBOT_RADIUS_M = 0.23
MAX_START_POSITION_ERROR_M = 0.15


@dataclass(frozen=True)
class TraceSummary:
    action_status: str
    final_position_error_m: float
    final_yaw_error_rad: float
    min_clearance_m: float
    detour_side: str | None
    nav_cmd_count: int
    safe_cmd_count: int
    plan_count: int


def evaluate(trace: TraceSummary) -> dict[str, object]:
    reasons: list[str] = []
    if trace.action_status != "SUCCEEDED":
        reasons.append(f"action status is {trace.action_status}, expected SUCCEEDED")
    if trace.final_position_error_m > MAX_POSITION_ERROR_M:
        reasons.append(
            f"position error {trace.final_position_error_m:.3f} m exceeds {MAX_POSITION_ERROR_M:.3f} m"
        )
    if trace.final_yaw_error_rad > MAX_YAW_ERROR_RAD:
        reasons.append(
            f"yaw error {trace.final_yaw_error_rad:.3f} rad exceeds {MAX_YAW_ERROR_RAD:.3f} rad"
        )
    if trace.min_clearance_m < MIN_CLEARANCE_M:
        reasons.append(
            f"clearance {trace.min_clearance_m:.3f} m is below {MIN_CLEARANCE_M:.3f} m"
        )
    if trace.detour_side not in ("north", "south"):
        reasons.append("detour around the fixed obstacle was not observed")
    if trace.nav_cmd_count <= 0:
        reasons.append("no /cmd_vel_nav samples were recorded")
    if trace.safe_cmd_count <= 0:
        reasons.append("no /cmd_vel_safe samples were recorded")
    if trace.plan_count <= 0:
        reasons.append("no /plan samples were recorded")
    return {
        "pass": not reasons,
        "reasons": reasons,
        "metrics": asdict(trace),
        "thresholds": {
            "max_position_error_m": MAX_POSITION_ERROR_M,
            "max_yaw_error_rad": MAX_YAW_ERROR_RAD,
            "min_clearance_m": MIN_CLEARANCE_M,
        },
    }


def summarize_localization_errors(errors: list[tuple[float, float]]) -> dict[str, object]:
    if not errors:
        return {
            "count": 0,
            "mean_position_error_m": None,
            "max_position_error_m": None,
            "mean_yaw_error_rad": None,
            "max_yaw_error_rad": None,
        }
    return {
        "count": len(errors),
        "mean_position_error_m": sum(item[0] for item in errors) / len(errors),
        "max_position_error_m": max(item[0] for item in errors),
        "mean_yaw_error_rad": sum(item[1] for item in errors) / len(errors),
        "max_yaw_error_rad": max(item[1] for item in errors),
    }


def _start_pose_error(
    pose: tuple[float, float, float], expected_x: float, expected_y: float
) -> float:
    return math.hypot(pose[0] - expected_x, pose[1] - expected_y)


def _summarize_plan(points: list[tuple[float, float]]) -> dict[str, object]:
    if not points:
        return {
            "pose_count": 0,
            "max_abs_y_m": 0.0,
            "min_obstacle_center_distance_m": math.inf,
            "crosses_obstacle_envelope": False,
        }
    min_distance = min(
        box_distance_to_point(NAV_OBSTACLE, x, y) for x, y in points)
    return {
        "pose_count": len(points),
        "max_abs_y_m": max(abs(y) for _, y in points),
        "min_obstacle_center_distance_m": min_distance,
        "crosses_obstacle_envelope": (
            min_distance <= ROBOT_RADIUS_M + MIN_CLEARANCE_M),
    }


def _footprint(x: float, y: float, yaw: float) -> tuple[tuple[float, float], ...]:
    c, s = math.cos(yaw), math.sin(yaw)
    corners = (
        (ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M),
        (ROBOT_HALF_LENGTH_M, -ROBOT_HALF_WIDTH_M),
        (-ROBOT_HALF_LENGTH_M, -ROBOT_HALF_WIDTH_M),
        (-ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M),
    )
    return tuple((x + c * px - s * py, y + s * px + c * py) for px, py in corners)


def _edges(poly: tuple[tuple[float, float], ...]):
    return tuple(zip(poly, poly[1:] + poly[:1]))


def _cross(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a, b, c, d) -> bool:
    ab_c, ab_d = _cross(a, b, c), _cross(a, b, d)
    cd_a, cd_b = _cross(c, d, a), _cross(c, d, b)
    if ab_c * ab_d < 0.0 and cd_a * cd_b < 0.0:
        return True

    def on_segment(start, end, point) -> bool:
        return (
            min(start[0], end[0]) - 1e-12 <= point[0] <= max(start[0], end[0]) + 1e-12
            and min(start[1], end[1]) - 1e-12 <= point[1] <= max(start[1], end[1]) + 1e-12
        )

    return (
        (abs(ab_c) <= 1e-12 and on_segment(a, b, c))
        or (abs(ab_d) <= 1e-12 and on_segment(a, b, d))
        or (abs(cd_a) <= 1e-12 and on_segment(c, d, a))
        or (abs(cd_b) <= 1e-12 and on_segment(c, d, b))
    )


def _point_in_convex(point, poly) -> bool:
    values = [_cross(a, b, point) for a, b in _edges(poly)]
    return all(value >= -1e-12 for value in values) or all(value <= 1e-12 for value in values)


def _point_segment_distance(point, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length_sq))
    return math.hypot(point[0] - (a[0] + t * dx), point[1] - (a[1] + t * dy))


def _polygon_distance(first, second) -> float:
    if any(_segments_intersect(a, b, c, d) for a, b in _edges(first) for c, d in _edges(second)):
        return 0.0
    if _point_in_convex(first[0], second) or _point_in_convex(second[0], first):
        return 0.0
    return min(
        [_point_segment_distance(point, a, b) for point in first for a, b in _edges(second)]
        + [_point_segment_distance(point, a, b) for point in second for a, b in _edges(first)]
    )


def _footprint_clearance(x: float, y: float, yaw: float) -> float:
    min_x, max_x, min_y, max_y = ARENA_BOUNDS
    robot = _footprint(x, y, yaw)
    wall = min(
        point_x - min_x for point_x, _ in robot
    )
    wall = min(wall, *(max_x - point_x for point_x, _ in robot))
    wall = min(wall, *(point_y - min_y for _, point_y in robot))
    wall = min(wall, *(max_y - point_y for _, point_y in robot))
    ob_min_x, ob_max_x, ob_min_y, ob_max_y = NAV_OBSTACLE.bounds
    obstacle_poly = (
        (ob_min_x, ob_min_y), (ob_max_x, ob_min_y),
        (ob_max_x, ob_max_y), (ob_min_x, ob_max_y),
    )
    obstacle = _polygon_distance(robot, obstacle_poly)
    return min(wall, obstacle)


def _detour_side(samples: list[tuple[float, float, float]]) -> str | None:
    min_x, max_x, min_y, max_y = NAV_OBSTACLE.bounds
    for x, y, _yaw in samples:
        if min_x <= x <= max_x:
            if y > max_y:
                return "north"
            if y < min_y:
                return "south"
    return None


def _atomic_json(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def run_ros(args: argparse.Namespace) -> dict[str, object]:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("nav2_acceptance")
    poses: list[tuple[float, float, float]] = []
    localization_errors: list[tuple[float, float]] = []
    counts = {"nav": 0, "safe": 0, "plan": 0, "status": 0}
    plan_summaries: list[dict[str, object]] = []
    first_obstacle_detection: dict[str, object] | None = None
    goal_sent_at: float | None = None

    def on_pose(msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        poses.append((msg.pose.pose.position.x, msg.pose.pose.position.y,
                      yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def on_aruco(msg: PoseStamped) -> None:
        if not poses:
            return
        true_x, true_y, true_yaw = poses[-1]
        q = msg.pose.orientation
        estimated_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        localization_errors.append((
            math.hypot(msg.pose.position.x - true_x, msg.pose.position.y - true_y),
            abs(wrap_angle(estimated_yaw - true_yaw)),
        ))

    def on_plan(msg: NavPath) -> None:
        counts["plan"] += 1
        summary = _summarize_plan([
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ])
        if goal_sent_at is not None:
            summary["after_goal_s"] = time.monotonic() - goal_sent_at
        if poses:
            summary["robot_pose"] = {"x": poses[-1][0], "y": poses[-1][1]}
        plan_summaries.append(summary)

    def on_obstacles(msg: String) -> None:
        nonlocal first_obstacle_detection
        if first_obstacle_detection is not None:
            return
        try:
            obstacles = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not obstacles:
            return
        first_obstacle_detection = {}
        if goal_sent_at is not None:
            first_obstacle_detection["after_goal_s"] = (
                time.monotonic() - goal_sent_at)
        if poses:
            first_obstacle_detection["robot_pose"] = {
                "x": poses[-1][0], "y": poses[-1][1]}

    node.create_subscription(Odometry, "/sim/true_pose_raw", on_pose, 20)
    node.create_subscription(PoseStamped, "/aruco/pose", on_aruco, 10)
    node.create_subscription(Twist, "/cmd_vel_nav",
                             lambda _msg: counts.__setitem__("nav", counts["nav"] + 1), 20)
    node.create_subscription(Twist, "/cmd_vel_safe",
                             lambda _msg: counts.__setitem__("safe", counts["safe"] + 1), 20)
    node.create_subscription(NavPath, "/plan", on_plan, 10)
    node.create_subscription(String, "/obstacles_measured", on_obstacles, 10)
    node.create_subscription(String, "/safety_gate/status",
                             lambda _msg: counts.__setitem__("status", counts["status"] + 1), 10)
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    deadline = time.monotonic() + args.timeout_s
    while not client.wait_for_server(timeout_sec=1.0):
        if time.monotonic() >= deadline:
            raise TimeoutError("/navigate_to_pose action server did not become ready")

    pose_deadline = min(deadline, time.monotonic() + 3.0)
    while not poses and time.monotonic() < pose_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not poses:
        raise TimeoutError("no /sim/true_pose_raw sample before goal dispatch")
    start_error = _start_pose_error(poses[-1], args.start_x, args.start_y)
    if start_error > MAX_START_POSITION_ERROR_M:
        raise RuntimeError(
            "Gazebo spawn drifted before goal dispatch: "
            f"expected ({args.start_x:.2f}, {args.start_y:.2f}), "
            f"observed ({poses[-1][0]:.2f}, {poses[-1][1]:.2f}), "
            f"error {start_error:.3f} m"
        )

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = args.goal_x
    goal.pose.pose.position.y = args.goal_y
    qx, qy, qz, qw = quaternion_from_yaw(args.goal_yaw)
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw
    goal_sent_at = time.monotonic()
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=max(0.1, deadline - time.monotonic()))
    handle = send_future.result()
    if handle is None or not handle.accepted:
        action_status = "REJECTED"
    else:
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            node, result_future, timeout_sec=max(0.1, deadline - time.monotonic())
        )
        wrapped = result_future.result()
        if wrapped is None:
            action_status = "TIMEOUT"
            handle.cancel_goal_async()
        else:
            names = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }
            action_status = names.get(wrapped.status, f"STATUS_{wrapped.status}")

    # Drain callbacks that arrived with the final action result.
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    if poses:
        final_x, final_y, final_yaw = poses[-1]
        position_error = math.hypot(final_x - args.goal_x, final_y - args.goal_y)
        yaw_error = abs(wrap_angle(final_yaw - args.goal_yaw))
        min_clearance = min(_footprint_clearance(x, y, yaw) for x, y, yaw in poses)
    else:
        position_error = yaw_error = math.inf
        min_clearance = -math.inf
    summary = TraceSummary(
        action_status=action_status,
        final_position_error_m=position_error,
        final_yaw_error_rad=yaw_error,
        min_clearance_m=min_clearance,
        detour_side=_detour_side(poses),
        nav_cmd_count=counts["nav"],
        safe_cmd_count=counts["safe"],
        plan_count=counts["plan"],
    )
    report = evaluate(summary)
    report["goal"] = {"x": args.goal_x, "y": args.goal_y, "yaw": args.goal_yaw}
    report["pose_sample_count"] = len(poses)
    report["safety_status_count"] = counts["status"]
    report["aruco_localization"] = summarize_localization_errors(localization_errors)
    first_detour = next(
        (item for item in plan_summaries
         if not item["crosses_obstacle_envelope"]),
        None,
    )
    report["obstacle_detection"] = first_obstacle_detection
    report["plan_history"] = {
        "count": len(plan_summaries),
        "first": plan_summaries[0] if plan_summaries else None,
        "first_detour": first_detour,
        "last": plan_summaries[-1] if plan_summaries else None,
    }
    node.destroy_node()
    rclpy.shutdown()
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-x", type=float, default=0.7)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=3.5)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = run_ros(args)
    except Exception as exc:  # noqa: BLE001 - a runtime gate must persist the failure.
        report = {"pass": False, "reasons": [f"runtime exception: {exc}"], "metrics": {}}
    _atomic_json(args.report, report)
    metrics = report.get("metrics", {})
    print(f"NAV2_METRICS pass={report['pass']} metrics={json.dumps(metrics, sort_keys=True)}")
    for reason in report.get("reasons", []):
        print(f"NAV2_REASON {reason}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
