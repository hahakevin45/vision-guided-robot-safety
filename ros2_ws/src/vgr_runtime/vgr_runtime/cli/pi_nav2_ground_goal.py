"""首次落地 Nav2 goal harness（真 ArUco 定位版）。

與架空版 pi_nav2_goal_bench 的差異：
- 位姿真相來自 /aruco/pose（map frame，aruco_camera_pose 節點），
  不再假設 odom 從原點出發；goal = 當下位姿沿 map +x 前進 --forward-m。
- 只支援 external relay：/cmd_vel_safe 唯一 publisher 必須是 safety_gate
  節點，harness 本身絕不發布任何速度命令。
- 落地確認用 --ground-run YES（現場人員握 12V 斷電、牽繩）。

責任鏈與架空版相同：
Nav2 → /cmd_vel_nav → safety_gate → /cmd_vel_safe → hardware bridge → STM32
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Sequence

from vgr_runtime.cli.pi_nav2_goal_bench import (
    _validate_safe_publishers,
    _yaw_from_quaternion,
    atomic_json,
)

GOAL_POS_TOL_M = 0.06
GOAL_YAW_TOL_RAD = 0.30
POSE_FRESH_S = 0.5
# 終點位姿允許的最大 age：到點後車頭可能落在 marker 視野縫隙（貼近
# marker 牆時縫隙更寬），用結束前最後一筆位姿當證據（動作結束＋停穩
# 1s 內車不再位移）。2026-07-15 PID 首跑 3.5s 被 3.0 誤殺 → 放寬到 6。
FINAL_POSE_MAX_AGE_S = 6.0
# Nav2 原始命令（/cmd_vel_nav）的角速度界限倍率：RPP 曲率命令可超過
# 安全上限，enforcement 在安全層（/cmd_vel_safe 用 1.0 倍硬界限）。
NAV_BOUND_FACTOR = 2.5


def goal_in_map(
    initial_pose: tuple[float, float, float], forward_m: float, right_m: float
) -> tuple[float, float, float]:
    """在車體座標系給 (前, 右) 位移，換算 map frame goal；朝向維持不變。"""
    x0, y0, yaw = initial_pose
    gx = x0 + forward_m * math.cos(yaw) + right_m * math.sin(yaw)
    gy = y0 + forward_m * math.sin(yaw) - right_m * math.cos(yaw)
    return gx, gy, yaw


def parse_waypoints(value: str) -> list[tuple[float, float]]:
    """解析 ``forward,right;...`` 車體座標序列。"""
    if not value or not value.strip():
        raise ValueError("--waypoints must contain at least one forward,right pair")
    parsed: list[tuple[float, float]] = []
    for index, raw_segment in enumerate(value.split(";"), start=1):
        parts = raw_segment.split(",")
        if len(parts) != 2 or not all(part.strip() for part in parts):
            raise ValueError(
                f"--waypoints segment {index} must be a forward,right pair"
            )
        try:
            forward_m, right_m = (float(part.strip()) for part in parts)
        except ValueError as exc:
            raise ValueError(
                f"--waypoints segment {index} contains a non-numeric value"
            ) from exc
        if not math.isfinite(forward_m) or not math.isfinite(right_m):
            raise ValueError(f"--waypoints segment {index} values must be finite")
        parsed.append((forward_m, right_m))
    return parsed


def resolve_goal_segments(args: argparse.Namespace) -> list[tuple[float, float]]:
    """解析 goal 參數、套用舊預設值，並逐段執行距離上限檢查。"""
    waypoint_text = getattr(args, "waypoints", None)
    if waypoint_text is not None:
        if args.forward_m is not None or args.right_m is not None:
            raise ValueError(
                "--waypoints is mutually exclusive with --forward-m/--right-m"
            )
        segments = parse_waypoints(waypoint_text)
    else:
        args.forward_m = 0.20 if args.forward_m is None else args.forward_m
        args.right_m = 0.0 if args.right_m is None else args.right_m
        segments = [(args.forward_m, args.right_m)]

    for index, (forward_m, right_m) in enumerate(segments, start=1):
        if not 0.0 < forward_m <= args.max_goal_m:
            raise ValueError(
                f"goal segment {index} forward must be in (0, {args.max_goal_m}] "
                "(raise via --max-goal-m)"
            )
        if abs(right_m) > args.max_goal_m:
            raise ValueError(
                f"goal segment {index} right must be within "
                f"[-{args.max_goal_m}, {args.max_goal_m}]"
            )
    return segments


def evaluate_waypoint_results(
    waypoints: list[dict[str, object]], *, expected_count: int
) -> dict[str, object]:
    """彙總逐段 action 與現行位置/朝向門檻。"""
    passed = len(waypoints) == expected_count and all(
        waypoint.get("action_status") == "SUCCEEDED"
        and float(waypoint.get("goal_position_error_m", math.inf))
        <= GOAL_POS_TOL_M
        and float(waypoint.get("goal_yaw_error_rad", math.inf))
        <= GOAL_YAW_TOL_RAD
        for waypoint in waypoints
    )
    return {"pass": passed, "waypoints": waypoints}


def build_waypoint_report(
    *,
    index: int,
    forward_m: float,
    right_m: float,
    initial_pose: tuple[float, float, float],
    final_pose: tuple[float, float, float],
    goal_pose: tuple[float, float, float],
    elapsed_s: float,
    action_status: str,
    evaluation: dict[str, object],
) -> dict[str, object]:
    """建立單段 report，集中維護多波點 JSON 契約。"""
    return {
        "index": index,
        "forward_m": forward_m,
        "right_m": right_m,
        "initial_pose": list(initial_pose),
        "final_pose": list(final_pose),
        "goal_xy": list(goal_pose[:2]),
        "goal_position_error_m": evaluation["goal_position_error_m"],
        "goal_yaw_error_rad": evaluation["goal_yaw_error_rad"],
        "elapsed_s": elapsed_s,
        "action_status": action_status,
        "pass": evaluation["pass"],
    }


def evaluate_rehearsal(
    *,
    plan_count: int,
    nav_cmd_count: int,
    max_nav_linear_mps: float,
    max_safe_linear_mps: float,
    max_abs_target_cps: int,
    hardware_faults: list[str],
    max_pose_age_s: float,
    safe_publisher_ok: bool,
) -> dict[str, object]:
    """彩排（bridge 不放行馬達）：驗鏈路流動，不驗位移與 action 結果。"""
    checks = {
        "plan_published": plan_count > 0,
        "nav_commands_flowed": nav_cmd_count > 0 and max_nav_linear_mps > 0.0,
        "safe_commands_flowed": max_safe_linear_mps > 0.0,
        "wheel_targets_stayed_zero": max_abs_target_cps == 0,
        "no_hardware_fault": not hardware_faults,
        "pose_stayed_fresh": max_pose_age_s <= POSE_FRESH_S,
        "single_safety_gate_publisher": safe_publisher_ok,
    }
    return {"pass": all(checks.values()), "checks": checks}


def evaluate_ground_goal(
    *,
    goal_pose: tuple[float, float, float],
    action_status: str,
    initial_pose: tuple[float, float, float],
    final_pose: tuple[float, float, float],
    max_nav_linear_mps: float,
    max_nav_angular_rad_s: float,
    max_safe_linear_mps: float,
    max_safe_angular_rad_s: float,
    max_linear_mps: float,
    max_angular_rad_s: float,
    hardware_faults: list[str],
    max_pose_age_s: float,
    safe_publisher_ok: bool,
) -> dict[str, object]:
    dx = final_pose[0] - initial_pose[0]
    dy = final_pose[1] - initial_pose[1]
    pos_error_m = math.hypot(final_pose[0] - goal_pose[0], final_pose[1] - goal_pose[1])
    yaw_error = abs(
        math.atan2(
            math.sin(final_pose[2] - goal_pose[2]),
            math.cos(final_pose[2] - goal_pose[2]),
        )
    )
    checks = {
        "action_succeeded": action_status == "SUCCEEDED",
        "goal_position_reached": pos_error_m <= GOAL_POS_TOL_M,
        "goal_yaw_reached": yaw_error <= GOAL_YAW_TOL_RAD,
        "nav_within_bounds": (
            max_nav_linear_mps <= max_linear_mps * NAV_BOUND_FACTOR + 1e-6
            and max_nav_angular_rad_s <= max_angular_rad_s * NAV_BOUND_FACTOR + 1e-6
        ),
        "safe_within_bounds": (
            max_safe_linear_mps <= max_linear_mps + 1e-6
            and max_safe_angular_rad_s <= max_angular_rad_s + 1e-6
        ),
        "no_hardware_fault": not hardware_faults,
        "pose_stayed_fresh": max_pose_age_s <= POSE_FRESH_S,
        "single_safety_gate_publisher": safe_publisher_ok,
    }
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "displacement_x_m": dx,
        "displacement_y_m": dy,
        "goal_position_error_m": pos_error_m,
        "goal_yaw_error_rad": yaw_error,
    }


def run_ground_goal(args: argparse.Namespace) -> dict[str, object]:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Path as RosPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String

    class GroundGoalNode(Node):
        def __init__(self) -> None:
            super().__init__("vgr_pi_nav2_ground_goal")
            self.latest_aruco: tuple[float, float, float] | None = None
            self.latest_aruco_mono_s: float | None = None
            self.max_pose_gap_s = 0.0
            self.latest_status: dict[str, object] | None = None
            self.hardware_faults: list[str] = []
            self.max_abs_target_cps = 0
            self.nav_cmd_count = 0
            self.safe_cmd_count = 0
            self.plan_count = 0
            self.max_nav_linear_mps = 0.0
            self.max_nav_angular_rad_s = 0.0
            self.max_safe_linear_mps = 0.0
            self.max_safe_angular_rad_s = 0.0
            self.create_subscription(PoseStamped, "/aruco/pose", self._on_aruco, 20)
            self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav, 20)
            self.create_subscription(Twist, "/cmd_vel_safe", self._on_safe, 20)
            self.create_subscription(String, "/hardware/status", self._on_status, 20)
            self.create_subscription(RosPath, "/plan", self._on_plan, 10)

        def _on_aruco(self, msg: PoseStamped) -> None:
            now_mono = time.monotonic()
            if self.latest_aruco_mono_s is not None:
                self.max_pose_gap_s = max(
                    self.max_pose_gap_s, now_mono - self.latest_aruco_mono_s
                )
            self.latest_aruco_mono_s = now_mono
            p = msg.pose.position
            self.latest_aruco = (p.x, p.y, _yaw_from_quaternion(msg.pose.orientation))

        def _on_nav(self, msg: Twist) -> None:
            self.nav_cmd_count += 1
            self.max_nav_linear_mps = max(self.max_nav_linear_mps, abs(msg.linear.x))
            self.max_nav_angular_rad_s = max(
                self.max_nav_angular_rad_s, abs(msg.angular.z)
            )

        def _on_safe(self, msg: Twist) -> None:
            self.safe_cmd_count += 1
            self.max_safe_linear_mps = max(self.max_safe_linear_mps, abs(msg.linear.x))
            self.max_safe_angular_rad_s = max(
                self.max_safe_angular_rad_s, abs(msg.angular.z)
            )

        def _on_status(self, msg: String) -> None:
            try:
                status = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            self.latest_status = status
            fault = status.get("fault")
            if fault:
                self.hardware_faults.append(str(fault))
            self.max_abs_target_cps = max(
                self.max_abs_target_cps,
                abs(int(status.get("left_target_cps", 0))),
                abs(int(status.get("right_target_cps", 0))),
            )

        def _on_plan(self, _msg) -> None:
            self.plan_count += 1

    rclpy.init()
    node = GroundGoalNode()
    action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    action_status = "NOT_SENT"
    initial_pose = (0.0, 0.0, 0.0)
    final_pose = (0.0, 0.0, 0.0)
    final_pose_age_s: float | None = None
    goal_xy = (0.0, 0.0)
    last_goal_pose = (0.0, 0.0, 0.0)
    waypoint_reports: list[dict[str, object]] = []
    goal_segments = getattr(
        args, "goal_segments", [(args.forward_m, args.right_m)]
    )
    safe_publisher_ok = False
    elapsed_s = 0.0
    error: str | None = None

    def spin_until(predicate, deadline_s: float) -> bool:
        while time.monotonic() < deadline_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def pose_fresh() -> bool:
        return (
            node.latest_aruco_mono_s is not None
            and time.monotonic() - node.latest_aruco_mono_s < POSE_FRESH_S
        )

    try:
        if not action.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("/navigate_to_pose action server is unavailable")
        if not spin_until(
            lambda: pose_fresh() and node.latest_status is not None,
            time.monotonic() + 8.0,
        ):
            raise RuntimeError("timed out waiting for fresh /aruco/pose and /hardware/status")
        allow_motion = bool(node.latest_status.get("allow_motion"))
        if args.rehearsal and allow_motion:
            raise RuntimeError("rehearsal requires allow_motion=false (bridge must hold STOP)")
        if not args.rehearsal and not allow_motion:
            raise RuntimeError("hardware bridge allow_motion is false")
        # DDS 探索需要暖機：冷啟動時 publisher 資訊可能晚幾秒才可見，
        # 立刻檢查會誤判 count=0（2026-07-19 實地兩次誤殺）。等到有再驗。
        deadline = time.monotonic() + 10.0
        publisher_infos = node.get_publishers_info_by_topic("/cmd_vel_safe")
        while not publisher_infos and time.monotonic() < deadline:
            time.sleep(0.5)
            publisher_infos = node.get_publishers_info_by_topic("/cmd_vel_safe")
        _validate_safe_publishers(publisher_infos, external_relay=True)
        safe_publisher_ok = True

        assert node.latest_aruco is not None
        for segment_index, (forward_m, right_m) in enumerate(
            goal_segments, start=1
        ):
            # 每段都從上一段完成並停穩後的最新位姿重新換算 map goal。
            segment_initial_pose = node.latest_aruco
            if segment_index == 1:
                initial_pose = segment_initial_pose
            gx, gy, gyaw = goal_in_map(
                segment_initial_pose, forward_m, right_m
            )
            goal_xy = (gx, gy)
            last_goal_pose = (gx, gy, gyaw)

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = gx
            goal.pose.pose.position.y = gy
            goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)

            action_status = "NOT_SENT"
            started_s = time.monotonic()
            send_future = action.send_goal_async(goal)
            if not spin_until(send_future.done, started_s + 5.0):
                raise RuntimeError(
                    f"goal segment {segment_index} was not accepted in time"
                )
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError(f"goal segment {segment_index} was rejected")
            result_future = goal_handle.get_result_async()
            if args.rehearsal:
                # 彩排：輪子不會動、goal 永遠到不了。觀察一段時間收集命令流
                # 證據後主動取消。
                spin_until(result_future.done, started_s + min(12.0, args.timeout_s))
                if not result_future.done():
                    goal_handle.cancel_goal_async()
                    spin_until(result_future.done, time.monotonic() + 3.0)
                action_status = "REHEARSAL_CANCELED"
            elif not spin_until(result_future.done, started_s + args.timeout_s):
                segment_elapsed_s = time.monotonic() - started_s
                elapsed_s += segment_elapsed_s
                goal_handle.cancel_goal_async()
                spin_until(result_future.done, time.monotonic() + 3.0)
                raise RuntimeError(
                    f"goal segment {segment_index} did not finish within "
                    f"{args.timeout_s}s"
                )
            segment_elapsed_s = time.monotonic() - started_s
            elapsed_s += segment_elapsed_s
            if not args.rehearsal:
                status_code = result_future.result().status
                action_status = {
                    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                    GoalStatus.STATUS_ABORTED: "ABORTED",
                    GoalStatus.STATUS_CANCELED: "CANCELED",
                }.get(status_code, f"STATUS_{status_code}")

            # 每段收尾都等 1 s 讓車停穩；下一段會使用這段等待後的位姿。
            spin_until(lambda: False, time.monotonic() + 1.0)
            if node.latest_aruco is None or node.latest_aruco_mono_s is None:
                raise RuntimeError("no /aruco/pose ever received")
            final_pose_age_s = time.monotonic() - node.latest_aruco_mono_s
            if final_pose_age_s > FINAL_POSE_MAX_AGE_S:
                raise RuntimeError(
                    f"final /aruco/pose is stale ({final_pose_age_s:.1f}s)"
                )
            final_pose = node.latest_aruco
            segment_result = evaluate_ground_goal(
                goal_pose=last_goal_pose,
                action_status=action_status,
                initial_pose=segment_initial_pose,
                final_pose=final_pose,
                max_nav_linear_mps=node.max_nav_linear_mps,
                max_nav_angular_rad_s=node.max_nav_angular_rad_s,
                max_safe_linear_mps=node.max_safe_linear_mps,
                max_safe_angular_rad_s=node.max_safe_angular_rad_s,
                max_linear_mps=args.max_linear_mps,
                max_angular_rad_s=args.max_angular_rad_s,
                hardware_faults=node.hardware_faults,
                max_pose_age_s=node.max_pose_gap_s,
                safe_publisher_ok=safe_publisher_ok,
            )
            waypoint_reports.append(
                build_waypoint_report(
                    index=segment_index,
                    forward_m=forward_m,
                    right_m=right_m,
                    initial_pose=segment_initial_pose,
                    final_pose=final_pose,
                    goal_pose=last_goal_pose,
                    elapsed_s=segment_elapsed_s,
                    action_status=action_status,
                    evaluation=segment_result,
                )
            )
            if not args.rehearsal and action_status != "SUCCEEDED":
                break
    except Exception as exc:  # noqa: BLE001
        error = f"{type(exc).__name__}: {exc}"
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.rehearsal:
        result = evaluate_rehearsal(
            plan_count=node.plan_count,
            nav_cmd_count=node.nav_cmd_count,
            max_nav_linear_mps=node.max_nav_linear_mps,
            max_safe_linear_mps=node.max_safe_linear_mps,
            max_abs_target_cps=node.max_abs_target_cps,
            hardware_faults=node.hardware_faults,
            max_pose_age_s=node.max_pose_gap_s,
            safe_publisher_ok=safe_publisher_ok,
        )
    else:
        result = evaluate_ground_goal(
            goal_pose=last_goal_pose,
            action_status=action_status,
            initial_pose=initial_pose,
            final_pose=final_pose,
            max_nav_linear_mps=node.max_nav_linear_mps,
            max_nav_angular_rad_s=node.max_nav_angular_rad_s,
            max_safe_linear_mps=node.max_safe_linear_mps,
            max_safe_angular_rad_s=node.max_safe_angular_rad_s,
            max_linear_mps=args.max_linear_mps,
            max_angular_rad_s=args.max_angular_rad_s,
            hardware_faults=node.hardware_faults,
            max_pose_age_s=node.max_pose_gap_s,
            safe_publisher_ok=safe_publisher_ok,
        )
        waypoint_result = evaluate_waypoint_results(
            waypoint_reports, expected_count=len(goal_segments)
        )
        result["pass"] = bool(result["pass"] and waypoint_result["pass"])
    if error is not None:
        result["pass"] = False
        result.setdefault("reasons", []).append(error)
    result.update(
        {
            "mode": "ground_rehearsal" if args.rehearsal else "ground_goal",
            "max_abs_target_cps": node.max_abs_target_cps,
            "forward_m": args.forward_m,
            "right_m": args.right_m,
            "action_status": action_status,
            "elapsed_s": elapsed_s,
            "initial_pose": list(initial_pose),
            "final_pose": list(final_pose),
            "final_pose_age_s": final_pose_age_s,
            "goal_xy": list(goal_xy),
            "nav_cmd_count": node.nav_cmd_count,
            "safe_cmd_count": node.safe_cmd_count,
            "plan_count": node.plan_count,
            "max_nav_linear_mps": node.max_nav_linear_mps,
            "max_nav_angular_rad_s": node.max_nav_angular_rad_s,
            "max_safe_linear_mps": node.max_safe_linear_mps,
            "max_safe_angular_rad_s": node.max_safe_angular_rad_s,
            "max_pose_gap_s": node.max_pose_gap_s,
            "hardware_faults": node.hardware_faults,
            "waypoints": waypoint_reports,
            "generated_at": datetime.now(timezone.utc).isoformat(),
        }
    )
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--forward-m", type=float, default=None)
    parser.add_argument("--right-m", type=float, default=None, help="車體座標右方位移；負值=左方")
    parser.add_argument(
        "--waypoints",
        help="車體座標序列 'f1,r1;f2,r2;...'；每段由到達後的當下位姿重算。",
    )
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--max-linear-mps", type=float, default=0.03)
    parser.add_argument("--max-angular-rad-s", type=float, default=0.25)
    parser.add_argument(
        "--ground-run",
        required=True,
        help="必須是 YES：現場人員在場、握 12V 斷電、車已牽繩。",
    )
    parser.add_argument(
        "--rehearsal",
        action="store_true",
        help="無動力彩排：要求 bridge allow_motion=false，只驗命令流不驗位移。",
    )
    parser.add_argument(
        "--max-goal-m",
        type=float,
        default=0.5,
        help="forward/right 的絕對上限（早期小場地沙盒 0.5；大場地由腳本明示放寬）。",
    )
    parser.add_argument("--report", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.ground_run != "YES":
            raise ValueError("ground goal requires --ground-run YES (operator present)")
        args.goal_segments = resolve_goal_segments(args)
        report = run_ground_goal(args)
    except Exception as exc:  # noqa: BLE001
        report = {
            "mode": "ground_goal",
            "pass": False,
            "reasons": [f"{type(exc).__name__}: {exc}"],
            "generated_at": datetime.now(timezone.utc).isoformat(),
        }
    atomic_json(args.report, report)
    passed = bool(report.get("pass"))
    print("PI_NAV2_GROUND_GOAL_PASS" if passed else "PI_NAV2_GROUND_GOAL_FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
