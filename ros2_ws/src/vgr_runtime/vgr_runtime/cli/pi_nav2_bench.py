"""Collect repeatable stationary and raised-wheel evidence from the Pi ROS graph."""
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
class BenchSample:
    stamp_s: float
    x_m: float
    yaw_rad: float
    raw_left: int
    raw_right: int
    left_target_cps: int
    right_target_cps: int
    fault: str | None = None


def evaluate_stationary(
    samples: Sequence[BenchSample],
    duration_s: float,
) -> dict[str, object]:
    elapsed = (
        samples[-1].stamp_s - samples[0].stamp_s
        if len(samples) > 1
        else 0.0
    )
    odom_hz = (len(samples) - 1) / elapsed if elapsed > 0.0 else 0.0
    left_drift = (
        abs(samples[-1].raw_left - samples[0].raw_left)
        if samples
        else 0
    )
    right_drift = (
        abs(samples[-1].raw_right - samples[0].raw_right)
        if samples
        else 0
    )
    reasons: list[str] = []
    if elapsed < duration_s:
        reasons.append("collection duration too short")
    if odom_hz < 18.0:
        reasons.append("odom rate below 18 Hz")
    if left_drift > 2 or right_drift > 2:
        reasons.append("stationary encoder drift above 2 counts")
    if any(s.left_target_cps or s.right_target_cps for s in samples):
        reasons.append("nonzero target during stationary gate")
    if any(s.fault for s in samples):
        reasons.append("hardware fault reported")
    return {
        "pass": not reasons,
        "reasons": reasons,
        "sample_count": len(samples),
        "duration_s": elapsed,
        "odom_hz": odom_hz,
        "left_drift_counts": left_drift,
        "right_drift_counts": right_drift,
    }


def evaluate_motion(
    samples: Sequence[BenchSample],
    command_s: float = 0.5,
) -> dict[str, object]:
    delta_x = samples[-1].x_m - samples[0].x_m if samples else 0.0
    left_forward = (
        samples[-1].raw_left - samples[0].raw_left
        if samples
        else 0
    )
    right_forward = (
        samples[-1].raw_right - samples[0].raw_right
        if samples
        else 0
    )
    final_targets = (
        [samples[-1].left_target_cps, samples[-1].right_target_cps]
        if samples
        else [0, 0]
    )
    moving_samples = [
        sample
        for sample in samples
        if sample.left_target_cps or sample.right_target_cps
    ]
    stop_observation_s = (
        samples[-1].stamp_s - moving_samples[-1].stamp_s
        if samples and moving_samples
        else 0.0
    )
    reasons: list[str] = []
    if command_s > 0.5:
        reasons.append("command duration above 0.5 s")
    if not moving_samples:
        reasons.append("no nonzero target observed")
    if delta_x <= 0.0:
        reasons.append("odometry x did not increase")
    if left_forward <= 0 or right_forward <= 0:
        reasons.append("normalized encoder direction is not forward")
    if final_targets != [0, 0]:
        reasons.append("final targets are nonzero")
    if moving_samples and stop_observation_s < 2.0:
        reasons.append("final zero-target observation shorter than 2 s")
    if any(s.fault for s in samples):
        reasons.append("hardware fault reported")
    return {
        "pass": not reasons,
        "reasons": reasons,
        "sample_count": len(samples),
        "delta_x_m": delta_x,
        "normalized_encoder_delta": [left_forward, right_forward],
        "command_s": command_s,
        "stop_observation_s": stop_observation_s,
        "final_targets": final_targets,
    }


def evaluate_turn(
    samples: Sequence[BenchSample],
    command_s: float = 0.5,
) -> dict[str, object]:
    delta_yaw = (
        _wrap_angle(samples[-1].yaw_rad - samples[0].yaw_rad)
        if samples
        else 0.0
    )
    left_delta = (
        samples[-1].raw_left - samples[0].raw_left if samples else 0
    )
    right_delta = (
        samples[-1].raw_right - samples[0].raw_right if samples else 0
    )
    final_targets = (
        [samples[-1].left_target_cps, samples[-1].right_target_cps]
        if samples
        else [0, 0]
    )
    moving_samples = [
        sample
        for sample in samples
        if sample.left_target_cps or sample.right_target_cps
    ]
    left_turn_samples = [
        sample
        for sample in moving_samples
        if sample.left_target_cps < 0 < sample.right_target_cps
    ]
    stop_observation_s = (
        samples[-1].stamp_s - moving_samples[-1].stamp_s
        if samples and moving_samples
        else 0.0
    )
    reasons: list[str] = []
    if command_s > 0.5:
        reasons.append("turn command duration above 0.5 s")
    if not left_turn_samples:
        reasons.append("no left-turn target observed")
    if delta_yaw <= 0.0:
        reasons.append("yaw did not increase")
    if left_delta >= 0 or right_delta <= 0:
        reasons.append("differential encoder direction is not left turn")
    if final_targets != [0, 0]:
        reasons.append("final turn targets are nonzero")
    if moving_samples and stop_observation_s < 2.0:
        reasons.append("final turn zero-target observation shorter than 2 s")
    if any(sample.fault for sample in samples):
        reasons.append("hardware fault reported during turn")
    return {
        "pass": not reasons,
        "reasons": reasons,
        "sample_count": len(samples),
        "delta_yaw_rad": delta_yaw,
        "differential_encoder_delta": [left_delta, right_delta],
        "command_s": command_s,
        "stop_observation_s": stop_observation_s,
        "final_targets": final_targets,
    }


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


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
        raise ValueError("motion requires VGR_WHEELS_RAISED=YES")


def _write_report(path: Path, report: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _collect(
    *,
    mode: str,
    duration_s: float,
    command_s: float,
    linear_mps: float,
    angular_rad_s: float,
) -> list[BenchSample]:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("vgr_pi_nav2_bench")
            self.samples: list[BenchSample] = []
            self.latest_status: dict[str, object] | None = None
            self.create_subscription(
                String,
                "/hardware/status",
                self._on_status,
                50,
            )
            self.create_subscription(Odometry, "/odom", self._on_odom, 50)
            self.publisher = self.create_publisher(Twist, "/cmd_vel_safe", 10)

        def _on_status(self, msg: String) -> None:
            try:
                status = json.loads(msg.data)
            except (TypeError, json.JSONDecodeError):
                self.latest_status = {"fault": "invalid hardware status JSON"}
                return
            self.latest_status = status

        def _on_odom(self, msg: Odometry) -> None:
            if self.latest_status is None:
                return
            status = self.latest_status
            self.samples.append(BenchSample(
                stamp_s=time.monotonic(),
                x_m=float(msg.pose.pose.position.x),
                yaw_rad=_yaw_from_quaternion(msg.pose.pose.orientation),
                raw_left=int(status.get("raw_left", 0)),
                raw_right=int(status.get("raw_right", 0)),
                left_target_cps=int(status.get("left_target_cps", 0)),
                right_target_cps=int(status.get("right_target_cps", 0)),
                fault=status.get("fault"),
            ))

        def publish(self, linear_x: float, angular_z: float = 0.0) -> None:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.publisher.publish(msg)

    def spin_for(node: Collector, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    rclpy.init()
    node = Collector()
    try:
        spin_for(node, 0.50)
        if mode == "stationary":
            spin_for(node, duration_s + 0.25)
        else:
            deadline = time.monotonic() + command_s
            while rclpy.ok() and time.monotonic() < deadline:
                node.publish(linear_mps, angular_rad_s)
                rclpy.spin_once(node, timeout_sec=0.05)
            for _ in range(5):
                node.publish(0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
            spin_for(node, 2.10)
        return list(node.samples)
    finally:
        for _ in range(3):
            node.publish(0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("stationary", "motion"), required=True)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--command-s", type=float, default=0.5)
    parser.add_argument("--linear-mps", type=float, default=0.02)
    parser.add_argument("--angular-rad-s", type=float, default=0.20)
    parser.add_argument("--wheels-raised", default="")
    parser.add_argument("--report", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.duration_s <= 0.0:
        raise SystemExit("--duration-s must be positive")
    if not 0.0 < args.command_s <= 0.5:
        raise SystemExit("--command-s must be in (0, 0.5]")
    if not 0.0 < args.linear_mps <= 0.03:
        raise SystemExit("--linear-mps must be in (0, 0.03]")
    if not 0.0 < args.angular_rad_s <= 0.25:
        raise SystemExit("--angular-rad-s must be in (0, 0.25]")
    if args.mode == "motion":
        try:
            require_raised_confirmation(args.wheels_raised)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

    samples = _collect(
        mode=args.mode,
        duration_s=args.duration_s,
        command_s=args.command_s,
        linear_mps=args.linear_mps,
        angular_rad_s=0.0,
    )
    if args.mode == "stationary":
        report = evaluate_stationary(samples, duration_s=args.duration_s)
        turn_samples: list[BenchSample] = []
    else:
        report = evaluate_motion(samples, command_s=args.command_s)
        turn_samples = []
        if report["pass"]:
            turn_samples = _collect(
                mode="motion",
                duration_s=args.duration_s,
                command_s=args.command_s,
                linear_mps=0.0,
                angular_rad_s=args.angular_rad_s,
            )
            turn_report = evaluate_turn(
                turn_samples,
                command_s=args.command_s,
            )
        else:
            turn_report = {
                "pass": False,
                "reasons": ["forward gate failed; turn was not run"],
            }
        report["turn"] = turn_report
        report["pass"] = bool(report["pass"] and turn_report["pass"])
        report["reasons"] = list(report["reasons"]) + [
            f"turn: {reason}" for reason in turn_report["reasons"]
        ]
    report.update({
        "mode": args.mode,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "samples": [asdict(sample) for sample in samples],
        "turn_samples": [asdict(sample) for sample in turn_samples],
    })
    _write_report(args.report, report)
    verdict = "PI_BENCH_PASS" if report["pass"] else "PI_BENCH_FAIL"
    print(f"{verdict} mode={args.mode} report={args.report}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
