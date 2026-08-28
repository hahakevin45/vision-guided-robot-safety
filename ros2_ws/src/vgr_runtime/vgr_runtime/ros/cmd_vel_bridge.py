"""ROS 2 node: /cmd_vel (geometry_msgs/Twist) → serial SET_WHEEL_SPEED.

Bridges Nav2 (or any cmd_vel source) to the robot's serial protocol.
Inverse kinematics are shared with the non-ROS drive_cmd_vel via vgr_core.motion.

Safety:
- Deadman: STOP if no new cmd_vel arrives within cmd_timeout_s (firmware also has
  a deadman — double protection).
- STOP on node shutdown.
- Speed clamped by DiffDriveParams.max_counts_per_s.

Usage (real robot, operator present):
    ros2 run vgr_runtime cmd_vel_bridge --ros-args -p device:=/dev/ttyACM0 -p max_counts_per_s:=400
"""
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from vgr_core.model import CommandID
from vgr_core.motion import DiffDriveParams, twist_to_wheel_counts
from vgr_driver.driver import ControllerBridge, PosixSerial


class CmdVelSerialBridge(Node):
    """Subscribe to /cmd_vel and convert to wheel counts/s sent over serial."""

    def __init__(
        self,
        bridge: ControllerBridge,
        params: DiffDriveParams,
        cmd_timeout_s: float = 0.5,
    ) -> None:
        super().__init__("cmd_vel_serial_bridge")
        self._bridge = bridge
        self._params = params
        self._cmd_timeout_s = cmd_timeout_s
        self._last_cmd_monotonic: float | None = None
        self._moving = False
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        # Check deadman more frequently than the timeout to ensure a fast stop.
        self.create_timer(cmd_timeout_s / 5.0, self._deadman_check)
        self.get_logger().info(
            f"cmd_vel bridge up: wheel_base={params.wheel_base_m}m "
            f"max={params.max_counts_per_s}cps timeout={cmd_timeout_s}s"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        left_cps, right_cps = twist_to_wheel_counts(
            msg.linear.x, msg.angular.z, self._params
        )
        try:
            self._bridge.send_set_wheel_speed(left_cps, right_cps)
            self._last_cmd_monotonic = self._now()
            self._moving = left_cps != 0 or right_cps != 0
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"send_set_wheel_speed failed: {exc}")
            self.stop()

    def _deadman_check(self) -> None:
        if not self._moving or self._last_cmd_monotonic is None:
            return
        if (self._now() - self._last_cmd_monotonic) > self._cmd_timeout_s:
            self.get_logger().warn("cmd_vel deadman timeout → STOP")
            self.stop()

    def stop(self) -> None:
        try:
            self._bridge.send_command(CommandID.STOP)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"STOP send failed: {exc}")
        self._moving = False

    @staticmethod
    def _now() -> float:
        return time.monotonic()


def _build_params(args: argparse.Namespace) -> DiffDriveParams:
    return DiffDriveParams(
        wheel_base_m=args.wheel_base_m,
        wheel_diameter_m=args.wheel_diameter_cm / 100.0,
        left_counts_per_rev=args.left_counts_per_rev,
        right_counts_per_rev=args.right_counts_per_rev,
        max_counts_per_s=args.max_counts_per_s,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Bridge /cmd_vel to serial SET_WHEEL_SPEED."
    )
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--cmd-timeout-s", type=float, default=0.5)
    parser.add_argument("--wheel-base-m", type=float, default=0.165)
    parser.add_argument("--wheel-diameter-cm", type=float, default=6.5)
    parser.add_argument("--left-counts-per-rev", type=float, default=750.0)
    parser.add_argument("--right-counts-per-rev", type=float, default=749.0)
    parser.add_argument("--max-counts-per-s", type=int, default=400)
    args = parser.parse_args()

    params = _build_params(args)
    rclpy.init()
    try:
        with PosixSerial(
            device=args.device,
            baudrate=args.baudrate,
            timeout_s=args.timeout_s,
        ) as serial:
            if args.settle_s > 0:
                time.sleep(args.settle_s)
                serial.flush_input()
            bridge = ControllerBridge(serial)
            bridge.send_command(CommandID.HEARTBEAT)
            node = CmdVelSerialBridge(
                bridge, params, cmd_timeout_s=args.cmd_timeout_s
            )
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.stop()
                node.destroy_node()
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
