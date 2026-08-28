"""E2 reverse-speed constant-publishing node.

Publishes /cmd_vel_nav (Twist, linear.x negative = reverse) at VGR_REV_V
(default -0.05 m/s) for VGR_REV_DURATION_S (default 30 s) or until SIGTERM.

Use: paper-box wall, VGR_GROUND_RUN=YES, motors enabled.
Safety: requires paper-box wall and VGR_GROUND_RUN=YES before enabling motors.
"""
from __future__ import annotations

import math
import signal
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ReverseCmdPublisher(Node):
    """Constant-speed reverse cmd_vel publisher."""

    def __init__(
        self,
        v_mps: float,
        duration_s: float,
        wiggle_w_amp: float = 0.0,
        wiggle_period_s: float = 6.0,
        spin_s: float = 0.0,
        dash_s: float = 4.0,
    ) -> None:
        super().__init__("reverse_cmd_publisher")
        self._v_mps = v_mps
        self._duration_s = duration_s
        self._w_amp = wiggle_w_amp
        self._w_period = max(wiggle_period_s, 0.5)
        self._spin_s = spin_s
        self._dash_s = max(dash_s, 0.5)
        self._pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._timer_period = 0.1
        self._elapsed = 0.0
        self._stopped = False
        self._timer = self.create_timer(self._timer_period, self._on_timer)
        self.get_logger().info(
            f"reverse_cmd_publisher up: v={self._v_mps} m/s, duration={self._duration_s} s"
        )
        signal.signal(signal.SIGTERM, self._on_sigterm)

    def _on_timer(self) -> None:
        if self._stopped:
            return
        self._elapsed += self._timer_period
        twist = Twist()
        if self._spin_s > 0.0:
            cycle = self._spin_s + self._dash_s
            t_in = self._elapsed % cycle
            n_cycle = int(self._elapsed // cycle)
            if t_in < self._spin_s:  # spin-in-place phase, direction alternates each cycle
                twist.linear.x = 0.0
                twist.angular.z = self._w_amp if n_cycle % 2 == 0 else -self._w_amp
            else:  # straight-dash phase
                twist.linear.x = self._v_mps
                twist.angular.z = 0.0
        else:
            twist.linear.x = self._v_mps
            twist.angular.z = self._w_amp * math.sin(
                2 * math.pi * self._elapsed / self._w_period
            )
        self._pub.publish(twist)
        if self._elapsed >= self._duration_s:
            self._stop_and_exit()

    def _stop_and_exit(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self._publish_stop()
        self.get_logger().info("duration reached, stopped")
        raise SystemExit(0)

    def _publish_stop(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._pub.publish(twist)

    def _on_sigterm(self, signum: int, frame) -> None:
        self.get_logger().info("SIGTERM received, stopping")
        self._stopped = True
        self._publish_stop()
        raise SystemExit(0)


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(
        description="Reverse constant-speed cmd_vel publisher."
    )
    parser.add_argument(
        "--v-mps",
        type=float,
        default=-0.05,
        help="Reverse speed in m/s (negative)",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=30.0,
        help="Duration to publish in seconds",
    )
    args, _unknown = parser.parse_known_args()

    rclpy.init()
    node = ReverseCmdPublisher(
        v_mps=args.v_mps,
        duration_s=args.duration_s,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
