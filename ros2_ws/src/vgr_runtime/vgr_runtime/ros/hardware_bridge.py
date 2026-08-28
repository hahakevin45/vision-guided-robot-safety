"""ROS 2 hardware bridge node for real robot.

Single-owner of /dev/ttyACM0. Subscribes to /cmd_vel_safe and publishes
/odom, /hardware/status, and TF. No motion allowed by default.
"""
from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from vgr_driver.driver import (
    ControllerBridge,
    FaultInjectingSerial,
    HardwareBridgeConfig,
    HardwareBridgeSession,
    HardwareSample,
)
from vgr_driver.driver.serial_transport import PosixSerial
from vgr_driver.driver.hardware_bridge import odom_payload


class HardwareBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vgr_hardware_bridge")
        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_s", 0.10)
        self.declare_parameter("settle_s", 0.50)
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("cmd_timeout_s", 0.20)
        self.declare_parameter("fault_inject_mode", "none")
        self.declare_parameter("fault_inject_at_s", -1.0)
        self.declare_parameter("fault_inject_count", 10)
        self.declare_parameter("wheel_base_m", 0.165)
        self.declare_parameter("wheel_diameter_m", 0.065)
        self.declare_parameter("left_counts_per_rev", 750.0)
        self.declare_parameter("right_counts_per_rev", 749.0)
        self.declare_parameter("left_encoder_sign", 1)
        self.declare_parameter("right_encoder_sign", 1)
        self.declare_parameter("max_counts_per_s", 120)

        config = HardwareBridgeConfig(
            allow_motion=bool(self.get_parameter("allow_motion").value),
            cmd_timeout_s=float(self.get_parameter("cmd_timeout_s").value),
            wheel_base_m=float(self.get_parameter("wheel_base_m").value),
            wheel_diameter_m=float(self.get_parameter("wheel_diameter_m").value),
            left_counts_per_rev=float(
                self.get_parameter("left_counts_per_rev").value
            ),
            right_counts_per_rev=float(
                self.get_parameter("right_counts_per_rev").value
            ),
            left_encoder_sign=int(
                self.get_parameter("left_encoder_sign").value
            ),
            right_encoder_sign=int(
                self.get_parameter("right_encoder_sign").value
            ),
            max_counts_per_s=int(
                self.get_parameter("max_counts_per_s").value
            ),
        )
        device = str(self.get_parameter("device").value)
        baudrate = int(self.get_parameter("baudrate").value)
        serial_timeout_s = float(
            self.get_parameter("serial_timeout_s").value
        )
        settle_s = float(self.get_parameter("settle_s").value)
        poll_hz = float(self.get_parameter("poll_hz").value)
        if poll_hz <= 0.0:
            raise ValueError("poll_hz must be positive")

        self._serial = PosixSerial(
            device=device,
            baudrate=baudrate,
            timeout_s=serial_timeout_s,
        )
        self._closed = False
        try:
            self._serial.open()
            if settle_s > 0.0:
                time.sleep(settle_s)
            self._serial.flush_input()
            fault_inject_mode = str(
                self.get_parameter("fault_inject_mode").value
            )
            fault_inject_at_s = float(
                self.get_parameter("fault_inject_at_s").value
            )
            fault_inject_count = int(
                self.get_parameter("fault_inject_count").value
            )
            controller = ControllerBridge(self._serial)
            self._session = HardwareBridgeSession(
                controller,
                config,
            )
            self._latest_sample = self._session.startup(time.monotonic())
            controller.serial_port = FaultInjectingSerial(
                self._serial,
                mode=fault_inject_mode,
                at_s=fault_inject_at_s,
                count=fault_inject_count,
                logger=self.get_logger(),
            )
        except Exception:
            self._serial.close()
            raise

        self._last_exchange_monotonic = time.monotonic()
        self._odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self._status_pub = self.create_publisher(
            String,
            "/hardware/status",
            20,
        )
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_cmd, 10)
        self._timer = self.create_timer(1.0 / poll_hz, self._on_timer)
        self.get_logger().info(
            "hardware bridge ready: "
            f"device={device} allow_motion={config.allow_motion} "
            f"max={config.max_counts_per_s}cps "
            f"timeout={config.cmd_timeout_s}s "
            f"fault_inject={fault_inject_mode}@{fault_inject_at_s}s"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._session.update_command(
            float(msg.linear.x),
            float(msg.angular.z),
            stamp_s=time.monotonic(),
        )

    def _on_timer(self) -> None:
        from vgr_driver.driver.hardware_bridge import HardwareFault
        try:
            sample = self._session.cycle(time.monotonic())
        except HardwareFault as exc:
            self.get_logger().error(str(exc))
            self._publish_status(self._latest_sample)
            self._timer.cancel()
            return
        self._latest_sample = sample
        self._last_exchange_monotonic = time.monotonic()
        self._publish_odom(sample)
        self._publish_status(sample)

    def _publish_odom(self, sample: HardwareSample) -> None:
        values = odom_payload(sample.odom)
        now = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = str(values["frame_id"])
        msg.child_frame_id = str(values["child_frame_id"])
        position = values["position"]
        quaternion = values["quaternion"]
        twist = values["twist"]
        msg.pose.pose.position.x = position["x"]
        msg.pose.pose.position.y = position["y"]
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        msg.pose.covariance = values["pose_covariance"]
        msg.twist.twist.linear.x = twist["linear_x"]
        msg.twist.twist.angular.z = twist["angular_z"]
        msg.twist.covariance = values["twist_covariance"]
        self._odom_pub.publish(msg)

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(transform)

    def _publish_status(self, sample: HardwareSample) -> None:
        status = String()
        status.data = json.dumps({
            "allow_motion": self._session.config.allow_motion,
            "fault": self._session.fault,
            "raw_left": sample.raw_left,
            "raw_right": sample.raw_right,
            "left_target_cps": sample.left_target_cps,
            "right_target_cps": sample.right_target_cps,
            "command_latency_ms": sample.command_latency_ms,
            "encoder_latency_ms": sample.encoder_latency_ms,
            "last_exchange_age_s": max(
                0.0,
                time.monotonic() - self._last_exchange_monotonic,
            ),
        }, sort_keys=True)
        self._status_pub.publish(status)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._session.stop()
        except Exception as exc:
            self.get_logger().error(f"shutdown STOP failed: {exc}")
        finally:
            self._serial.close()


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = HardwareBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
