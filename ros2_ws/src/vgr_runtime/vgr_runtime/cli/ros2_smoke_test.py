from __future__ import annotations

import argparse
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a local ROS2 pub/sub smoke test.")
    parser.add_argument("--topic", default="/vgr/smoke")
    parser.add_argument("--message", default="phase2_ros2_smoke")
    parser.add_argument("--timeout-s", type=float, default=2.0)
    args = parser.parse_args()

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/vision_guided_robot_ros_logs")
    received: list[str] = []

    class Sub(Node):
        def __init__(self) -> None:
            super().__init__("vgr_smoke_sub")
            self.create_subscription(String, args.topic, self.cb, 10)

        def cb(self, msg: String) -> None:
            received.append(msg.data)

    class Pub(Node):
        def __init__(self) -> None:
            super().__init__("vgr_smoke_pub")
            self.publisher = self.create_publisher(String, args.topic, 10)

    rclpy.init()
    sub = Sub()
    pub = Pub()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sub)
    executor.add_node(pub)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        time.sleep(0.5)
        msg = String()
        msg.data = args.message
        for _ in range(5):
            pub.publisher.publish(msg)
            time.sleep(0.1)

        deadline = time.time() + args.timeout_s
        while time.time() < deadline and args.message not in received:
            time.sleep(0.05)
    finally:
        executor.shutdown()
        sub.destroy_node()
        pub.destroy_node()
        rclpy.shutdown()

    passed = args.message in received
    print({"received": received, "pass": passed})
    print("ROS2 SMOKE TEST: PASS" if passed else "ROS2 SMOKE TEST: FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
