from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Phase2DiagnosticsPublisher(Node):
    def __init__(self, report_path: Path, topic: str) -> None:
        super().__init__("phase2_diagnostics_publisher")
        self.report_path = report_path
        self.publisher = self.create_publisher(String, topic, 10)

    def publish_once(self) -> dict:
        payload = self._load_payload()
        message = String()
        message.data = json.dumps(payload)
        self.publisher.publish(message)
        self.get_logger().info(f"published Phase 2 diagnostics from {self.report_path}")
        return payload

    def _load_payload(self) -> dict:
        if not self.report_path.exists():
            raise FileNotFoundError(self.report_path)
        report = json.loads(self.report_path.read_text(encoding="utf-8"))
        return {
            "source": str(self.report_path),
            "pass": bool(report.get("pass") or report.get("summary", {}).get("pass")),
            "controller": report.get("controller") or report.get("summary", {}).get("controller"),
            "device": report.get("device"),
            "summary": report.get("summary", report),
        }


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish Phase 2 validation diagnostics to ROS2.")
    parser.add_argument("--report", type=Path, default=Path("outputs/phase2_e2e_batch_report.json"))
    parser.add_argument("--topic", default="/vgr/diagnostics")
    parser.add_argument("--once", action="store_true", default=True)
    args = parser.parse_args()

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/vision_guided_robot_ros_logs")
    rclpy.init()
    node = Phase2DiagnosticsPublisher(args.report, args.topic)
    try:
        payload = node.publish_once()
        rclpy.spin_once(node, timeout_sec=0.2)
        print(json.dumps(payload, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("ROS2 DIAGNOSTICS PUBLISH: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
