from __future__ import annotations

import argparse
import json
import os
import subprocess
import threading
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


TOPICS = {
    "vision": "/vision/target",
    "command": "/robot/high_level_command",
    "mcu": "/mcu/state",
    "diagnostics": "/diagnostics",
}


class TopicCollector(Node):
    def __init__(self) -> None:
        super().__init__("phase2_ros2_topic_certifier")
        self.messages: dict[str, list[dict]] = {key: [] for key in TOPICS}
        for key, topic in TOPICS.items():
            self.create_subscription(String, topic, self._callback_for(key), 10)

    def _callback_for(self, key: str):
        def callback(msg: String) -> None:
            try:
                self.messages[key].append(json.loads(msg.data))
            except json.JSONDecodeError:
                self.messages[key].append({"raw": msg.data})

        return callback


def main() -> int:
    parser = argparse.ArgumentParser(description="Certify ROS2 topics emitted by the e2e bridge.")
    parser.add_argument("--video", type=Path, default=Path("marker_video/marker_left.webm"))
    parser.add_argument("--controller", choices=["mock", "serial"], default="mock")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--max-frames", type=int, default=60)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--report", type=Path, default=Path("outputs/ros2_topic_certification.json"))
    args = parser.parse_args()

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/vision_guided_robot_ros_logs")
    rclpy.init()
    collector = TopicCollector()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(collector)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    bridge_cmd = build_bridge_command(args)
    bridge_proc = subprocess.Popen(bridge_cmd, cwd=Path.cwd(), env=os.environ.copy())
    try:
        deadline = time.time() + args.timeout_s
        while time.time() < deadline and bridge_proc.poll() is None:
            if all(len(collector.messages[key]) > 0 for key in TOPICS):
                break
            time.sleep(0.1)
        bridge_exit = bridge_proc.wait(timeout=max(0.1, args.timeout_s))
        time.sleep(0.5)
    finally:
        if bridge_proc.poll() is None:
            bridge_proc.terminate()
            try:
                bridge_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                bridge_proc.kill()
        executor.shutdown()
        collector.destroy_node()
        rclpy.shutdown()

    inner_report_path = Path("outputs/ros2_topic_bridge_inner_report.json")
    inner_report = {}
    inner_report_error = None
    if inner_report_path.exists():
        try:
            inner_report = json.loads(inner_report_path.read_text(encoding="utf-8"))
        except Exception as exc:
            inner_report_error = f"{type(exc).__name__}: {exc}"
    result = evaluate(collector.messages, bridge_exit, args.controller, inner_report)
    if inner_report_error is not None:
        result["inner_report_error"] = inner_report_error
        result["pass"] = False
        result["summary"]["pass"] = False
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result["summary"], indent=2))
    print("ROS2 TOPIC CERTIFICATION: PASS" if result["pass"] else "ROS2 TOPIC CERTIFICATION: FAIL")
    return 0 if result["pass"] else 1


def build_bridge_command(args: argparse.Namespace) -> list[str]:
    command = [
        "python3",
        "-m",
        "vgr_runtime.cli.ros2_e2e_bridge",
        "--video",
        str(args.video),
        "--controller",
        args.controller,
        "--max-frames",
        str(args.max_frames),
        "--report",
        "outputs/ros2_topic_bridge_inner_report.json",
    ]
    if args.controller == "serial":
        command.extend(["--device", args.device, "--baudrate", str(args.baudrate)])
    return command


def evaluate(
    messages: dict[str, list[dict]],
    bridge_exit: int,
    controller: str = "mock",
    inner_report: dict | None = None,
) -> dict:
    counts = {key: len(value) for key, value in messages.items()}
    commands = Counter(msg.get("command") for msg in messages["command"])
    mcu_errors = Counter(msg.get("error") for msg in messages["mcu"])
    mcu_states = Counter(msg.get("state") for msg in messages["mcu"])
    inner = inner_report or {}
    final_stop_state = inner.get("final_stop_state")
    ended_with_stop = final_stop_state in ("STOP", "SAFE_STOP") if controller == "serial" else True
    checks = {
        "bridge_exit_ok": bridge_exit == 0,
        "vision_topic_received": counts["vision"] > 0,
        "command_topic_received": counts["command"] > 0,
        "mcu_topic_received": counts["mcu"] > 0,
        "diagnostics_topic_received": counts["diagnostics"] > 0,
        "turn_left_seen": commands.get("TURN_LEFT", 0) > 0,
        "mcu_errors_ok": all(error == "OK" for error in mcu_errors if error is not None),
        "tracking_seen": mcu_states.get("TRACKING", 0) > 0,
        "ended_with_stop": ended_with_stop,
    }
    return {
        "pass": all(checks.values()),
        "summary": {
            "pass": all(checks.values()),
            "bridge_exit": bridge_exit,
            "topic_counts": counts,
            "commands": dict(commands),
            "mcu_states": dict(mcu_states),
            "mcu_errors": dict(mcu_errors),
            "final_stop_state": final_stop_state,
            "checks": checks,
        },
        "messages": messages,
    }


if __name__ == "__main__":
    raise SystemExit(main())
