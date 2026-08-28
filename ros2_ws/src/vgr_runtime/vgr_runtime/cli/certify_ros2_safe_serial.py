from __future__ import annotations

import argparse
import json
import os
import pty
import time
from pathlib import Path

from vgr_core.model import CommandID

from vgr_driver.driver import ControllerBridge
from vgr_driver.driver.mock_serial_mcu import MockSerialMCU
from vgr_driver.driver import PosixSerial


SAFE_COMMANDS = (CommandID.HEARTBEAT, CommandID.STOP)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Certify ROS2-side safe serial access without issuing motion commands."
    )
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--settle-s", type=float, default=0.0)
    parser.add_argument("--mock-serial", action="store_true")
    parser.add_argument("--publish-ros2", action="store_true")
    parser.add_argument("--report", type=Path, default=Path("outputs/ros2_safe_serial_certification.json"))
    args = parser.parse_args()

    master_fd = None
    slave_fd = None
    mock_mcu = None
    device = args.device
    if args.mock_serial:
        master_fd, slave_fd = pty.openpty()
        device = os.ttyname(slave_fd)
        mock_mcu = MockSerialMCU(master_fd, timeout_s=args.timeout_s)
        mock_mcu.start()

    result = {
        "pass": False,
        "device": device,
        "using_pty_mock_mcu": args.mock_serial,
        "commands_requested": [command.name for command in SAFE_COMMANDS],
        "exchanges": [],
        "checks": {},
        "published_ros2": False,
        "error": None,
    }
    try:
        if not args.mock_serial and args.settle_s > 0:
            time.sleep(args.settle_s)
        exchanges = run_safe_serial_sequence(device=device, baudrate=args.baudrate, timeout_s=args.timeout_s)
        result["exchanges"] = exchanges
        evaluation = evaluate_safe_exchanges(exchanges)
        result["checks"] = evaluation["checks"]
        result["pass"] = evaluation["pass"]
        if args.publish_ros2:
            publish_ros2_summary(result)
            result["published_ros2"] = True
    except Exception as exc:  # noqa: BLE001 - CLI reports hardware/ROS failures as JSON.
        result["error"] = str(exc)
    finally:
        if mock_mcu is not None:
            mock_mcu.stop()
        if master_fd is not None:
            os.close(master_fd)
        if slave_fd is not None:
            os.close(slave_fd)

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    print("ROS2 SAFE SERIAL CERTIFICATION: PASS" if result["pass"] else "ROS2 SAFE SERIAL CERTIFICATION: FAIL")
    return 0 if result["pass"] else 1


def run_safe_serial_sequence(*, device: str, baudrate: int, timeout_s: float) -> list[dict]:
    with PosixSerial(device=device, baudrate=baudrate, timeout_s=timeout_s) as serial:
        serial.flush_input()
        bridge = ControllerBridge(serial)
        exchanges = []
        for command in SAFE_COMMANDS:
            exchange = bridge.send_command(command)
            state = exchange.state
            exchanges.append(
                {
                    "command": command.name,
                    "host_sequence": exchange.sequence,
                    "state_sequence": state.sequence,
                    "state": state.state.name,
                    "error": state.error.name,
                    "motor_intent": state.motor_intent.name,
                    "uptime_ms": state.uptime_ms,
                    "latency_ms": exchange.latency_ms,
                }
            )
        return exchanges


def evaluate_safe_exchanges(exchanges: list[dict]) -> dict:
    requested = [command.name for command in SAFE_COMMANDS]
    seen = [exchange.get("command") for exchange in exchanges]
    checks = {
        "only_safe_commands_requested": seen == requested,
        "read_all_state_packets": len(exchanges) == len(SAFE_COMMANDS),
        "sequence_echo_ok": all(
            exchange.get("host_sequence") == exchange.get("state_sequence")
            for exchange in exchanges
        ),
        "mcu_errors_ok": all(exchange.get("error") == "OK" for exchange in exchanges),
        "all_motor_intents_stop": all(
            exchange.get("motor_intent") == "STOP" for exchange in exchanges
        ),
        "ended_safe_stop": bool(exchanges) and exchanges[-1].get("state") == "SAFE_STOP",
    }
    return {"pass": all(checks.values()), "checks": checks}


def publish_ros2_summary(result: dict) -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SafeSerialNode(Node):
        def __init__(self) -> None:
            super().__init__("vgr_safe_serial_certifier")
            self.mcu_pub = self.create_publisher(String, "/mcu/state", 10)
            self.diag_pub = self.create_publisher(String, "/diagnostics", 10)

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/vision_guided_robot_ros_logs")
    rclpy.init()
    node = SafeSerialNode()
    try:
        for exchange in result["exchanges"]:
            msg = String()
            msg.data = json.dumps(exchange)
            node.mcu_pub.publish(msg)
        diag = String()
        diag.data = json.dumps(
            {
                "source": "vgr_runtime.cli.certify_ros2_safe_serial",
                "pass": result["pass"],
                "checks": result["checks"],
            }
        )
        node.diag_pub.publish(diag)
        rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
