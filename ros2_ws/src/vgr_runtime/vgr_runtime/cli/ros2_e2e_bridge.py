from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vgr_core.model import (
    CommandConfig,
    CommandDecision,
    CommandGenerator,
    CommandID,
    Detection,
    MCUResponse,
    SafetyGovernor,
)
from vgr_core.control import Diagnostics
from vgr_driver.driver import ControllerBridge, MockMCU, PosixSerial
from vgr_core.protocol import CommandPacket, encode_command


class _MockController:
    """MCU stand-in for pre-hardware testing."""

    def __init__(self, command_timeout_s: float = 0.5) -> None:
        self.mcu = MockMCU(command_timeout_s=command_timeout_s)
        self.sequence = 0

    def send(self, command: CommandID, timestamp: float | None = None) -> MCUResponse:
        packet = CommandPacket(sequence=self.sequence, command=command)
        response = self.mcu.receive(encode_command(packet), timestamp=timestamp)
        self.sequence = (self.sequence + 1) & 0xFF
        return response

    def tick(self, timestamp: float | None = None) -> MCUResponse | None:
        return self.mcu.tick(timestamp=timestamp)

    def close(self) -> None:
        return None


class _SerialController:
    """Sends commands to real STM32 via serial bridge."""

    def __init__(self, device: str, baudrate: int = 115200, timeout_s: float = 0.5) -> None:
        self.serial = PosixSerial(device=device, baudrate=baudrate, timeout_s=timeout_s)
        self.serial.open()
        self.bridge = ControllerBridge(self.serial)

    def send(self, command: CommandID, timestamp: float | None = None) -> MCUResponse:
        del timestamp
        exchange = self.bridge.send_command(command)
        from vgr_core.model import ErrorCode
        accepted = exchange.state.error == ErrorCode.OK
        return MCUResponse(
            state=exchange.state.state,
            error=exchange.state.error,
            sequence=exchange.state.sequence,
            accepted=accepted,
            message="accepted" if accepted else exchange.state.error.name.lower(),
            latency_ms=exchange.latency_ms,
            motor_intent=exchange.state.motor_intent,
        )

    def tick(self, timestamp: float | None = None) -> MCUResponse | None:
        del timestamp
        return None

    def close(self) -> None:
        self.serial.close()


class Ros2E2EBridge(Node):
    """Publish e2e pipeline internal state as ROS2 topics."""

    def __init__(self) -> None:
        super().__init__("vision_guided_robot_e2e_bridge")
        self.declare_parameter("video_path", "")
        self.declare_parameter("controller", "mock")
        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self._config = CommandConfig()
        self._controller = self._build_controller()
        self._generator = CommandGenerator(self._config)
        self._governor = SafetyGovernor(self._config)
        self._diagnostics = Diagnostics()
        self._frame_index = 0
        self.published: dict[str, int] = {
            "frame": 0,
            "detection": 0,
            "command": 0,
            "mcu": 0,
        }

    def _build_controller(self):
        controller_type = self.get_parameter("controller").value
        if controller_type == "mock":
            return _MockController()
        else:
            device = str(self.get_parameter("device").value)
            baudrate = int(self.get_parameter("baudrate").value)
            return _SerialController(device, baudrate)

    def publish_frame(
        self,
        detection: Detection,
        decision: CommandDecision,
        response: MCUResponse | None,
    ) -> None:
        """Publish vision, command, MCU and diagnostics messages per frame."""
        self._diagnostics.record(detection, decision, response)
        frame_data = {
            "frame_index": detection.frame_index,
            "detected": detection.detected,
            "center_x": detection.center_x,
            "center_y": detection.center_y,
            "area_ratio": detection.area_ratio,
            "confidence": detection.confidence,
            "command": decision.command.name,
            "safety_state": decision.safety_state.name,
            "reason": decision.reason,
        }
        if response is not None:
            frame_data["mcu_state"] = response.state.name
            frame_data["mcu_error"] = response.error.name
            frame_data["motor_intent"] = response.motor_intent.name
        self._publish_frame_data(frame_data)
        self._frame_index += 1
        rclpy.spin_once(self, timeout_sec=0.0)

    def _publish_frame_data(self, data: dict) -> None:
        import std_msgs.msg
        pub = self.create_publisher(std_msgs.msg.String, "/e2e/frame", 10)
        msg = std_msgs.msg.String()
        msg.data = json.dumps(data)
        pub.publish(msg)

    def spin_until_empty(self) -> None:
        while self._frame_index > 0:
            rclpy.spin_once(self, timeout_sec=0.0)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run e2e vision pipeline and publish ROS2 topics.")
    parser.add_argument("--video-path", default="")
    parser.add_argument("--controller", choices=["mock", "serial"], default="mock")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--dump-json", default="")
    args = parser.parse_args()

    rclpy.init()
    node = Ros2E2EBridge()
    try:
        config = node._config
        diagnostics = node._diagnostics
        controller = node._controller
        generator = node._generator
        governor = node._governor
        import cv2
        cap = cv2.VideoCapture(args.video_path or 0)
        frame_idx = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            from vgr_driver.vision import ArucoDetector
            detector = ArucoDetector()
            detection = detector.detect(frame, frame_idx)
            proposed, reason = generator.from_detection(detection)
            decision = governor.evaluate(detection, proposed, reason)
            response = None
            if decision.accepted_by_governor:
                response = controller.send(decision.command)
            node.publish_frame(detection, decision, response)
            frame_idx += 1
        cap.release()
        if args.dump_json:
            diagnostics.write(Path(args.dump_json))
        result = diagnostics.summary()
        result["frames"] = diagnostics.frames
        result["detections"] = diagnostics.detections
        result["pass"] = diagnostics.commands_accepted > 0
        return 0 if result["pass"] else 1
    except Exception:
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
