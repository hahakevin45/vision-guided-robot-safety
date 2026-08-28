"""Vision-to-control pipeline.

Wires ArucoDetector → CommandGenerator → SafetyGovernor → Controller
and produces a Diagnostics report.  Supports recorded video, live camera,
mock MCU or real STM32.
"""
from __future__ import annotations

from pathlib import Path
from time import monotonic
from typing import Callable

import cv2

from vgr_core.control import Diagnostics
from vgr_core.model import (
    CommandConfig,
    CommandDecision,
    CommandGenerator,
    Detection,
    MCUResponse,
    SafetyGovernor,
)
from vgr_driver.vision import ArucoDetector, draw_detection_overlay


FrameCallback = Callable[
    [Detection, CommandDecision, MCUResponse | None, dict], None
]


class Phase1Pipeline:
    """Main vision-to-control pipeline.

    Same pipeline works with recorded video, live camera, mock MCU or real STM32,
    enabling direct reuse of Phase 1 vision/safety logic in Phase 2 hardware
    validation.
    """

    def __init__(
        self,
        config: CommandConfig | None = None,
        controller=None,
    ) -> None:
        self.config = config or CommandConfig()
        self.detector = ArucoDetector()
        self.generator = CommandGenerator(self.config)
        self.governor = SafetyGovernor(self.config)
        self.controller = controller or _make_mock(self.config)

    def run_video(
        self,
        video_path: Path,
        max_frames: int = 120,
        debug_video_path: Path | None = None,
        frame_callback: FrameCallback | None = None,
    ) -> Diagnostics:
        """Run the full pipeline against a recorded video for reproducible regression tests."""
        diagnostics = Diagnostics()
        cap = cv2.VideoCapture(str(video_path))
        out = None
        frame_idx = 0
        try:
            if debug_video_path is not None:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                out = cv2.VideoWriter(
                    str(debug_video_path),
                    cv2.VideoWriter_fourcc(*"vp80"),
                    30.0,
                    (w, h),
                )
            while frame_idx < max_frames:
                ret, frame = cap.read()
                if not ret:
                    break
                detection, decision, response, extra = self.process_frame(
                    frame, frame_idx
                )
                diagnostics.record(detection, decision, response)
                if out is not None:
                    command_text = decision.command.name
                    safety_text = decision.safety_state.name
                    annotated = draw_detection_overlay(
                        frame, detection, command_text, safety_text
                    )
                    out.write(annotated)
                if frame_callback is not None:
                    frame_callback(detection, decision, response, extra)
                frame_idx += 1
        finally:
            cap.release()
            if out is not None:
                out.release()
        return diagnostics

    def run_camera(
        self,
        camera_index: int = 0,
        max_frames: int = 120,
        debug_video_path: Path | None = None,
        frame_callback: FrameCallback | None = None,
        width: int | None = None,
        height: int | None = None,
        fps: float = 30.0,
    ) -> Diagnostics:
        """Run the pipeline against a live USB camera for on-device demos."""
        diagnostics = Diagnostics()
        cap = cv2.VideoCapture(camera_index)
        if width is not None and height is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        out = None
        frame_idx = 0
        try:
            if debug_video_path is not None:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                out = cv2.VideoWriter(
                    str(debug_video_path),
                    cv2.VideoWriter_fourcc(*"vp80"),
                    fps,
                    (w, h),
                )
            while frame_idx < max_frames:
                ret, frame = cap.read()
                if not ret:
                    break
                detection, decision, response, extra = self.process_frame(
                    frame, frame_idx
                )
                diagnostics.record(detection, decision, response)
                if out is not None:
                    command_text = decision.command.name
                    safety_text = decision.safety_state.name
                    annotated = draw_detection_overlay(
                        frame, detection, command_text, safety_text
                    )
                    out.write(annotated)
                if frame_callback is not None:
                    frame_callback(detection, decision, response, extra)
                frame_idx += 1
        finally:
            cap.release()
            if out is not None:
                out.release()
        return diagnostics

    def process_frame(
        self,
        frame,
        frame_index: int,
    ) -> tuple[Detection, CommandDecision, MCUResponse | None, dict]:
        """Process a single frame: detect, generate command, safety-check, send to MCU."""
        timestamp = monotonic()
        detection = self.detector.detect(frame, frame_index, timestamp)
        proposed, reason = self.generator.from_detection(detection)
        decision = self.governor.evaluate(detection, proposed, reason)
        response = None
        timeout_response = None
        if decision.accepted_by_governor:
            response = self.controller.send(decision.command, timestamp)
        else:
            timeout_response = self.controller.tick(timestamp)
        return detection, decision, response, {"timeout": timeout_response is not None}


def _make_mock(config: CommandConfig):
    """Build a mock controller using the new inline classes."""
    from vgr_driver.driver import ControllerBridge, MockMCU, PosixSerial
    from vgr_core.protocol import CommandPacket, encode_command

    class _MockController:
        def __init__(self, command_timeout_s: float = 0.5) -> None:
            self.mcu = MockMCU(command_timeout_s=command_timeout_s)
            self.sequence = 0

        def send(self, command, timestamp=None) -> MCUResponse:
            from vgr_core.protocol import CommandPacket, encode_command
            packet = CommandPacket(sequence=self.sequence, command=command)
            response = self.mcu.receive(encode_command(packet), timestamp=timestamp)
            self.sequence = (self.sequence + 1) & 0xFF
            return response

        def tick(self, timestamp=None) -> MCUResponse | None:
            return self.mcu.tick(timestamp=timestamp)

        def close(self) -> None:
            return None

        def resync(self):
            return self.send(4)  # CommandID.HEARTBEAT

    return _MockController(command_timeout_s=config.target_lost_timeout_s + 0.2)
