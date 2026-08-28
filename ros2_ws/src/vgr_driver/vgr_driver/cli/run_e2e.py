from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_core.model import CommandConfig
from vgr_driver.pipeline import Phase1Pipeline


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run Phase 1 vision pipeline against mock or real STM32 controller."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--video", type=Path)
    source.add_argument("--camera-index", type=int)
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--debug-video", type=Path, default=None)
    parser.add_argument("--report", type=Path, default=Path("outputs/e2e_report.json"))
    parser.add_argument("--controller", choices=["mock", "serial"], default="mock")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--min-confidence", type=float, default=CommandConfig.min_confidence)
    parser.add_argument("--target-lost-timeout-s", type=float, default=CommandConfig.target_lost_timeout_s)
    parser.add_argument("--camera-width", type=int, default=None)
    parser.add_argument("--camera-height", type=int, default=None)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    args = parser.parse_args()

    config = CommandConfig(
        min_confidence=args.min_confidence,
        target_lost_timeout_s=args.target_lost_timeout_s,
    )

    if args.controller == "mock":
        controller = _MockController(command_timeout_s=config.target_lost_timeout_s + 0.2)
    else:
        controller = _build_serial_controller(args.device, args.baudrate, args.timeout_s)

    resync = None
    if args.controller == "serial":
        time.sleep(args.settle_s)
        if hasattr(controller, "serial"):
            controller.serial.flush_input()
        resync = controller.resync()

    pipeline = Phase1Pipeline(config=config, controller=controller)
    if args.video is not None:
        diagnostics = pipeline.run_video(
            video_path=args.video,
            max_frames=args.max_frames,
            debug_video_path=args.debug_video,
        )
    else:
        diagnostics = pipeline.run_camera(
            camera_index=args.camera_index,
            max_frames=args.max_frames,
            debug_video_path=args.debug_video,
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
        )

    result = {
        "controller": args.controller,
        "device": args.device if args.controller == "serial" else None,
        "resync": None if resync is None else {
            "state": resync.state.name,
            "error": resync.error.name,
            "sequence": resync.sequence,
            "accepted": resync.accepted,
            "latency_ms": resync.latency_ms,
        },
        "summary": diagnostics.summary(),
        "events": diagnostics.events,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"resync": result["resync"], "summary": result["summary"]}, indent=2))

    passed = True
    if args.controller == "serial":
        passed = resync is not None and resync.accepted and diagnostics.mcu_rejected == 0
    print("E2E PIPELINE: PASS" if passed else "E2E PIPELINE: FAIL")
    return 0 if passed else 1


# ---------------------------------------------------------------------------
# Controller helpers
# ---------------------------------------------------------------------------


def _build_serial_controller(device: str, baudrate: int, timeout_s: float):
    from vgr_driver.driver import ControllerBridge, PosixSerial
    serial = PosixSerial(device=device, baudrate=baudrate, timeout_s=timeout_s)
    serial.open()
    bridge = ControllerBridge(serial)
    return _SerialControllerBridge(serial, bridge)


class _MockController:
    def __init__(self, command_timeout_s: float = 0.5) -> None:
        from vgr_driver.driver import MockMCU
        from vgr_core.protocol import CommandPacket, encode_command
        self.mcu = MockMCU(command_timeout_s=command_timeout_s)
        self.sequence = 0

    def send(self, command, timestamp=None):
        from vgr_core.protocol import CommandPacket, encode_command
        packet = CommandPacket(sequence=self.sequence, command=command)
        response = self.mcu.receive(encode_command(packet), timestamp=timestamp)
        self.sequence = (self.sequence + 1) & 0xFF
        return response

    def tick(self, timestamp=None):
        return self.mcu.tick(timestamp=timestamp)

    def close(self):
        return None

    def resync(self):
        from vgr_core.model import CommandID
        return self.send(CommandID.HEARTBEAT)


class _SerialControllerBridge:
    """Wraps ControllerBridge, exposing .serial for flush_input and resync."""
    def __init__(self, serial, bridge) -> None:
        self.serial = serial
        self._bridge = bridge

    def send(self, command, timestamp=None):
        from vgr_core.model import ErrorCode, MCUResponse
        exchange = self._bridge.send_command(command)
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

    def tick(self, timestamp=None):
        return None

    def close(self):
        self.serial.close()

    def resync(self):
        from vgr_core.model import CommandID
        return self.send(CommandID.HEARTBEAT)


if __name__ == "__main__":
    raise SystemExit(main())
