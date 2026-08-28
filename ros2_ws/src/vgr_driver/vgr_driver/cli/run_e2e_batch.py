from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from vgr_core.model import CommandConfig
from vgr_driver.pipeline import Phase1Pipeline


DEFAULT_VIDEOS = [
    "marker_left.webm",
    "marker_right.webm",
    "marker_lost.webm",
    "marker_close.webm",
    "marker_up.webm",
]


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Phase 2 e2e batch certification.")
    parser.add_argument("--video-dir", type=Path, default=Path("marker_video"))
    parser.add_argument("--videos", nargs="+", default=DEFAULT_VIDEOS)
    parser.add_argument("--controller", choices=["mock", "serial"], default="mock")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--report", type=Path, default=Path("outputs/phase2_e2e_batch_report.json"))
    parser.add_argument("--per-video-dir", type=Path, default=Path("outputs/e2e_batch"))
    parser.add_argument("--min-confidence", type=float, default=CommandConfig.min_confidence)
    parser.add_argument("--target-lost-timeout-s", type=float, default=CommandConfig.target_lost_timeout_s)
    args = parser.parse_args()

    config = CommandConfig(
        min_confidence=args.min_confidence,
        target_lost_timeout_s=args.target_lost_timeout_s,
    )

    results = []
    for video_name in args.videos:
        video_path = args.video_dir / video_name
        if not video_path.exists():
            results.append(
                {
                    "video": video_name,
                    "pass": False,
                    "error": f"missing video: {video_path}",
                }
            )
            continue

        controller = _build_controller(args, config)
        resync = None
        try:
            if args.controller == "serial":
                time.sleep(args.settle_s)
                controller.serial.flush_input()
                resync = controller.resync()

            pipeline = Phase1Pipeline(config=config, controller=controller)
            diagnostics = pipeline.run_video(video_path=video_path, max_frames=args.max_frames)
            result = _evaluate_video(video_name, diagnostics.summary(), diagnostics.events, resync)
            _write_per_video_report(args.per_video_dir, video_name, result, diagnostics.events)
            results.append(result)
        except Exception as exc:
            results.append({"video": video_name, "pass": False, "error": str(exc)})
            try:
                controller.close()
            except Exception:
                pass

    aggregate = _aggregate_results(args, results)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(aggregate, indent=2), encoding="utf-8")
    print(json.dumps(aggregate["summary"], indent=2))
    if aggregate["pass"]:
        print("PHASE 2 E2E BATCH: PASS")
        return 0
    print("PHASE 2 E2E BATCH: FAIL")
    return 1


# ---------------------------------------------------------------------------
# Controller helpers
# ---------------------------------------------------------------------------


def _build_controller(args: argparse.Namespace, config: CommandConfig):
    if args.controller == "mock":
        return _MockController(command_timeout_s=config.target_lost_timeout_s + 0.2)
    return _build_serial_controller(args.device, args.baudrate, args.timeout_s)


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


# ---------------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------------


def _evaluate_video(video_name: str, summary: dict, events: list[dict], resync) -> dict:
    """Check whether a single video achieved its intended safety/control case."""
    accepted_events = [event for event in events if "mcu_error" in event]
    mcu_errors_ok = all(event["mcu_error"] == "OK" for event in accepted_events)
    sequence_reached_mcu = summary["mcu_accepted"] > 0
    expected = _expected_behavior(video_name)
    commands = {event["command"] for event in events}
    safety_states = {event["safety_state"] for event in events}

    behavior_ok = True
    if expected == "TURN_LEFT":
        behavior_ok = "TURN_LEFT" in commands
    elif expected == "TURN_RIGHT":
        behavior_ok = "TURN_RIGHT" in commands
    elif expected == "SAFE_STOP":
        behavior_ok = "SAFE_STOP" in safety_states or "STOP" in commands
    elif expected == "STOP":
        behavior_ok = "STOP" in commands

    resync_ok = True if resync is None else resync.accepted
    passed = bool(sequence_reached_mcu and mcu_errors_ok and behavior_ok and resync_ok)

    return {
        "video": video_name,
        "pass": passed,
        "expected_behavior": expected,
        "summary": summary,
        "checks": {
            "resync_ok": resync_ok,
            "mcu_received_commands": sequence_reached_mcu,
            "mcu_errors_ok": mcu_errors_ok,
            "expected_behavior_seen": behavior_ok,
        },
        "resync": None
        if resync is None
        else {
            "state": resync.state.name,
            "error": resync.error.name,
            "sequence": resync.sequence,
            "accepted": resync.accepted,
            "latency_ms": resync.latency_ms,
        },
    }


def _expected_behavior(video_name: str) -> str:
    """Map filename to test case for human-readable batch reports."""
    if "left" in video_name:
        return "TURN_LEFT"
    if "right" in video_name:
        return "TURN_RIGHT"
    if "lost" in video_name:
        return "SAFE_STOP"
    if "close" in video_name or "up" in video_name:
        return "STOP"
    return "ANY"


def _write_per_video_report(
    output_dir: Path,
    video_name: str,
    result: dict,
    events: list[dict],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{Path(video_name).stem}.json"
    output_path.write_text(
        json.dumps({"result": result, "events": events}, indent=2),
        encoding="utf-8",
    )


def _aggregate_results(args: argparse.Namespace, results: list[dict]) -> dict:
    """Summarise batch certification results."""
    passed = all(result.get("pass", False) for result in results)
    total_frames = sum(result.get("summary", {}).get("frames", 0) for result in results)
    total_mcu_accepted = sum(
        result.get("summary", {}).get("mcu_accepted", 0) for result in results
    )
    total_mcu_rejected = sum(
        result.get("summary", {}).get("mcu_rejected", 0) for result in results
    )
    return {
        "pass": passed,
        "controller": args.controller,
        "device": args.device if args.controller == "serial" else None,
        "videos": results,
        "summary": {
            "pass": passed,
            "controller": args.controller,
            "video_count": len(results),
            "passed_videos": sum(1 for result in results if result.get("pass", False)),
            "total_frames": total_frames,
            "total_mcu_accepted": total_mcu_accepted,
            "total_mcu_rejected": total_mcu_rejected,
        },
    }


if __name__ == "__main__":
    raise SystemExit(main())
