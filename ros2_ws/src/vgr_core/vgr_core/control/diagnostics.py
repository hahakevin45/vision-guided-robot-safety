"""Pipeline instrumentation and diagnostics.

Collects per-frame events and produces reproducible JSON reports.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from statistics import mean

from vgr_core.model import CommandDecision, Detection, MCUResponse


@dataclass
class Diagnostics:
    """Accumulated pipeline execution events and statistics."""

    frames: int = 0
    detections: int = 0
    commands_accepted: int = 0
    commands_rejected: int = 0
    mcu_accepted: int = 0
    mcu_rejected: int = 0
    timeouts: int = 0
    detection_latency_ms: list[float] = field(default_factory=list)
    mcu_latency_ms: list[float] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)

    def record(
        self,
        detection: Detection,
        decision: CommandDecision,
        response: MCUResponse | None,
    ) -> None:
        """Record a single frame's detection, command decision and MCU response."""
        self.frames += 1
        if detection.detected:
            self.detections += 1
        if decision.accepted_by_governor:
            self.commands_accepted += 1
        else:
            self.commands_rejected += 1
        self.detection_latency_ms.append(detection.latency_ms)

        event = {
            "frame": detection.frame_index,
            "detected": detection.detected,
            "center_x": detection.center_x,
            "center_y": detection.center_y,
            "area_ratio": detection.area_ratio,
            "confidence": detection.confidence,
            "command": decision.command.name,
            "safety_state": decision.safety_state.name,
            "reason": decision.reason,
            "governor_accepted": decision.accepted_by_governor,
        }
        if response is not None:
            if response.accepted:
                self.mcu_accepted += 1
            else:
                self.mcu_rejected += 1
            self.mcu_latency_ms.append(response.latency_ms)
            event.update({
                "mcu_state": response.state.name,
                "mcu_error": response.error.name,
                "motor_intent": response.motor_intent.name,
                "mcu_message": response.message,
            })
        self.events.append(event)

    def record_timeout(self) -> None:
        """Record a mock MCU timeout event."""
        self.timeouts += 1

    def summary(self) -> dict:
        """Produce summary statistics for the entire test run."""
        return {
            "frames": self.frames,
            "detections": self.detections,
            "detection_rate": self.detections / self.frames if self.frames else 0.0,
            "commands_accepted": self.commands_accepted,
            "commands_rejected": self.commands_rejected,
            "mcu_accepted": self.mcu_accepted,
            "mcu_rejected": self.mcu_rejected,
            "timeouts": self.timeouts,
            "avg_detection_latency_ms": _avg(self.detection_latency_ms),
            "p95_detection_latency_ms": _p95(self.detection_latency_ms),
            "avg_mcu_latency_ms": _avg(self.mcu_latency_ms),
            "p95_mcu_latency_ms": _p95(self.mcu_latency_ms),
        }

    def write(self, path: Path) -> None:
        """Write summary and per-frame events as a JSON report."""
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps({"summary": self.summary(), "events": self.events}, indent=2),
            encoding="utf-8",
        )


def _avg(values: list[float]) -> float:
    """Return 0 for empty data to avoid division-by-zero in reports."""
    return mean(values) if values else 0.0


def _p95(values: list[float]) -> float:
    """Estimate p95 latency using simple sorting."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * 0.95))
    return ordered[index]
