"""安全與活性指標。全部吃 Trace（含 ground truth），不吃 Observation。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .runner import Trace

STOP_SPEED_EPS = 0.005      # m/s，低於此視為停止
# 把 omega 差異換算成輪速差的等效線速度（半輪距），讓 cmd_distortion 單位一致。
OMEGA_WEIGHT_M = 0.0825


def collided(trace: "Trace") -> bool:
    return any(s.clearance < 0.0 for s in trace.samples)


def min_clearance(trace: "Trace") -> float:
    return min(s.clearance for s in trace.samples)


def max_speed(trace: "Trace") -> float:
    return max(abs(s.actual_twist.v) for s in trace.samples)


def time_to_stop_after(trace: "Trace", t0: float | None) -> float:
    """t0 之後第一次完全停止的耗時；沒停過回傳 inf。"""
    if t0 is None:
        return math.inf
    for s in trace.samples:
        if s.t >= t0 and abs(s.actual_twist.v) < STOP_SPEED_EPS:
            return s.t - t0
    return math.inf


def intervention_ratio(trace: "Trace") -> float:
    n = len(trace.samples)
    return sum(1 for s in trace.samples if s.mode != "PASS") / n if n else 0.0


def cmd_distortion(trace: "Trace") -> float:
    """∫ ||filtered − desired||² dt，omega 以半輪距換算成等效線速度。"""
    total = 0.0
    for prev, cur in zip(trace.samples, trace.samples[1:]):
        dt = cur.t - prev.t
        dv = prev.cmd.v - prev.desired.v
        dw = (prev.cmd.omega - prev.desired.omega) * OMEGA_WEIGHT_M
        total += (dv * dv + dw * dw) * dt
    return total


@dataclass(frozen=True)
class MetricsReport:
    collided: bool
    min_clearance: float
    max_speed_mps: float
    time_to_stop_after_fault_s: float
    intervention_ratio: float
    cmd_distortion: float


def summarize(trace: "Trace", fault_t0: float | None = None) -> MetricsReport:
    return MetricsReport(
        collided=collided(trace),
        min_clearance=min_clearance(trace),
        max_speed_mps=max_speed(trace),
        time_to_stop_after_fault_s=time_to_stop_after(trace, fault_t0),
        intervention_ratio=intervention_ratio(trace),
        cmd_distortion=cmd_distortion(trace),
    )
