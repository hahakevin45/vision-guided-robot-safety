"""R3 虛擬 geofence evaluator（spec 9）。

Ground truth 只進這裡：虛擬線幾何 + 真實 stop/軌跡量測 → signed distance、
crossing、capture depth、min true clearance。filter/controller 永遠看不到這些。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VirtualLine:
    p1: tuple[float, float]
    p2: tuple[float, float]
    safe_side_normal: tuple[float, float]   # 指向安全側的單位法線


def _unit(nx: float, ny: float) -> tuple[float, float]:
    norm = math.hypot(nx, ny)
    if norm <= 0.0:
        raise ValueError("normal must be non-zero")
    return nx / norm, ny / norm


def signed_distance_to_line(point: tuple[float, float], line: VirtualLine) -> float:
    """有號距離：安全側為正。"""
    x, y = point
    nx, ny = _unit(*line.safe_side_normal)
    ex, ey = line.p2[0] - line.p1[0], line.p2[1] - line.p1[1]
    # 線上最近點（clamped projection）
    denom = ex * ex + ey * ey
    t = 0.0 if denom == 0.0 else min(
        1.0, max(0.0, ((x - line.p1[0]) * ex + (y - line.p1[1]) * ey) / denom))
    cx, cy = line.p1[0] + t * ex, line.p1[1] + t * ey
    return nx * (x - cx) + ny * (y - cy)


@dataclass(frozen=True)
class R3StopOutcome:
    crossed: bool
    min_true_clearance_m: float     # 車心 signed distance − robot radius
    capture_depth_m: float | None   # 越線深度（footprint），未越線為 None
    stop_signed_m: float


@dataclass(frozen=True)
class R3TraceOutcome:
    crossed: bool
    min_true_clearance_m: float
    capture_depth_m: float | None
    max_cross_signed_m: float


def evaluate_r3_stop(line: VirtualLine, stop_point: tuple[float, float],
                     robot_radius: float) -> R3StopOutcome:
    if not (math.isfinite(stop_point[0]) and math.isfinite(stop_point[1])):
        raise ValueError("stop point must be finite")
    signed = signed_distance_to_line(stop_point, line)
    clearance = signed - robot_radius
    crossed = clearance < 0.0
    return R3StopOutcome(
        crossed=crossed,
        min_true_clearance_m=clearance,
        capture_depth_m=(-clearance if crossed else None),
        stop_signed_m=signed,
    )


def evaluate_r3_trace(line: VirtualLine, points: list[tuple[float, float]],
                      robot_radius: float) -> R3TraceOutcome:
    if not points:
        raise ValueError("trace must not be empty")
    signed_vals: list[float] = []
    for p in points:
        if not (math.isfinite(p[0]) and math.isfinite(p[1])):
            raise ValueError("trace point must be finite")
        signed_vals.append(signed_distance_to_line(p, line))
    min_signed = min(signed_vals)
    clearance = min_signed - robot_radius
    crossed = clearance < 0.0
    max_cross = max(-v for v in signed_vals if v < robot_radius) if any(
        v < robot_radius for v in signed_vals) else 0.0
    return R3TraceOutcome(
        crossed=crossed,
        min_true_clearance_m=clearance,
        capture_depth_m=(max_cross - robot_radius if crossed else None),
        max_cross_signed_m=max_cross,
    )


@dataclass(frozen=True)
class R3Aggregate:
    n_sapf: int
    n_passthrough: int
    sapf_passed: bool       # 10/10 不越線且 clearance ≥ 需求
    passthrough_crossed: bool  # 3/3 越線


def aggregate_r3(sapf_outcomes: list[R3StopOutcome | R3TraceOutcome],
                 passthrough_outcomes: list[R3StopOutcome | R3TraceOutcome],
                 *, clearance_requirement_m: float) -> R3Aggregate:
    """spec 9.3：SAPF-new 10/10 不越線且保持共同 clearance；passthrough 3/3 越線。"""
    sapf_ok = (
        len(sapf_outcomes) >= 10
        and all(not o.crossed and o.min_true_clearance_m >= clearance_requirement_m
                for o in sapf_outcomes)
    )
    passthrough_ok = (
        len(passthrough_outcomes) >= 3
        and all(o.crossed for o in passthrough_outcomes)
    )
    return R3Aggregate(
        n_sapf=len(sapf_outcomes),
        n_passthrough=len(passthrough_outcomes),
        sapf_passed=sapf_ok,
        passthrough_crossed=passthrough_ok,
    )
