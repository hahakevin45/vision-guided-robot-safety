"""R3 Gazebo trace 評估（spec 9）。

trace JSONL（trace_recorder 格式）→ /sim/true_pose 序列 → evaluate_r3_trace
→ eval.json。true_pose 只進 evaluator（spec 4.3）。
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from safety_sim.experiments.r3_geofence import (
    R3TraceOutcome,
    VirtualLine,
    evaluate_r3_trace,
)

ROBOT_RADIUS_M = 0.23
CLEARANCE_REQUIREMENT_M = 0.05


@dataclass(frozen=True)
class R3FileResult:
    run_id: str
    arm: str
    n_pose_samples: int
    outcome: R3TraceOutcome
    passed: bool


def parse_r3_trace(trace_path: Path) -> list[tuple[float, float]]:
    """取出 /sim/true_pose 的 (x, y) 序列；忽略其他 topic。"""
    points: list[tuple[float, float]] = []
    with Path(trace_path).open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if row.get("topic") == "/sim/true_pose":
                p = row["true_pose"]
                points.append((float(p["x"]), float(p["y"])))
    return points


def evaluate_r3_file(trace_path: Path, line_x: float, arm: str,
                     run_id: str) -> R3FileResult:
    """依 arm 判定：sapf_new 須不越線且淨空達標；passthrough 須越線。"""
    points = parse_r3_trace(trace_path)
    line = VirtualLine(p1=(line_x, -10.0), p2=(line_x, 10.0),
                       safe_side_normal=(-1.0, 0.0))
    outcome = evaluate_r3_trace(line, points, robot_radius=ROBOT_RADIUS_M)
    if arm in ("sapf_new", "cbf"):
        passed = (not outcome.crossed
                  and outcome.min_true_clearance_m >= CLEARANCE_REQUIREMENT_M)
    elif arm == "passthrough":
        passed = outcome.crossed
    else:
        raise ValueError(f"unknown R3 arm: {arm}")
    return R3FileResult(run_id=run_id, arm=arm,
                        n_pose_samples=len(points), outcome=outcome,
                        passed=passed)


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    parser = argparse.ArgumentParser(prog="evaluate_r3_trace")
    parser.add_argument("trace", type=Path)
    parser.add_argument("line_x", type=float)
    parser.add_argument("arm", choices=("sapf_new", "cbf", "passthrough"))
    parser.add_argument("--run-id", default="single")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    result = evaluate_r3_file(args.trace, args.line_x, args.arm, args.run_id)
    payload = {
        "run_id": result.run_id,
        "arm": result.arm,
        "n_pose_samples": result.n_pose_samples,
        "crossed": result.outcome.crossed,
        "min_true_clearance_m": result.outcome.min_true_clearance_m,
        "capture_depth_m": result.outcome.capture_depth_m,
    }
    args.out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, sort_keys=True))
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
