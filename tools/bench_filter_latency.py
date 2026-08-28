"""Benchmark single-call filter() latency for safe_apf, cbf, gf_dwa.

Synthesises an Observation stream covering near-wall, far-wall, diagonal,
obstacle, stale-pose, and no-pose scenarios.  Each filter receives ≥2000
filter() calls.

Outputs a markdown table to stdout and (optionally) to --out FILE.

Usage:
    python3 tools/bench_filter_latency.py [--num-calls N] [--out FILE]
"""
from __future__ import annotations

import argparse
import math
import socket
import statistics
import sys
import time

from vgr_core.motion import DiffDriveParams, twist_to_wheel_counts
from safety_sim.filters import make_filter
from vgr_core.safety import Circle, Observation, Pose, StaticInfo, Twist

# ── shared geometry (mirrors test_filter_*.py) ──────────────────────

FENCE = ((0.0, -1.0), (4.0, -1.0), (4.0, 1.0), (0.0, 1.0))
STATIC = StaticInfo(
    params=DiffDriveParams(),
    robot_radius_m=0.10,
    geofence=FENCE,
    max_v_mps=0.15,
    max_omega_rad_s=1.5,
)

# ── synthetic command stream ────────────────────────────────────────

_COMMANDS = (
    Twist(0.15, 0.0),
    Twist(0.10, 0.0),
    Twist(0.05, 0.0),
    Twist(0.0, 0.0),
    Twist(-0.10, 0.0),
    Twist(0.10, 0.5),
    Twist(0.10, -0.5),
    Twist(0.0, 1.0),
    Twist(0.0, -1.0),
)

def _scenarios():
    """Yield (pose, pose_age_s, obstacles, link_age_s, wheel_feedback) tuples."""
    fb_stand = (0.0, 0.0)
    fb_fwd = twist_to_wheel_counts(0.15, 0.0, STATIC.params)
    fb_turn = twist_to_wheel_counts(0.10, 0.5, STATIC.params)

    scenarios = (
        # far from any wall
        (Pose(1.0, 0.0, 0.0), 0.0, (), 0.0, fb_fwd),
        (Pose(2.0, 0.5, 0.3), 0.0, (), 0.0, fb_fwd),
        # near right wall  (x ≈ 4.0)
        (Pose(3.86, 0.0, 0.0), 0.0, (), 0.0, fb_stand),
        (Pose(3.80, -0.5, 0.2), 0.0, (), 0.0, fb_stand),
        # near rear wall  (x ≈ 0.0)
        (Pose(0.14, 0.0, math.pi), 0.0, (), 0.0, fb_stand),
        # near front wall  (y ≈ 1.0)
        (Pose(1.0, 0.86, math.pi / 2), 0.0, (), 0.0, fb_stand),
        # near bottom wall  (y ≈ -1.0)
        (Pose(1.0, -0.86, -math.pi / 2), 0.0, (), 0.0, fb_stand),
        # diagonal approach to top-right corner
        (Pose(3.70, 0.80, math.pi / 4), 0.0, (), 0.0, fb_stand),
        (Pose(3.60, 0.70, 0.5), 0.0, (), 0.0, fb_stand),
        # circle obstacles
        (Pose(2.0, 0.0, 0.0), 0.0, (Circle(2.5, 0.0, 0.15),), 0.0, fb_stand),
        (Pose(1.5, 0.3, 0.0), 0.0, (Circle(1.8, 0.0, 0.15), Circle(2.2, 0.5, 0.15)), 0.0, fb_stand),
        # turning feedback
        (Pose(2.0, 0.0, 0.0), 0.0, (), 0.0, fb_turn),
        # stale pose
        (Pose(2.0, 0.0, 0.0), 0.4, (), 0.0, fb_stand),
        # no pose
        (None, math.inf, (), 0.0, fb_stand),
        # angled away from wall
        (Pose(3.80, 0.0, math.pi), 0.0, (), 0.0, fb_stand),
        (Pose(3.84, 0.5, math.pi / 2), 0.0, (), 0.0, fb_stand),
        (Pose(3.84, -0.5, -math.pi / 2), 0.0, (), 0.0, fb_stand),
        # centre, slight offset
        (Pose(2.0, -0.3, -0.2), 0.0, (), 0.0, fb_fwd),
        # near wall with drift
        (Pose(3.80, 0.0, 0.0), 0.0, (), 0.0, fb_stand),
    )
    return scenarios


# ── benchmark core ──────────────────────────────────────────────────

def run_benchmark(filter_name: str, num_calls: int = 2000) -> dict:
    """Measure filter() latency over *num_calls* synthetic observations."""
    filt = make_filter(filter_name)
    filt.reset(STATIC)

    scenes = _scenarios()
    latencies: list[float] = []
    t = 0.0
    dt = 0.05

    for i in range(num_calls):
        pose, pa, obst, link, wf = scenes[i % len(scenes)]
        obs = Observation(
            pose=pose,
            pose_age_s=pa,
            wheel_feedback=wf,
            obstacles=obst,
            link_age_s=link,
        )
        cmd = _COMMANDS[i % len(_COMMANDS)]

        t0 = time.perf_counter()
        filt.filter(cmd, obs, t, dt)
        latencies.append(time.perf_counter() - t0)
        t += dt

    latencies.sort()
    n = len(latencies)
    return {
        "count": n,
        "min_us": latencies[0] * 1e6,
        "median_us": statistics.median(latencies) * 1e6,
        "p95_us": latencies[int(n * 0.95)] * 1e6,
        "p99_us": latencies[int(n * 0.99)] * 1e6,
        "max_us": latencies[-1] * 1e6,
    }


def format_table(results: dict[str, dict], host: str = "") -> str:
    lines = [
        "# Filter Latency Benchmark",
        "",
        f"| Filter | Count | Median (μs) | P95 (μs) | P99 (μs) | Max (μs) |",
        f"|--------|-------|-------------|----------|----------|----------|",
    ]
    for name in ("safe_apf", "cbf", "gf_dwa"):
        r = results[name]
        lines.append(
            f"| {name} | {r['count']} | {r['median_us']:.1f} |"
            f" {r['p95_us']:.1f} | {r['p99_us']:.1f} | {r['max_us']:.1f} |"
        )
    lines.append("")
    lines.append(f"_Measured on: {host or 'current machine'}_")
    return "\n".join(lines)


# ── CLI ─────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="Benchmark filter latency")
    ap.add_argument("--out", type=str, default=None, help="Write markdown to FILE")
    ap.add_argument(
        "--num-calls", type=int, default=2000,
        help="Filter() calls per filter (default: 2000)",
    )
    args = ap.parse_args()

    filter_names = ("safe_apf", "cbf", "gf_dwa")
    results: dict[str, dict] = {}

    for name in filter_names:
        sys.stderr.write(f"Benchmarking {name} ... ")
        sys.stderr.flush()
        r = run_benchmark(name, args.num_calls)
        results[name] = r
        sys.stderr.write(
            f"median={r['median_us']:.1f}μs  p95={r['p95_us']:.1f}μs"
            f"  max={r['max_us']:.1f}μs\n"
        )

    host = socket.gethostname()
    table = format_table(results, host=host)
    print(table)

    if args.out:
        with open(args.out, "w") as f:
            f.write(table + "\n")
        sys.stderr.write(f"Written to {args.out}\n")


if __name__ == "__main__":
    main()
