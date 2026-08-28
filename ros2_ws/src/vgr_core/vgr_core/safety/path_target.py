"""Nav2 `/plan` lookahead target selection for the SAPF goal contract.

The SAPF attractive target `q*` is a point on the Nav2 global path a fixed
path distance ahead of the robot. Selection starts at the path point nearest
to the current pose and accumulates forward only, so the target never moves
backward behind the robot; when less than `lookahead_m` of path remains, the
final point is used.
"""
from __future__ import annotations

import math


def select_path_lookahead(
    points: tuple[tuple[float, float], ...],
    pose_xy: tuple[float, float],
    lookahead_m: float,
) -> tuple[float, float] | None:
    """Return the lookahead point, or None when no valid selection exists."""
    if not points or lookahead_m <= 0.0:
        return None
    best_i, best_d = -1, math.inf
    for i, (px, py) in enumerate(points):
        if not (math.isfinite(px) and math.isfinite(py)):
            return None
        d = math.hypot(px - pose_xy[0], py - pose_xy[1])
        if d < best_d:
            best_d, best_i = d, i
    acc = 0.0
    for i in range(best_i, len(points) - 1):
        seg = math.hypot(points[i + 1][0] - points[i][0],
                         points[i + 1][1] - points[i][1])
        if acc + seg >= lookahead_m:
            return points[i + 1]
        acc += seg
    return points[-1]
