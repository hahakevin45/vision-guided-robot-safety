"""Active ArUco field evaluation — no future true-pose alignment.

Evaluates a single run from recorder rows + manifest.
Paired summary compares controlled_adaptive vs controlled_fixed_028
across repeats and checks natural_adaptive end-to-end success.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

# ------------------------------------------------------------------ #
# Geometry                                                             #
# ------------------------------------------------------------------ #

def wall_clearance(
    x: float,
    y: float,
    walls,
    *,
    wall_thickness: float,
    robot_radius: float,
) -> float:
    """Signed clearance from wall edges minus half-wall and robot radius."""
    signed = []
    for (x1, y1), (x2, y2) in zip(walls, (*walls[1:], walls[0])):
        dx, dy = x2 - x1, y2 - y1
        length = (math.hypot(dx, dy))
        if length <= 0.0:
            raise ValueError("wall edge must have positive length")
        nx, ny = -dy / length, dx / length
        signed.append(nx * (x - x1) + ny * (y - y1))
    return min(signed) - wall_thickness / 2.0 - robot_radius


# ------------------------------------------------------------------ #
# Causal alignment helpers                                             #
# ------------------------------------------------------------------ #

def _prior_row(rows: list[dict], t: float) -> dict | None:
    """Return the last row with timestamp <= t (prior-sample lookup)."""
    prior = None
    for row in rows:
        if float(row["t"]) > t:
            break
        prior = row
    return prior


# ------------------------------------------------------------------ #
# Single-run evaluation                                                #
# ------------------------------------------------------------------ #

def evaluate_active_field(rows: list[dict], manifest: dict) -> dict:
    """Evaluate one active ArUco field run.

    Parameters
    ----------
    rows:
        List of trace dicts with topics. Expected topics:
        - /sim/true_pose  → {"t", "true_pose": {"x","y","theta"}}
        - /safety_gate/status → {"t", "mode", "debug": {...}}
        - /aruco/marker_ids   → {"t", "stamp_s", "ids": [...]}
        - /aruco/pose_raw     → {"t", "stamp_s", "pose": {"x","y","theta"}}
        - /aruco/pose         → {"t", "stamp_s", "pose": {"x","y","theta"}}
        - /aruco/dropout_window → {"t", "event", "dropout": bool, "applied_t_s", "pose": {...}}
        - /cmd_vel_safe     → {"t", "twist": {"v", "omega"}}
    manifest:
        Dict with keys: arm, repeat, start_pose, goal, walls,
        wall_thickness_m, robot_radius_m, timeout_sim_s,
        dropout (controlled arms), runtime_failures.

    Returns
    -------
    dict with stable schema.
    """
    # ---- stable schema defaults ------------------------------------ #
    result = {
        "arm": str(manifest.get("arm", "")),
        "repeat": int(manifest.get("repeat", 1)),
        "valid": True,
        "invalid_reasons": [],
        "initial_marker_5_fix_t_s": None,
        "dropout_start_t_s": None,
        "dropout_end_t_s": None,
        "dropout_duration_s": None,
        "dropout_true_path_m": None,
        "first_dead_reckoning_t_s": None,
        "max_blind_dist_m": 0.0,
        "max_true_localization_error_m": 0.0,
        "max_pose_drift_m": 0.0,
        "max_d_safe_m": 0.0,
        "minimum_envelope_excess_m": None,
        "inflation_error_ratio_at_max_error": None,
        "min_true_wall_clearance_m": float("inf"),
        "collision_envelope_violated": False,
        "path_length_m": 0.0,
        "minimum_true_y_m": float("inf"),
        "max_southward_excursion_m": 0.0,
        "motion_start_t_s": None,
        "arrive_t_s": None,
        "time_to_goal_s": None,
        "final_true_goal_distance_m": float("inf"),
        "reached_goal": False,
        "marker_0_reacquire_t_s": None,
        "visibility_to_reacquire_s": None,
        "blind_reset_t_s": None,
        "reacquire_to_reset_s": None,
        "recovered": False,
    }

    def _invalidate(reason: str) -> None:
        result["valid"] = False
        result["invalid_reasons"].append(reason)

    arm = result["arm"]
    wall_thickness = float(manifest.get("wall_thickness_m", 0.05))
    robot_radius = float(manifest.get("robot_radius_m", 0.23))
    goal = manifest.get("goal", {"x": 0.5, "y": 0.3})
    walls = manifest.get("walls", [])
    timeout_sim_s = float(manifest.get("timeout_sim_s", 90.0))

    # Validate recorder order before sorting for causal lookups. Sorting is
    # allowed for metric calculation, but must never repair malformed evidence.
    finite_rows = []
    previous_t = None
    for index, row in enumerate(rows):
        try:
            row_t = float(row["t"])
        except (KeyError, TypeError, ValueError):
            _invalidate(f"missing or invalid timestamp at row {index}")
            continue
        if not math.isfinite(row_t):
            _invalidate(f"non-finite timestamp at row {index}")
            continue
        if previous_t is not None and row_t < previous_t:
            _invalidate(
                f"timestamp regression at row {index}: {row_t} < {previous_t}")
        previous_t = row_t
        finite_rows.append(row)
    rows = sorted(finite_rows, key=lambda r: float(r["t"]))

    # ---- split by topic -------------------------------------------- #
    true_rows = [r for r in rows if r.get("topic") == "/sim/true_pose"]
    status_rows = [r for r in rows if r.get("topic") == "/safety_gate/status"]
    ids_rows = [r for r in rows if r.get("topic") == "/aruco/marker_ids"]
    dropout_rows = [r for r in rows if r.get("topic") == "/aruco/dropout_window"]
    cmd_rows = [r for r in rows if r.get("topic") == "/cmd_vel_safe"]

    # ---- runtime_failures always invalidate ------------------------ #
    if manifest.get("runtime_failures"):
        result["valid"] = False
        result["invalid_reasons"].extend(manifest["runtime_failures"])

    # ---- validity: required fields must be finite for blind statuses #
    # Only statuses actually reporting dead-reckoning / blind estimates
    # need estimated pose, drift, d_safe and blind distance. Startup STOP
    # rows (e.g. reason "missing_pose") precede the initial marker fix and
    # legitimately lack these fields, so they must not invalidate a run.
    for r in status_rows:
        debug = r.get("debug", {})
        if float(debug.get("dead_reckoning", 0)) < 0.5:
            continue
        required = ("estimated_x", "estimated_y", "pose_drift_m",
                    "d_safe_m", "blind_dist_m")
        for f in required:
            v = debug.get(f)
            if v is None or not math.isfinite(v):
                result["valid"] = False
                result["invalid_reasons"].append(
                    f"non-finite {f} at t={r.get('t')} [{r.get('topic')}]")

    # ---- initial marker 5 fix (before first non-zero cmd) ---------- #
    first_nonzero_cmd_t = None
    for r in cmd_rows:
        twist = r.get("twist", {})
        if float(twist.get("v", 0.0)) != 0.0:
            first_nonzero_cmd_t = float(r["t"])
            break

    if first_nonzero_cmd_t is not None:
        for r in ids_rows:
            ids = r.get("ids", [])
            if 5 in ids and float(r["t"]) < first_nonzero_cmd_t:
                result["initial_marker_5_fix_t_s"] = float(r["t"])
                break

    # ---- controlled arm: parse dropout_window events ---------------- #
    is_controlled = arm in ("controlled_adaptive", "controlled_fixed_028")
    dropout_cfg = manifest.get("dropout", {})
    dropout_enabled = bool(dropout_cfg.get("enabled", True)) and is_controlled

    if dropout_enabled:
        for r in dropout_rows:
            if r.get("dropout") is True:
                result["dropout_start_t_s"] = float(r.get("applied_t_s", r["t"]))
            elif r.get("dropout") is False and result["dropout_end_t_s"] is None:
                result["dropout_end_t_s"] = float(r.get("applied_t_s", r["t"]))

        if (result["dropout_start_t_s"] is not None
                and result["dropout_end_t_s"] is not None):
            dur = result["dropout_end_t_s"] - result["dropout_start_t_s"]
            result["dropout_duration_s"] = max(0.0, dur)

    # ---- natural arm: derive dropout from empty ID sequence --------- #
    is_natural = arm == "natural_adaptive"
    if is_natural:
        marker_5_t = result["initial_marker_5_fix_t_s"]
        if marker_5_t is not None:
            post5_ids = [r for r in ids_rows
                         if float(r["t"]) > marker_5_t]
            for r in post5_ids:
                if not r.get("ids"):
                    result["dropout_start_t_s"] = float(r.get("stamp_s", r["t"]))
                    break
            for r in post5_ids:
                if 0 in r.get("ids", []):
                    result["dropout_end_t_s"] = float(r.get("stamp_s", r["t"]))
                    result["marker_0_reacquire_t_s"] = result["dropout_end_t_s"]
                    break
            if (result["dropout_start_t_s"] is not None
                    and result["dropout_end_t_s"] is not None):
                dur = result["dropout_end_t_s"] - result["dropout_start_t_s"]
                result["dropout_duration_s"] = dur if dur > 0.4 else None

    # ---- evidence integrity: validity is about evidence, not outcome #
    # Timeout / collision / STOP / failure-to-goal are all VALID
    # outcomes; only missing required evidence makes a run invalid.

    # All runs: an accepted marker 5 fix must precede any motion.
    if (first_nonzero_cmd_t is not None
            and result["initial_marker_5_fix_t_s"] is None):
        _invalidate("missing initial accepted marker 5 before motion")

    # Controlled arms: exactly a start then end dropout transition.
    if dropout_enabled:
        if result["dropout_start_t_s"] is None:
            _invalidate("missing controlled dropout start transition")
        if result["dropout_end_t_s"] is None:
            _invalidate("missing controlled dropout end transition")
        if (result["dropout_start_t_s"] is not None
                and result["dropout_end_t_s"] is not None
                and result["dropout_end_t_s"] <= result["dropout_start_t_s"]):
            _invalidate("dropout transitions out of order "
                        "(end must follow start)")

    # Natural arm: empty accepted-ID gap strictly > 0.4 s plus marker 0.
    if is_natural:
        if result["dropout_start_t_s"] is None:
            _invalidate("missing natural dropout gap (no empty accepted IDs)")
        if result["marker_0_reacquire_t_s"] is None:
            _invalidate("missing marker 0 reacquisition")
        if (result["dropout_start_t_s"] is not None
                and result["dropout_end_t_s"] is not None):
            dur = result["dropout_end_t_s"] - result["dropout_start_t_s"]
            if dur <= 0.4:
                _invalidate(
                    f"natural dropout gap {dur:.3f}s not strictly > 0.4s")

    # ---- first dead_reckoning status inside this outage ------------ #
    blind_start_t = result["dropout_start_t_s"]
    for r in status_rows:
        row_t = float(r["t"])
        if blind_start_t is not None and row_t < blind_start_t:
            continue
        dr = r.get("debug", {}).get("dead_reckoning", 0)
        if float(dr) >= 0.5:
            result["first_dead_reckoning_t_s"] = row_t
            break
    if (result["dropout_start_t_s"] is not None
            and result["dropout_end_t_s"] is not None
            and result["first_dead_reckoning_t_s"] is None):
        _invalidate("pose outage never activated dead reckoning")

    # ---- path metrics (causal alignment via _prior_row) ------------ #
    motion_start_t = None
    arrive_t = None
    goal_x, goal_y = goal.get("x", 0.5), goal.get("y", 0.3)
    GOAL_TOLERANCE = 0.05
    TWIST_ZERO_TOLERANCE = 1e-6
    MAX_COMMAND_GAP_S = 0.2

    def _has_zero_twist_dwell(start_t: float) -> bool:
        dwell_end_t = start_t + 1.0
        commands = [row for row in cmd_rows if float(row["t"]) >= start_t]
        checked = []
        for row in commands:
            checked.append(row)
            if float(row["t"]) >= dwell_end_t:
                break
        if not checked or float(checked[-1]["t"]) < dwell_end_t:
            return False
        sample_times = [start_t, *(float(row["t"]) for row in checked)]
        if any(
            later - earlier > MAX_COMMAND_GAP_S + 1e-9
            for earlier, later in zip(sample_times, sample_times[1:])
        ):
            return False
        return all(
            abs(float(row.get("twist", {}).get("v", 0.0)))
            <= TWIST_ZERO_TOLERANCE
            and abs(float(row.get("twist", {}).get("omega", 0.0)))
            <= TWIST_ZERO_TOLERANCE
            for row in checked
        )

    path_points: list[tuple[float, float, float]] = []
    clearances: list[float] = []
    southward_vals: list[float] = []

    for tr in true_rows:
        t = float(tr["t"])
        pose = tr.get("true_pose", {})
        x, y = float(pose["x"]), float(pose["y"])

        if motion_start_t is None:
            if first_nonzero_cmd_t is not None and t < first_nonzero_cmd_t:
                continue
            motion_start_t = t

        if motion_start_t is not None and t - motion_start_t > timeout_sim_s:
            break

        path_points.append((t, x, y))
        clearances.append(wall_clearance(x, y, walls,
                                         wall_thickness=wall_thickness,
                                         robot_radius=robot_radius))
        southward_vals.append(abs(y - goal_y))

        # Goal completion requires both geometric arrival and an uninterrupted
        # one-second zero-twist dwell beginning at that true-pose sample.
        dist_to_goal = math.hypot(x - goal_x, y - goal_y)
        if (arrive_t is None
                and dist_to_goal <= GOAL_TOLERANCE
                and _has_zero_twist_dwell(t)):
            arrive_t = t

    # path length
    path_length = 0.0
    for i in range(1, len(path_points)):
        path_length += math.hypot(
            path_points[i][1] - path_points[i - 1][1],
            path_points[i][2] - path_points[i - 1][2],
        )

    result["motion_start_t_s"] = motion_start_t
    result["arrive_t_s"] = arrive_t
    result["path_length_m"] = path_length

    if path_points:
        result["min_true_wall_clearance_m"] = min(clearances) if clearances else float("inf")
        result["minimum_true_y_m"] = min(y for _, _, y in path_points)
        result["max_southward_excursion_m"] = (
            max(southward_vals) if southward_vals else 0.0)
        fx, fy = path_points[-1][1], path_points[-1][2]
        result["final_true_goal_distance_m"] = math.hypot(fx - goal_x, fy - goal_y)
        result["collision_envelope_violated"] = any(c < 0.0 for c in clearances)
        min_c = min(clearances) if clearances else float("inf")
    else:
        result["min_true_wall_clearance_m"] = float("inf")

    # ---- blind error and inflation metrics (causal status->true alignment) #
    # Align each blind status to the latest true pose with t <= status.t,
    # never aligning true rows to a later/prior status.
    blind_dists: list[float] = []
    pose_drifts: list[float] = []
    d_safes: list[float] = []
    errors: list[float] = []
    envelope_excesses: list[float] = []

    for st in status_rows:
        t = float(st["t"])
        debug = st.get("debug", {})
        dr = float(debug.get("dead_reckoning", 0))
        if dr < 0.5:
            continue
        prior_true = _prior_row(true_rows, t)
        if prior_true is None:
            continue

        true_pose = prior_true.get("true_pose", {})
        tx = float(true_pose.get("x", 0.0))
        ty = float(true_pose.get("y", 0.0))
        ex = float(debug.get("estimated_x", tx))
        ey = float(debug.get("estimated_y", ty))
        error = math.hypot(ex - tx, ey - ty)
        errors.append(error)

        blind_d = float(debug.get("blind_dist_m", 0.0))
        pose_d = float(debug.get("pose_drift_m", 0.0))
        d_s = float(debug.get("d_safe_m", 0.0))
        blind_dists.append(blind_d)
        pose_drifts.append(pose_d)
        d_safes.append(d_s)

        envelope_excesses.append((d_s - 0.28) - error)

    if blind_dists:
        result["max_blind_dist_m"] = max(blind_dists)
    if pose_drifts:
        result["max_pose_drift_m"] = max(pose_drifts)
    if d_safes:
        result["max_d_safe_m"] = max(d_safes)
    if errors:
        result["max_true_localization_error_m"] = max(errors)
    if envelope_excesses:
        result["minimum_envelope_excess_m"] = min(envelope_excesses)

    # inflation error ratio at max error
    if errors and d_safes:
        max_err = max(errors)
        max_idx = errors.index(max_err)
        if d_safes[max_idx] > 0:
            result["inflation_error_ratio_at_max_error"] = (
                max_err / d_safes[max_idx])

    # ---- goal completion ------------------------------------------- #
    if arrive_t is not None and motion_start_t is not None:
        result["time_to_goal_s"] = arrive_t - motion_start_t
        result["reached_goal"] = True

    # ---- marker-0 reacquisition ------------------------------------ #
    if dropout_enabled:
        reopen_t = result["dropout_end_t_s"]
        for r in ids_rows:
            marker_t = float(r.get("stamp_s", r["t"]))
            if (reopen_t is not None and marker_t >= reopen_t
                    and 0 in r.get("ids", [])):
                result["marker_0_reacquire_t_s"] = marker_t
                break

        if (result["marker_0_reacquire_t_s"] is not None
                and result["dropout_start_t_s"] is not None):
            result["visibility_to_reacquire_s"] = (
                result["marker_0_reacquire_t_s"]
                - result["dropout_start_t_s"])

    # A recovery is real only after this outage entered dead reckoning and a
    # post-marker-0 status reports the complete blind-state reset.
    reacquire_t = result["marker_0_reacquire_t_s"]
    if (reacquire_t is not None
            and result["first_dead_reckoning_t_s"] is not None):
        for r in status_rows:
            row_t = float(r["t"])
            if row_t < reacquire_t:
                continue
            debug = r.get("debug", {})
            dr = float(debug.get("dead_reckoning", 1.0))
            bd = debug.get("blind_dist_m")
            pd = debug.get("pose_drift_m")
            ds = debug.get("d_safe_m")
            if any(v is None for v in (bd, pd, ds)):
                continue
            bd, pd, ds = float(bd), float(pd), float(ds)
            if (dr < 0.5 and bd <= 1e-6 and pd <= 1e-6
                    and abs(ds - 0.28) <= 1e-6):
                result["blind_reset_t_s"] = row_t
                break

    if (reacquire_t is not None and result["blind_reset_t_s"] is not None):
        result["reacquire_to_reset_s"] = (
            result["blind_reset_t_s"] - reacquire_t)
        result["recovered"] = True

    # ---- dropout true path ----------------------------------------- #
    if (result["dropout_start_t_s"] is not None
            and result["dropout_end_t_s"] is not None
            and path_points):
        ds_t = result["dropout_start_t_s"]
        de_t = result["dropout_end_t_s"]
        seg = [(t, x, y) for t, x, y in path_points if ds_t <= t <= de_t]
        seg_len = 0.0
        for i in range(1, len(seg)):
            seg_len += math.hypot(
                seg[i][1] - seg[i - 1][1], seg[i][2] - seg[i - 1][2])
        result["dropout_true_path_m"] = seg_len

    return result


# ------------------------------------------------------------------ #
# Paired run comparison                                                #
# ------------------------------------------------------------------ #

def compare_active_field_runs(results: list[dict]) -> dict:
    """Summarise a batch of evaluated runs.

    Builds controlled (adaptive vs fixed) pairs per repeat and
    evaluates adaptive-clearance and natural end-to-end claims.
    """
    by_key = {(row["arm"], int(row["repeat"])): row for row in results}
    repeats = sorted({int(row["repeat"]) for row in results})
    pairs = []
    for repeat in repeats:
        adaptive = by_key.get(("controlled_adaptive", repeat))
        fixed = by_key.get(("controlled_fixed_028", repeat))
        if adaptive is None or fixed is None:
            continue
        delta_time = (
            adaptive["time_to_goal_s"] - fixed["time_to_goal_s"]
            if adaptive["reached_goal"] and fixed["reached_goal"] else None)
        pairs.append({
            "repeat": repeat,
            "min_clearance_delta_m": (
                adaptive["min_true_wall_clearance_m"]
                - fixed["min_true_wall_clearance_m"]),
            "southward_excursion_delta_m": (
                adaptive["max_southward_excursion_m"]
                - fixed["max_southward_excursion_m"]),
            "path_length_delta_m": (
                adaptive["path_length_m"] - fixed["path_length_m"]),
            "time_to_goal_delta_s": delta_time,
            "max_error_delta_m": (
                adaptive["max_true_localization_error_m"]
                - fixed["max_true_localization_error_m"]),
        })

    def _accept(row):
        return (row and row["valid"]
                and not row["collision_envelope_violated"]
                and row["recovered"] and row["reached_goal"])

    adaptive_present = any(("controlled_adaptive", r) in by_key
                           for r in repeats)
    fixed_present = any(("controlled_fixed_028", r) in by_key
                        for r in repeats)
    natural_present = any(("natural_adaptive", r) in by_key
                          for r in repeats)

    adaptive_accept = all(
        _accept(by_key[("controlled_adaptive", r)])
        for r in repeats if ("controlled_adaptive", r) in by_key)
    fixed_accept = all(
        _accept(by_key[("controlled_fixed_028", r)])
        for r in repeats if ("controlled_fixed_028", r) in by_key)
    natural_accept = all(
        _accept(by_key[("natural_adaptive", r)])
        for r in repeats if ("natural_adaptive", r) in by_key)

    # Both controlled arms must be present and pass; at least one pair
    # with positive clearance delta is required. No hard-coded repeat gate.
    superiority = (
        adaptive_present
        and fixed_present
        and adaptive_accept
        and fixed_accept
        and len(pairs) >= 1
        and all(p["min_clearance_delta_m"] > 0.0 for p in pairs))

    return {
        "runs": results,
        "controlled_pairs": pairs,
        "adaptive_clearance_claim": superiority,
        "natural_end_to_end_claim": natural_present and natural_accept,
        "scenario_solved": superiority
        and natural_present
        and natural_accept,
    }


# ------------------------------------------------------------------ #
# CLI                                                                  #
# ------------------------------------------------------------------ #

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate active ArUco field trace(s).")
    parser.add_argument("--trace", type=str, help="Path to trace.jsonl")
    parser.add_argument("--manifest", type=str, help="Path to manifest.json")
    parser.add_argument("--output", type=str, help="Path to write evaluation.json")
    args = parser.parse_args(argv)

    if args.trace and args.manifest:
        rows = []
        trace_path = Path(args.trace)
        if trace_path.exists():
            for line in trace_path.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    rows.append(json.loads(line))
        manifest = {}
        manifest_path = Path(args.manifest)
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

        out = evaluate_active_field(rows, manifest)
        if args.output:
            Path(args.output).write_text(
                json.dumps(out, indent=2), encoding="utf-8")
        else:
            print(json.dumps(out, indent=2))
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
