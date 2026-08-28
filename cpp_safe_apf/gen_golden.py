#!/usr/bin/env python3
"""Generate golden.json — differential-test fixture for the C++ SafeApfFilter port.

Drives the REAL Python reference (safety_sim.filters.safe_apf.SafeApfFilter)
through a deterministic, RNG-free case list and serializes each case's inputs
and expected outputs. The C++ harness (main.cpp) replays every case and must
match mode exactly and v/omega to within 1e-9 absolute.

Environment note: safety_sim.types imports vgr_core.motion; vgr_core ships in
this repo under ros2_ws/install/vgr_core/lib/python3.10/site-packages, which
this script adds to sys.path. No existing repo files are modified.

Notes:
- Case 10 (force cancellation): the reference's `_apf_command` never returns
  None — the `cmd is None -> STOP` branch in `filter()` is dead code — so the
  norm < 1e-9 cancellation path yields cmd=(0,0) with mode MODIFIED. The golden
  encodes the reference's *actual* behavior; the case still pins the
  cancellation path exactly.
- S1-S7 one-tick cases: the scenario definitions are importable, so all seven
  are included (no skips). Each carries a per-case "static" override matching
  its own geofence / robot radius / speed limits, mirroring what
  safety_sim.runner.run_scenario feeds the filter.

JSON schema:
{
  "static": {"geofence": [[x,y],...], "robot_radius_m": r,
             "max_v_mps": v, "max_omega_rad_s": w},
  "cases": [{
    "name": str,
    "desired": [v, omega],
    "obs": [x, y, theta, pose_age_s, link_age_s, pose_drift_m, wheel_l, wheel_r],
    "t": float, "dt": float,
    "expected": {"mode": "PASS|MODIFIED|STOP", "v": ..., "omega": ...},
    // optional:
    "no_pose": true,      // x/y/theta ignored; filter sees pose=None
    "static": {...}       // per-case StaticInfo override (S1-S7, triangles)
  }]
}
"""
import json
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "install" / "vgr_core"
                        / "lib" / "python3.10" / "site-packages"))

from safety_sim.filters.safe_apf import SafeApfFilter
from safety_sim.types import Observation, Pose, StaticInfo, Twist

# ---------------------------------------------------------------------------
# Static configurations
# ---------------------------------------------------------------------------

TRAPEZOID = ((-0.08, -0.59), (2.24, -0.61), (2.27, 1.72), (0.34, 1.74))
TRIANGLE = ((0.0, 0.0), (2.0, 0.0), (0.0, 2.0))
TWO_POINT = ((0.0, 0.0), (1.0, 1.0))


def make_static(geofence, radius=0.20, max_v=0.15, max_omega=1.5):
    return StaticInfo(params=None, robot_radius_m=radius,
                      geofence=tuple(tuple(p) for p in geofence),
                      max_v_mps=max_v, max_omega_rad_s=max_omega)


STATIC = make_static(TRAPEZOID)
STATIC_TRIANGLE = make_static(TRIANGLE)
STATIC_TWO_POINT = make_static(TWO_POINT)


def static_to_json(s):
    return {"geofence": [[x, y] for x, y in s.geofence],
            "robot_radius_m": s.robot_radius_m,
            "max_v_mps": s.max_v_mps,
            "max_omega_rad_s": s.max_omega_rad_s}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_obs(pose, pose_age=0.0, link_age=0.0, drift=0.0, wheel=(0.0, 0.0)):
    return Observation(pose=pose, pose_age_s=pose_age, wheel_feedback=wheel,
                       link_age_s=link_age, pose_drift_m=drift)


def apf_theta_error(f, static, pose, desired, d_safe, signed_speed):
    """theta_error the reference _apf_command would see, or None if the total
    force norm is below the 1e-9 cancellation threshold."""
    walls = f._wall_distances(pose, static)
    travel_theta = pose.theta if signed_speed > 0 else pose.theta + math.pi
    fx = abs(desired[0]) * math.cos(travel_theta)
    fy = abs(desired[0]) * math.sin(travel_theta)
    influence = max(f._influence, d_safe + 1e-6)
    for d, nx, ny in walls:
        if d >= influence:
            continue
        if d <= d_safe:
            strength = static.max_v_mps
        else:
            rel = (influence - d) / (influence - d_safe)
            strength = static.max_v_mps * rel * rel
        fx += strength * nx
        fy += strength * ny
    if math.hypot(fx, fy) < 1e-9:
        return None
    return f._wrap(math.atan2(fy, fx) - travel_theta)


def run_case(name, desired, pose=None, pose_age=0.0, link_age=0.0, drift=0.0,
             t=0.0, dt=0.05, static=None, no_pose=False, check=None):
    static = static or STATIC
    filt = SafeApfFilter()
    filt.reset(static)
    o = make_obs(None if no_pose else Pose(*pose), pose_age, link_age, drift)
    dec = filt.filter(Twist(*desired), o, t, dt)
    if check is not None:
        check(filt, static, o, desired, dec)
    case = {
        "name": name,
        "desired": [desired[0], desired[1]],
        "obs": ([0.0, 0.0, 0.0] if no_pose else [pose[0], pose[1], pose[2]])
               + [pose_age, link_age, drift, 0.0, 0.0],
        "t": t,
        "dt": dt,
        "expected": {"mode": dec.mode, "v": dec.cmd.v, "omega": dec.cmd.omega},
    }
    if no_pose:
        case["no_pose"] = True
    if static is not STATIC:
        case["static"] = static_to_json(static)
    return case


# ---------------------------------------------------------------------------
# Mandatory hand-built cases
# ---------------------------------------------------------------------------

cases = []


def chk_open_area(f, s, o, d, dec):
    assert dec.mode == "PASS", dec.mode
    assert dec.cmd.v == d[0] and dec.cmd.omega == d[1]


cases.append(run_case("open_area_straight", (0.10, 0.0), (1.0, 0.5, 0.0),
                      check=chk_open_area))


def chk_approach_wall(f, s, o, d, dec):
    # Pose 1.45m above bottom-left corner, heading +y toward the top wall.
    assert dec.mode == "MODIFIED", dec.mode
    assert 0.0 < dec.cmd.v < d[0] - 1e-9, (dec.cmd.v, d[0])
    # CBF speed governor must be active and biting: allowed < desired.v.
    travel_theta = o.pose.theta          # signed_speed > 0
    ux, uy = math.cos(travel_theta), math.sin(travel_theta)
    d_safe = s.robot_radius_m + f._extra_safe
    allowed = min(max(0.0, (dist - d_safe) / (-(ux * nx + uy * ny)))
                  for dist, nx, ny in f._wall_distances(o.pose, s)
                  if -(ux * nx + uy * ny) > 1e-9)
    assert allowed < d[0] - 1e-9, allowed
    assert dec.cmd.v <= allowed + 1e-9, (dec.cmd.v, allowed)


cases.append(run_case("approach_wall_cbf", (0.15, 0.0),
                      (1.0, 1.45, math.pi / 2.0), check=chk_approach_wall))


def chk_inside_dsafe(f, s, o, d, dec):
    assert dec.mode == "MODIFIED", dec.mode
    d_safe = s.robot_radius_m + f._extra_safe
    walls = f._wall_distances(o.pose, s)
    assert min(dist for dist, _, _ in walls) <= d_safe   # saturated repulsion


cases.append(run_case("inside_dsafe_saturated", (0.15, 0.0),
                      (0.32, 0.5, 0.0), check=chk_inside_dsafe))


def chk_stop(f, s, o, d, dec):
    assert dec.mode == "STOP", dec.mode
    assert dec.cmd.v == 0.0 and dec.cmd.omega == 0.0


cases.append(run_case("stale_pose_stop", (0.10, 0.0), (1.0, 0.5, 0.0),
                      pose_age=0.6, check=chk_stop))
cases.append(run_case("stale_link_stop", (0.10, 0.0), (1.0, 0.5, 0.0),
                      link_age=0.6, check=chk_stop))
cases.append(run_case("no_pose_stop", (0.10, 0.0), no_pose=True, check=chk_stop))


def chk_reverse(f, s, o, d, dec):
    assert d[0] < 0.0
    assert o.pose.theta == 0.0          # travel_theta mirrored to pi
    assert dec.mode == "MODIFIED", dec.mode
    walls = f._wall_distances(o.pose, s)
    assert walls and min(w[0] for w in walls) < f._influence


cases.append(run_case("reverse_toward_wall", (-0.15, 0.0), (0.35, 0.5, 0.0),
                      check=chk_reverse))


def chk_drift(f, s, o, d, dec):
    assert abs(dec.debug["d_safe_eff_m"] - 0.50) < 1e-12, dec.debug
    assert dec.mode == "MODIFIED", dec.mode
    d_safe = s.robot_radius_m + f._extra_safe + 0.25
    walls = f._wall_distances(o.pose, s)
    assert min(dist for dist, _, _ in walls) <= d_safe   # inflated d_safe bites


cases.append(run_case("drift_inflates_dsafe", (0.15, 0.0),
                      (1.0, 1.45, math.pi / 2.0), drift=0.25, check=chk_drift))


def chk_theta_error(f, s, o, d, dec):
    assert dec.mode == "MODIFIED", dec.mode
    assert dec.cmd.v == 0.0                 # spin in place
    assert abs(dec.cmd.omega - 1.5) < 1e-12  # clamped to max_omega
    d_safe = s.robot_radius_m + f._extra_safe
    err = apf_theta_error(f, s, o.pose, d, d_safe, 1)
    assert err is not None and abs(err) > f._theta_error_max + 1e-12, err


cases.append(run_case("theta_error_spin", (0.05, 0.0),
                      (1.0, 1.55, math.pi / 2.0), check=chk_theta_error))


# Force cancellation: reverse full-speed into the bottom wall while heading
# exactly away from it, so travel_theta points exactly opposite the wall's
# inward normal. Attractive (|v| = max_v) and saturated repulsion
# (strength = max_v inside d_safe) cancel: norm < 1e-9 -> cmd = (0, 0).
# The reference returns MODIFIED here (its `cmd is None -> STOP` branch is
# dead code); the golden pins the real outcome.
bx1, by1 = TRAPEZOID[0]
bx2, by2 = TRAPEZOID[1]
b_ex, b_ey = bx2 - bx1, by2 - by1
b_len = math.hypot(b_ex, b_ey)
b_nx, b_ny = -b_ey / b_len, b_ex / b_len          # CCW inward normal (up)


def chk_cancellation(f, s, o, d, dec):
    assert dec.mode == "MODIFIED", dec.mode
    assert dec.cmd.v == 0.0 and dec.cmd.omega == 0.0
    d_safe = s.robot_radius_m + f._extra_safe
    walls = f._wall_distances(o.pose, s)
    assert min(dist for dist, _, _ in walls) <= d_safe
    assert apf_theta_error(f, s, o.pose, d, d_safe, -1) is None  # norm < 1e-9


cases.append(run_case("force_cancellation", (-0.15, 0.0),
                      (1.0, -0.45, math.atan2(b_ny, b_nx)),
                      check=chk_cancellation))


def chk_zero_v(f, s, o, d, dec):
    assert dec.mode == "PASS", dec.mode
    assert dec.cmd.v == 0.0 and dec.cmd.omega == 0.0


cases.append(run_case("zero_v_pass", (0.0, 0.0), (1.0, 0.5, 0.0),
                      check=chk_zero_v))


# Wall exactly at the influence boundary: place the robot as close as
# representable to distance == 0.45 from the bottom wall (>= 0.45 so the
# strict `distance < influence` test does NOT fire), heading along the wall so
# the CBF closing term is ~0. Expected: PASS.
px = 1.0
py = (0.45 - b_nx * (px - bx1)) / b_ny + by1
d0 = b_nx * (px - bx1) + b_ny * (py - by1)
while d0 < 0.45:
    py = math.nextafter(py, math.inf)
    d0 = b_nx * (px - bx1) + b_ny * (py - by1)
assert 0.45 <= d0 < 0.45 + 1e-12, (d0, py)
theta_wall = math.atan2(b_ey, b_ex)               # heading along the bottom edge


def chk_boundary(f, s, o, d, dec):
    assert dec.mode == "PASS", dec.mode
    assert dec.cmd.v == d[0] and dec.cmd.omega == d[1]
    distances = sorted(dist for dist, _, _ in f._wall_distances(o.pose, s))
    assert distances[0] >= 0.45 and distances[1] > 0.9, distances


cases.append(run_case("influence_boundary", (0.10, 0.0),
                      (px, py, theta_wall), check=chk_boundary))


def chk_two_point(f, s, o, d, dec):
    assert not f._wall_distances(o.pose, s)        # len(fence) < 3 -> no walls
    assert dec.mode == "PASS", dec.mode


cases.append(run_case("two_point_geofence", (0.10, 0.0), (0.5, 0.5, 0.0),
                      static=STATIC_TWO_POINT, check=chk_two_point))


def chk_triangle(f, s, o, d, dec):
    assert dec.mode == "MODIFIED", dec.mode
    d_safe = s.robot_radius_m + f._extra_safe
    walls = f._wall_distances(o.pose, s)
    assert len(walls) == 3
    assert min(dist for dist, _, _ in walls) <= d_safe


cases.append(run_case("three_point_geofence", (0.15, 0.0), (0.9, 0.9, 0.0),
                      static=STATIC_TRIANGLE, check=chk_triangle))

# ---------------------------------------------------------------------------
# One tick (t=0) of each S1-S7 standard scenario, driven exactly like
# safety_sim.runner.run_scenario does at the first control tick.
# ---------------------------------------------------------------------------

from safety_sim.scenarios import get_scenario  # noqa: E402


def scenario_case(name):
    sc = get_scenario(name)
    world = sc.make_world()
    nav = sc.make_nav()
    static = StaticInfo(params=None, robot_radius_m=sc.robot_radius_m,
                        geofence=world.geofence, max_v_mps=sc.max_v_mps,
                        max_omega_rad_s=sc.max_omega_rad_s)
    filt = SafeApfFilter()
    filt.reset(static)
    o = Observation(pose=sc.start_pose, pose_age_s=0.0,
                    wheel_feedback=(0.0, 0.0), obstacles=world.obstacles,
                    link_age_s=0.0, pose_drift_m=0.0)
    desired = nav.command(o, 0.0)
    dec = filt.filter(desired, o, 0.0, 1.0 / sc.control_hz)
    return {
        "name": f"{name}_t0_tick",
        "desired": [desired.v, desired.omega],
        "obs": [sc.start_pose.x, sc.start_pose.y, sc.start_pose.theta,
                0.0, 0.0, 0.0, 0.0, 0.0],
        "t": 0.0,
        "dt": 1.0 / sc.control_hz,
        "expected": {"mode": dec.mode, "v": dec.cmd.v, "omega": dec.cmd.omega},
        "static": static_to_json(static),
    }


for sc_name in ("S1", "S2", "S3", "S4", "S5", "S6", "S7"):
    cases.append(scenario_case(sc_name))

# ---------------------------------------------------------------------------
# Write golden.json
# ---------------------------------------------------------------------------

out = {"static": static_to_json(STATIC), "cases": cases}
out_path = Path(__file__).resolve().parent / "golden.json"
with open(out_path, "w") as fh:
    json.dump(out, fh, indent=2)
    fh.write("\n")

modes = [c["expected"]["mode"] for c in cases]
print(f"wrote {out_path} with {len(cases)} cases")
print("mode distribution:", {m: modes.count(m) for m in sorted(set(modes))})
for c in cases:
    e = c["expected"]
    print(f"  {c['name']:<28} -> {e['mode']:<8} v={e['v']:.9g} omega={e['omega']:.9g}")
