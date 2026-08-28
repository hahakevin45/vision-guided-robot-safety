"""Pure SAPF field equations from Szczepanski 2023 (IEEE RA-L, DOI 10.1109/LRA.2023.3290819).

Implements the paper's equations only: attractive gradient Eq (3)-(4),
repulsive gradient Eq (5)-(6), distance-dependent potential Eq (12)-(17)
with per-obstacle rotation R(gamma), and analytic gains Eq (23) and (29).
The IEEE paper is the source of truth; the author's MATLAB script is not
copied where it deviates (degree/radian handling, front-field truncation).

Convention (paper): the returned gradient is the potential gradient
∇U(q) = ∇Uatt + Σ R(γi)·∇Urep,i. The motion direction is -∇U, converted by
the filter via θ* = atan2(-∇Uy, -∇Ux).

No ROS/Gazebo/NumPy/serial dependencies; stdlib math only. All angles radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ObstacleSample:
    """One normalized obstacle contribution to the field.

    distance: robot-to-obstacle-boundary distance (m), after pose-drift
        inflation, in (0, Q*] for a contributing obstacle.
    grad_x, grad_y: unit gradient of that distance, pointing from the
        boundary into the robot's free space.
    bearing: signed angle of the obstacle relative to the robot heading,
        wrapped to [-pi, pi].
    """

    distance: float
    grad_x: float
    grad_y: float
    bearing: float


@dataclass(frozen=True)
class SapfFieldResult:
    gradient_x: float
    gradient_y: float
    obstacle_gradient_x: float
    obstacle_gradient_y: float
    max_abs_gamma_rad: float
    min_obstacle_distance_m: float
    n_contributing_obstacles: int


def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi], matching atan2 convention."""
    return math.atan2(math.sin(angle), math.cos(angle))


def attractive_gradient(
    qx: float, qy: float, gx: float, gy: float, *, d_g_star: float, zeta: float
) -> tuple[float, float]:
    """Eq (3)-(4): attractive gradient, piecewise quadratic/conic."""
    d_g = math.hypot(gx - qx, gy - qy)
    if d_g <= d_g_star:
        return zeta * (qx - gx), zeta * (qy - gy)
    scale = d_g_star * zeta / d_g
    return scale * (qx - gx), scale * (qy - gy)


def repulsive_gradient(
    d_oi: float, grad_x: float, grad_y: float, *, Q_star: float, eta: float
) -> tuple[float, float]:
    """Eq (5)-(6): repulsive gradient of one obstacle.

    Raises ValueError for a zero or negative distance: the formula is
    singular there and the caller must fail closed instead of fabricating
    a direction with an epsilon.
    """
    if d_oi <= 0.0:
        raise ValueError(f"singular obstacle distance {d_oi}")
    if d_oi > Q_star:
        return 0.0, 0.0
    scale = eta * (1.0 / Q_star - 1.0 / d_oi) / (d_oi * d_oi)
    return scale * grad_x, scale * grad_y


def relative_distance(d_oi: float, *, d_safe: float, d_vort: float) -> float:
    """Eq (16): normalized piecewise distance 0 -> 1 between d_safe and 2*d_vort-d_safe."""
    if d_oi <= d_safe:
        return 0.0
    upper = 2.0 * d_vort - d_safe
    if d_oi >= upper:
        return 1.0
    return (d_oi - d_safe) / (2.0 * (d_vort - d_safe))


def direction_function(alpha: float, alpha_th: float) -> float:
    """Eq (17): +1 for alpha <= alpha_th, -1 otherwise (rotation sense)."""
    return 1.0 if alpha <= alpha_th else -1.0


def rotation_gamma(d_rel: float, alpha: float, alpha_th: float) -> float:
    """Eq (15): rotation angle gamma in [-pi/2, pi/2]."""
    sign = direction_function(alpha, alpha_th)
    if d_rel <= 0.5:
        return math.pi * sign * d_rel
    return math.pi * sign * (1.0 - d_rel)


def rotate(gx: float, gy: float, gamma: float) -> tuple[float, float]:
    """Eq (14): R(gamma) applied to a 2D vector."""
    c, s = math.cos(gamma), math.sin(gamma)
    return c * gx - s * gy, s * gx + c * gy


def compute_analytic_gains(
    *, d_g_star: float, a_max: float, v_max: float, d_safe: float, Q_star: float
) -> tuple[float, float]:
    """Eq (23) and (29): analytical scaling factors of the potential."""
    zeta = math.sqrt(2.0 * a_max * d_g_star) / d_g_star
    eta = d_safe * d_safe * Q_star * (v_max - d_g_star * zeta) / (d_safe - Q_star)
    return zeta, eta


def command_from_gradient(
    gradient_x: float,
    gradient_y: float,
    *,
    pose_theta: float,
    v_max: float,
    omega_max: float,
    theta_error_max: float,
    k_omega: float,
) -> tuple[float, float]:
    """Eq (10)-(11): convert the gradient to (v*, omega*).

    Motion direction is -gradient: theta* = atan2(-gy, -gx). Linear speed is
    capped by v_max and scaled by the orientation error; beyond
    theta_error_max the vehicle stops turning in place. Shared by the SAPF
    filter and the obstacle-free nominal controller so both produce identical
    commands when no obstacle is inside Q*.
    """
    theta_star = math.atan2(-gradient_y, -gradient_x)
    theta_err = wrap_angle(theta_star - pose_theta)
    abs_err = abs(theta_err)
    if abs_err > theta_error_max:
        v_star = 0.0
    else:
        orientation_scale = (theta_error_max - abs_err) / theta_error_max
        v_star = min(math.hypot(gradient_x, gradient_y), v_max) * orientation_scale
    omega_star = max(-omega_max, min(k_omega * theta_err, omega_max))
    return max(v_star, 0.0), omega_star


def attractive_command(
    pose: tuple[float, float, float],
    goal: tuple[float, float],
    *,
    d_g_star: float,
    zeta: float,
    v_max: float,
    omega_max: float,
    theta_error_max: float,
    k_omega: float,
) -> tuple[float, float]:
    """Obstacle-free goal-attraction command (spec 4.2 shared nominal).

    The attractive gradient alone feeds Eq (10)-(11). This is what the R3
    passthrough arm commands; SAPF-new must match it while every obstacle and
    wall lies beyond Q*.
    """
    ax, ay = attractive_gradient(pose[0], pose[1], goal[0], goal[1],
                                 d_g_star=d_g_star, zeta=zeta)
    return command_from_gradient(ax, ay, pose_theta=pose[2],
                                 v_max=v_max, omega_max=omega_max,
                                 theta_error_max=theta_error_max,
                                 k_omega=k_omega)


def compute_sapf_field(
    qx: float,
    qy: float,
    goal: tuple[float, float],
    obstacles: tuple[ObstacleSample, ...],
    *,
    d_g_star: float,
    zeta: float,
    Q_star: float,
    eta: float,
    d_safe: float,
    d_vort: float,
    alpha_th: float,
) -> SapfFieldResult:
    """Total field: Eq (2) with per-obstacle rotation, Eq (12)-(13).

    Each obstacle is rotated by its own gamma before summing, matching the
    paper's definition; a single rotation of the summed repulsive gradient
    would be a different method.
    """
    ax, ay = attractive_gradient(qx, qy, goal[0], goal[1], d_g_star=d_g_star, zeta=zeta)
    ox = oy = 0.0
    max_gamma = 0.0
    min_dist = math.inf
    n = 0
    for ob in obstacles:
        if ob.distance > Q_star:
            continue
        if not (
            math.isfinite(ob.distance)
            and math.isfinite(ob.grad_x)
            and math.isfinite(ob.grad_y)
            and math.isfinite(ob.bearing)
        ):
            raise ValueError("non-finite obstacle sample")
        d_rel = relative_distance(ob.distance, d_safe=d_safe, d_vort=d_vort)
        gamma = rotation_gamma(d_rel, ob.bearing, alpha_th)
        rx, ry = rotate(
            *repulsive_gradient(ob.distance, ob.grad_x, ob.grad_y, Q_star=Q_star, eta=eta),
            gamma,
        )
        ox += rx
        oy += ry
        max_gamma = max(max_gamma, abs(gamma))
        min_dist = min(min_dist, ob.distance)
        n += 1
    return SapfFieldResult(
        gradient_x=ax + ox,
        gradient_y=ay + oy,
        obstacle_gradient_x=ox,
        obstacle_gradient_y=oy,
        max_abs_gamma_rad=max_gamma,
        min_obstacle_distance_m=min_dist if min_dist != math.inf else math.inf,
        n_contributing_obstacles=n,
    )
