"""Pin the Szczepanski 2023 SAPF equations (IEEE RA-L, DOI 10.1109/LRA.2023.3290819).

Eq (3)-(4) attractive, (5)-(6) repulsive, (15)-(17) distance-dependent
potential with per-obstacle rotation R(gamma), (23) and (29) analytic gains.
All angles are radians; the paper is the source of truth, not the author's
MATLAB script (which has degree/radian inconsistencies).
"""
import math

import pytest

from safety_sim.sapf_field import (
    ObstacleSample,
    SapfFieldResult,
    attractive_gradient,
    compute_analytic_gains,
    compute_sapf_field,
    direction_function,
    relative_distance,
    repulsive_gradient,
    rotate,
    rotation_gamma,
    wrap_angle,
    attractive_command,
    command_from_gradient,
)

D_SAFE = 0.28
D_VORT = 0.40
Q_STAR = 0.80
D_G_STAR = 0.30
A_MAX = 0.5
V_MAX = 0.15
ZETA = 1.825741858351
ETA = 0.047971459244
ALPHA_TH = math.radians(5.0)


def test_attractive_gradient_quadratic_near_goal():
    # Eq (4) first branch: zeta * (q - q*)
    gx, gy = attractive_gradient(1.15, 0.0, 1.0, 0.0, d_g_star=D_G_STAR, zeta=ZETA)
    assert gx == pytest.approx(ZETA * 0.15)
    assert gy == pytest.approx(0.0)


def test_attractive_gradient_conic_far_goal():
    # Eq (4) second branch: magnitude d_g* * zeta, direction (q - q*)
    gx, gy = attractive_gradient(0.5, 0.0, 3.2, 0.0, d_g_star=D_G_STAR, zeta=ZETA)
    assert math.hypot(gx, gy) == pytest.approx(D_G_STAR * ZETA)
    assert gx < 0.0  # gradient points from goal toward the robot


def test_attractive_gradient_continuous_at_boundary():
    near = attractive_gradient(1.30, 0.0, 1.0, 0.0, d_g_star=D_G_STAR, zeta=ZETA)
    far = attractive_gradient(1.30 - 1e-9, 0.0, 1.0, 0.0, d_g_star=D_G_STAR, zeta=ZETA)
    assert near[0] == pytest.approx(far[0], rel=1e-6)


def test_repulsive_gradient_zero_beyond_Q_star():
    gx, gy = repulsive_gradient(0.90, 1.0, 0.0, Q_star=Q_STAR, eta=ETA)
    assert (gx, gy) == (0.0, 0.0)


def test_repulsive_gradient_direction_inside_Q_star():
    # d <= Q*: (1/Q* - 1/d) < 0, so the gradient points against grad(d)
    # (toward the obstacle), which is what the paper's Eq (6) defines.
    gx, gy = repulsive_gradient(0.40, 1.0, 0.0, Q_star=Q_STAR, eta=ETA)
    assert gx < 0.0
    assert gy == pytest.approx(0.0)


def test_repulsive_gradient_singular_at_zero_rejected():
    with pytest.raises(ValueError):
        repulsive_gradient(0.0, 1.0, 0.0, Q_star=Q_STAR, eta=ETA)
    with pytest.raises(ValueError):
        repulsive_gradient(-0.1, 1.0, 0.0, Q_star=Q_STAR, eta=ETA)


def test_relative_distance_piecewise():
    # Eq (16): d_safe -> 0, d_vort -> 0.5, 2*d_vort-d_safe -> 1, beyond -> 1
    assert relative_distance(D_SAFE, d_safe=D_SAFE, d_vort=D_VORT) == 0.0
    assert relative_distance(D_VORT, d_safe=D_SAFE, d_vort=D_VORT) == pytest.approx(0.5)
    upper = 2.0 * D_VORT - D_SAFE
    assert relative_distance(upper, d_safe=D_SAFE, d_vort=D_VORT) == 1.0
    assert relative_distance(upper + 0.1, d_safe=D_SAFE, d_vort=D_VORT) == 1.0


def test_direction_function_threshold():
    # Eq (17): alpha <= alpha_th -> +1, otherwise -1
    assert direction_function(ALPHA_TH, ALPHA_TH) == 1.0
    assert direction_function(-ALPHA_TH, ALPHA_TH) == 1.0
    assert direction_function(ALPHA_TH + 1e-9, ALPHA_TH) == -1.0
    # Eq (17) is alpha <= alpha_th -> +1; any value at or below the
    # threshold, including -pi, keeps the +1 rotation sense.
    assert direction_function(-math.pi, ALPHA_TH) == 1.0


def test_gamma_boundaries():
    # Eq (15): 0 at d_safe, +/-pi/2 at d_vort, 0 at 2*d_vort-d_safe
    assert rotation_gamma(0.0, 0.0, ALPHA_TH) == pytest.approx(0.0)
    assert rotation_gamma(0.5, 0.0, ALPHA_TH) == pytest.approx(math.pi / 2.0)
    assert rotation_gamma(1.0, 0.0, ALPHA_TH) == pytest.approx(0.0)


def test_gamma_sign_follows_direction_function():
    assert rotation_gamma(0.5, 0.0, ALPHA_TH) == pytest.approx(math.pi / 2.0)
    assert rotation_gamma(0.5, math.pi, ALPHA_TH) == pytest.approx(-math.pi / 2.0)


def test_rotation_ccw_for_positive_gamma():
    rx, ry = rotate(1.0, 0.0, math.pi / 2.0)
    assert rx == pytest.approx(0.0)
    assert ry == pytest.approx(1.0)


def test_analytic_gains_match_vgr_parameters():
    zeta, eta = compute_analytic_gains(d_g_star=D_G_STAR, a_max=A_MAX, v_max=V_MAX,
                                       d_safe=D_SAFE, Q_star=Q_STAR)
    assert zeta == pytest.approx(ZETA)
    assert eta == pytest.approx(ETA)


def test_field_rotates_single_obstacle_gradient():
    # robot west of an obstacle, dead ahead (bearing 0), d = d_vort:
    # gamma = +pi/2, so the inward repulsive gradient (-x) becomes lateral (-y).
    qx, qy = 2.4, 0.0
    grad = repulsive_gradient(D_VORT, 1.0, 0.0, Q_star=Q_STAR, eta=ETA)
    result = compute_sapf_field(
        qx, qy, (3.2, 0.0),
        (ObstacleSample(D_VORT, 1.0, 0.0, 0.0),),
        d_g_star=D_G_STAR, zeta=ZETA, Q_star=Q_STAR, eta=ETA,
        d_safe=D_SAFE, d_vort=D_VORT, alpha_th=ALPHA_TH,
    )
    assert result.obstacle_gradient_x == pytest.approx(0.0, abs=1e-12)
    assert result.obstacle_gradient_y == pytest.approx(grad[0])  # rotate((g,0), +pi/2) = (0, g)
    assert result.max_abs_gamma_rad == pytest.approx(math.pi / 2.0)
    assert result.n_contributing_obstacles == 1


def test_field_flips_rotation_for_obstacle_behind_robot():
    # bearing pi > alpha_th -> D = -1 -> gamma = -pi/2 -> lateral force flips sign
    qx, qy = 2.4, 0.0
    result = compute_sapf_field(
        qx, qy, (3.2, 0.0),
        (ObstacleSample(D_VORT, 1.0, 0.0, math.pi),),
        d_g_star=D_G_STAR, zeta=ZETA, Q_star=Q_STAR, eta=ETA,
        d_safe=D_SAFE, d_vort=D_VORT, alpha_th=ALPHA_TH,
    )
    # bearing pi -> D = -1 -> gamma = -pi/2: R(-pi/2).(g, 0) = (0, -g) with
    # g < 0, so the lateral component is positive, opposite to the ahead case.
    rep = repulsive_gradient(D_VORT, 1.0, 0.0, Q_star=Q_STAR, eta=ETA)
    rx, ry = rotate(rep[0], rep[1], rotation_gamma(0.5, math.pi, ALPHA_TH))
    assert result.obstacle_gradient_x == pytest.approx(rx, abs=1e-12)
    assert result.obstacle_gradient_y == pytest.approx(ry)
    assert result.obstacle_gradient_y > 0.0


def test_field_sum_matches_manual_attractive_plus_rotated_repulsive():
    # combined check: total = attractive + per-obstacle rotated repulsive
    qx, qy = 2.4, 0.0
    goal = (3.2, 0.0)
    ax, ay = attractive_gradient(qx, qy, goal[0], goal[1], d_g_star=D_G_STAR, zeta=ZETA)
    rep = repulsive_gradient(D_VORT, 1.0, 0.0, Q_star=Q_STAR, eta=ETA)
    rx, ry = rotate(rep[0], rep[1], rotation_gamma(0.5, 0.0, ALPHA_TH))
    result = compute_sapf_field(
        qx, qy, goal,
        (ObstacleSample(D_VORT, 1.0, 0.0, 0.0),),
        d_g_star=D_G_STAR, zeta=ZETA, Q_star=Q_STAR, eta=ETA,
        d_safe=D_SAFE, d_vort=D_VORT, alpha_th=ALPHA_TH,
    )
    assert result.gradient_x == pytest.approx(ax + rx)
    assert result.gradient_y == pytest.approx(ay + ry)
    assert isinstance(result, SapfFieldResult)


def test_field_ignores_obstacles_beyond_Q_star():
    result = compute_sapf_field(
        1.0, 0.0, (3.2, 0.0),
        (ObstacleSample(1.20, 1.0, 0.0, 0.0),),
        d_g_star=D_G_STAR, zeta=ZETA, Q_star=Q_STAR, eta=ETA,
        d_safe=D_SAFE, d_vort=D_VORT, alpha_th=ALPHA_TH,
    )
    assert result.n_contributing_obstacles == 0
    assert result.min_obstacle_distance_m == math.inf
    assert result.max_abs_gamma_rad == 0.0


def test_field_rejects_non_finite_obstacle_sample():
    with pytest.raises(ValueError):
        compute_sapf_field(
            1.0, 0.0, (3.2, 0.0),
            (ObstacleSample(math.nan, 1.0, 0.0, 0.0),),
            d_g_star=D_G_STAR, zeta=ZETA, Q_star=Q_STAR, eta=ETA,
            d_safe=D_SAFE, d_vort=D_VORT, alpha_th=ALPHA_TH,
        )


def test_field_output_always_finite():
    result = compute_sapf_field(
        1.0, 0.0, (3.2, 0.0), (),
        d_g_star=D_G_STAR, zeta=ZETA, Q_star=Q_STAR, eta=ETA,
        d_safe=D_SAFE, d_vort=D_VORT, alpha_th=ALPHA_TH,
    )
    assert math.isfinite(result.gradient_x)
    assert math.isfinite(result.gradient_y)


def test_wrap_angle_ranges():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi)  # (-pi, pi] convention


# --- command conversion: Eq (10)-(11), shared with the nominal controller ---

def test_command_from_gradient_heading_aligned():
    # gradient 指向 +x（車頭 +x）：θ* = atan2(0, -gx)… 當 gradient=(1,0)，
    # -∇U = (-1,0) → θ* = π。這裡用 gradient=(-1,0)（運動方向 +x）。
    v, w = command_from_gradient(
        -1.0, 0.0, pose_theta=0.0,
        v_max=V_MAX, omega_max=1.5,
        theta_error_max=math.pi / 4.0, k_omega=1.5,
    )
    assert v == pytest.approx(V_MAX)      # 幅值被 v_max 截斷
    assert w == pytest.approx(0.0)


def test_command_from_gradient_heading_error_zeroes_linear():
    # θ_err > θ_err_max：v=0，ω 依 k_ω·θ_err 限幅。
    v, w = command_from_gradient(
        -1.0, 0.0, pose_theta=math.pi,   # 車頭與運動方向相反
        v_max=V_MAX, omega_max=1.5,
        theta_error_max=math.pi / 4.0, k_omega=1.5,
    )
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(-1.5)       # θ_err=-π → k_ω·θ_err 截斷至 -1.5


def test_command_from_gradient_linear_scales_with_heading_error():
    # θ_err = θ_max/2：orientation_scale = 0.5 → v = 0.5·min(|g|, v_max)
    v, _ = command_from_gradient(
        -1.0, 0.0, pose_theta=math.pi / 8.0,
        v_max=V_MAX, omega_max=1.5,
        theta_error_max=math.pi / 4.0, k_omega=1.5,
    )
    assert v == pytest.approx(0.5 * V_MAX)


def test_command_from_gradient_omega_clamped_symmetric():
    v, w = command_from_gradient(
        0.0, -1.0, pose_theta=0.0,        # 運動方向 +y（左轉需求為正）
        v_max=V_MAX, omega_max=0.25,
        theta_error_max=math.pi / 4.0, k_omega=1.5,
    )
    assert abs(w) <= 0.25 + 1e-9


def test_attractive_command_matches_obstacle_free_sapf():
    """無障礙時 SAPF 命令 = attractive command（spec 4.2 前提測試）。"""
    goal = (3.2, 0.0)
    pose = (1.0, 0.0, 0.0)
    v_nom, w_nom = attractive_command(
        pose, goal, d_g_star=D_G_STAR, zeta=ZETA,
        v_max=V_MAX, omega_max=0.25,
        theta_error_max=math.pi / 4.0, k_omega=1.5,
    )
    field = compute_sapf_field(
        pose[0], pose[1], goal, (),
        d_g_star=D_G_STAR, zeta=ZETA, Q_star=Q_STAR, eta=ETA,
        d_safe=D_SAFE, d_vort=D_VORT, alpha_th=ALPHA_TH,
    )
    v_sapf, w_sapf = command_from_gradient(
        field.gradient_x, field.gradient_y, pose_theta=pose[2],
        v_max=V_MAX, omega_max=0.25,
        theta_error_max=math.pi / 4.0, k_omega=1.5,
    )
    assert v_nom == pytest.approx(v_sapf, abs=1e-12)
    assert w_nom == pytest.approx(w_sapf, abs=1e-12)
