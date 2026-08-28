import math

import cv2
import numpy as np

from vgr_driver.cli.vision_wheel_revolution import (
    MarkerObservation,
    build_cross_calibration,
    compute_pid_pulse_s,
    detect_yellow_marker,
    estimate_revolutions,
    is_encoder_direction_valid,
)


def test_detect_yellow_marker_returns_center_and_angle():
    frame = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.circle(frame, (100, 60), 10, (0, 255, 255), -1)

    observation = detect_yellow_marker(frame, wheel_center=(80.0, 60.0))

    assert observation is not None
    assert observation.area > 200
    assert abs(observation.cx - 100.0) < 1.0
    assert abs(observation.cy - 60.0) < 1.0
    assert abs(observation.angle_rad) < 0.05


def test_detect_yellow_marker_prefers_previous_angle_over_larger_distractor():
    frame = np.zeros((140, 180, 3), dtype=np.uint8)
    cv2.circle(frame, (120, 70), 7, (0, 255, 255), -1)
    cv2.circle(frame, (80, 115), 14, (0, 255, 255), -1)

    observation = detect_yellow_marker(
        frame,
        wheel_center=(90.0, 70.0),
        previous_angle_rad=0.0,
        max_radius_px=70.0,
    )

    assert observation is not None
    assert abs(observation.cx - 120.0) < 1.0
    assert abs(observation.cy - 70.0) < 1.0


def test_detect_yellow_marker_can_reject_small_yellow_noise():
    frame = np.zeros((140, 180, 3), dtype=np.uint8)
    cv2.circle(frame, (120, 70), 12, (0, 255, 255), -1)
    cv2.circle(frame, (80, 115), 3, (0, 255, 255), -1)

    observation = detect_yellow_marker(
        frame,
        wheel_center=(90.0, 70.0),
        previous_angle_rad=math.pi / 2.0,
        min_area=100,
        max_radius_px=70.0,
    )

    assert observation is not None
    assert abs(observation.cx - 120.0) < 1.0
    assert abs(observation.cy - 70.0) < 1.0


def test_estimate_revolutions_unwraps_marker_angles():
    angles_deg = [170, -170, -120, -60, 0, 60, 120, 170, -170]
    observations = [
        MarkerObservation(
            frame_index=i,
            t_s=i * 0.1,
            cx=0.0,
            cy=0.0,
            area=100,
            angle_rad=math.radians(angle),
        )
        for i, angle in enumerate(angles_deg)
    ]

    estimate = estimate_revolutions(observations)

    assert estimate["valid_observations"] == len(observations)
    assert estimate["direction"] == 1
    assert 0.95 <= estimate["absolute_revolutions"] <= 1.10
    assert estimate["rpm"] > 0


def test_estimate_revolutions_handles_reverse_motion():
    angles_deg = [-170, 170, 120, 60, 0, -60, -120, -170, 170]
    observations = [
        MarkerObservation(
            frame_index=i,
            t_s=i * 0.1,
            cx=0.0,
            cy=0.0,
            area=100,
            angle_rad=math.radians(angle),
        )
        for i, angle in enumerate(angles_deg)
    ]

    estimate = estimate_revolutions(observations)

    assert estimate["direction"] == -1
    assert 0.95 <= estimate["absolute_revolutions"] <= 1.10


def test_estimate_revolutions_rejects_unrealistic_angle_jumps():
    observations = [
        MarkerObservation(
            frame_index=i,
            t_s=i * 0.1,
            cx=0.0,
            cy=0.0,
            area=100,
            angle_rad=angle,
        )
        for i, angle in enumerate([0.0, 0.1, 0.2, 2.8, -2.9, 0.3])
    ]

    estimate = estimate_revolutions(observations, max_step_revolutions=0.20)

    assert estimate["tracking_reliable"] is False
    assert estimate["rejected_angle_steps"] > 0


def test_estimate_revolutions_rejects_mixed_step_directions():
    observations = [
        MarkerObservation(
            frame_index=i,
            t_s=i * 0.1,
            cx=0.0,
            cy=0.0,
            area=100,
            angle_rad=math.radians(angle),
        )
        for i, angle in enumerate([0, 20, 40, 20, 45, 60])
    ]

    estimate = estimate_revolutions(observations, max_reverse_step_ratio=0.19)

    assert estimate["tracking_reliable"] is False
    assert estimate["reverse_step_ratio"] > 0.19


def test_signed_encoder_direction_must_match_expected_direction():
    assert is_encoder_direction_valid(raw_delta=100, expected_direction=1) is True
    assert is_encoder_direction_valid(raw_delta=-100, expected_direction=1) is False
    assert is_encoder_direction_valid(raw_delta=-100, expected_direction=-1) is True


def test_cross_calibration_estimates_counts_per_visual_revolution():
    calibration = build_cross_calibration(
        encoder_abs_counts=87,
        encoder_counts_per_rev=802,
        vision_abs_revolutions=0.1396618667438708,
    )

    assert calibration["encoder_revolutions_from_input_calibration"] > 0.10
    assert 622 <= calibration["counts_per_visual_revolution"] <= 624
    assert calibration["vision_to_encoder_revolution_ratio"] > 1.2
    assert calibration["vision_encoder_agree"] is False


def test_cross_calibration_marks_close_encoder_and_vision_as_agreeing():
    calibration = build_cross_calibration(
        encoder_abs_counts=607,
        encoder_counts_per_rev=576,
        vision_abs_revolutions=1.06,
        agreement_tolerance_revolutions=0.10,
    )

    assert calibration["vision_encoder_delta_revolutions"] < 0.01
    assert calibration["vision_encoder_agree"] is True


def test_pid_pulse_shortens_as_target_gets_close():
    far = compute_pid_pulse_s(
        error_counts=1000.0,
        previous_error_counts=1100.0,
        integral_counts=0.0,
        kp=0.0002,
        ki=0.0,
        kd=0.0,
        min_pulse_s=0.01,
        max_pulse_s=0.08,
    )
    near = compute_pid_pulse_s(
        error_counts=30.0,
        previous_error_counts=80.0,
        integral_counts=0.0,
        kp=0.0002,
        ki=0.0,
        kd=0.0,
        min_pulse_s=0.01,
        max_pulse_s=0.08,
    )

    assert far == 0.08
    assert 0.01 <= near < far


def test_pid_pulse_uses_derivative_to_reduce_overshoot():
    pulse = compute_pid_pulse_s(
        error_counts=40.0,
        previous_error_counts=100.0,
        integral_counts=0.0,
        kp=0.0003,
        ki=0.0,
        kd=0.0002,
        min_pulse_s=0.01,
        max_pulse_s=0.08,
    )

    assert pulse == 0.01
