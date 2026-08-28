#include "velocity_control.h"

static float clampf(float value, float lo, float hi) {
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

void vgr_velocity_init(vgr_velocity_controller_t *controller, float kp, float ki, float kd) {
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->integral = 0.0f;
    controller->prev_error = 0.0f;
}

vgr_velocity_output_t vgr_velocity_step(
    vgr_velocity_controller_t *controller,
    int16_t target_counts_per_s,
    int32_t delta_counts,
    float dt_s,
    uint32_t cmd_age_ms) {

    vgr_velocity_output_t out;

    if (cmd_age_ms > VGR_CMD_TIMEOUT_MS) {
        controller->integral = 0.0f;
        controller->prev_error = 0.0f;
        out.duty_percent = 0u;
        out.direction = VGR_MOTOR_DIR_STOP;
        return out;
    }

    int32_t clamped_target = target_counts_per_s;
    if (clamped_target > VGR_MAX_TARGET_COUNTS_PER_S) {
        clamped_target = VGR_MAX_TARGET_COUNTS_PER_S;
    } else if (clamped_target < -VGR_MAX_TARGET_COUNTS_PER_S) {
        clamped_target = -VGR_MAX_TARGET_COUNTS_PER_S;
    }

    if (clamped_target == 0) {
        controller->integral = 0.0f;
        controller->prev_error = 0.0f;
        out.duty_percent = 0u;
        out.direction = VGR_MOTOR_DIR_STOP;
        return out;
    }

    float target_f = (float)clamped_target;
    float measured = (float)delta_counts / dt_s;
    float sign = (target_f > 0.0f) ? 1.0f : -1.0f;
    /* Signed error along the commanded direction: reverse or wrong-direction
     * motion must not read as zero error just because magnitudes match. */
    float error = (target_f - measured) * sign;

    float integral_candidate = controller->integral + error * dt_s;
    float derivative = (error - controller->prev_error) / dt_s;

    float unsat = controller->kp * error + controller->ki * integral_candidate + controller->kd * derivative;

    if (unsat > (float)VGR_MOTOR_MAX_DUTY_PERCENT && error > 0.0f) {
        /* saturated high while still under target: freeze integral (anti-windup) */
    } else if (unsat < 0.0f && error < 0.0f) {
        /* saturated low while overshooting: freeze integral (anti-windup) */
    } else {
        controller->integral = integral_candidate;
    }

    float output = controller->kp * error + controller->ki * controller->integral + controller->kd * derivative;
    output = clampf(output, 0.0f, (float)VGR_MOTOR_MAX_DUTY_PERCENT);

    controller->prev_error = error;

    out.duty_percent = (uint8_t)(output + 0.5f);
    out.direction = (target_f > 0.0f) ? VGR_MOTOR_DIR_FORWARD : VGR_MOTOR_DIR_REVERSE;
    return out;
}
