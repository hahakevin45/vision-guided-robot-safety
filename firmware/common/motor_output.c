#include "motor_output.h"

static vgr_motor_output_t stopped_output(void) {
    vgr_motor_output_t output = {
        .left_direction = VGR_MOTOR_DIR_STOP,
        .right_direction = VGR_MOTOR_DIR_STOP,
        .left_duty_percent = 0u,
        .right_duty_percent = 0u,
        .standby_enabled = false,
    };
    return output;
}

static vgr_motor_direction_t maybe_invert(vgr_motor_direction_t direction, int inverted) {
    if (!inverted) {
        return direction;
    }
    if (direction == VGR_MOTOR_DIR_FORWARD) {
        return VGR_MOTOR_DIR_REVERSE;
    }
    if (direction == VGR_MOTOR_DIR_REVERSE) {
        return VGR_MOTOR_DIR_FORWARD;
    }
    return direction;
}

static vgr_motor_output_t apply_inversion(vgr_motor_output_t output) {
    output.left_direction = maybe_invert(output.left_direction, VGR_LEFT_MOTOR_INVERTED);
    output.right_direction = maybe_invert(output.right_direction, VGR_RIGHT_MOTOR_INVERTED);
    return output;
}

static vgr_motor_output_t maybe_swap_channels(vgr_motor_output_t output) {
    if (!VGR_SWAP_MOTOR_CHANNELS) {
        return output;
    }

    vgr_motor_direction_t left_direction = output.left_direction;
    uint8_t left_duty_percent = output.left_duty_percent;
    output.left_direction = output.right_direction;
    output.left_duty_percent = output.right_duty_percent;
    output.right_direction = left_direction;
    output.right_duty_percent = left_duty_percent;
    return output;
}

vgr_motor_output_t vgr_motor_output_from_intent(vgr_motor_intent_t intent) {
    vgr_motor_output_t output = stopped_output();

    switch (intent) {
    case VGR_MOTOR_FORWARD:
        output.left_direction = VGR_MOTOR_DIR_FORWARD;
        output.right_direction = VGR_MOTOR_DIR_FORWARD;
        output.left_duty_percent = VGR_MOTOR_BENCH_DUTY_PERCENT;
        output.right_duty_percent = VGR_MOTOR_BENCH_DUTY_PERCENT;
        output.standby_enabled = true;
        break;
    case VGR_MOTOR_TURN_LEFT:
        output.right_direction = VGR_MOTOR_DIR_FORWARD;
        output.right_duty_percent = VGR_MOTOR_BENCH_DUTY_PERCENT;
        output.standby_enabled = true;
        break;
    case VGR_MOTOR_TURN_RIGHT:
        output.left_direction = VGR_MOTOR_DIR_FORWARD;
        output.left_duty_percent = VGR_MOTOR_BENCH_DUTY_PERCENT;
        output.standby_enabled = true;
        break;
    case VGR_MOTOR_STOP:
    default:
        break;
    }

    return apply_inversion(maybe_swap_channels(output));
}
