#include "state_machine.h"

void vgr_controller_init(vgr_controller_t *controller, uint32_t command_timeout_ms) {
    controller->state = VGR_MCU_IDLE;
    controller->last_error = VGR_ERR_OK;
    controller->motor_intent = VGR_MOTOR_STOP;
    controller->last_sequence = 0u;
    controller->has_sequence = false;
    controller->last_command_ms = 0u;
    controller->command_timeout_ms = command_timeout_ms;
}

vgr_error_t vgr_controller_apply(vgr_controller_t *controller, const vgr_command_packet_t *packet, uint32_t now_ms) {
    if (packet->command == VGR_CMD_HEARTBEAT && packet->sequence == 0u) {
        controller->has_sequence = false;
    }

    if (controller->has_sequence) {
        uint8_t expected = (uint8_t)(controller->last_sequence + 1u);
        if (packet->sequence != expected) {
            controller->state = VGR_MCU_SAFE_STOP;
            controller->last_error = VGR_ERR_BAD_SEQUENCE;
            controller->motor_intent = VGR_MOTOR_STOP;
            controller->last_sequence = packet->sequence;
            return controller->last_error;
        }
    }

    controller->has_sequence = true;
    controller->last_sequence = packet->sequence;
    controller->last_command_ms = now_ms;
    controller->last_error = VGR_ERR_OK;

    switch (packet->command) {
    case VGR_CMD_STOP:
        controller->state = VGR_MCU_SAFE_STOP;
        controller->motor_intent = VGR_MOTOR_STOP;
        break;
    case VGR_CMD_HEARTBEAT:
        if (controller->state == VGR_MCU_IDLE) {
            controller->state = VGR_MCU_ARMED;
        }
        controller->motor_intent = VGR_MOTOR_STOP;
        break;
    case VGR_CMD_TURN_LEFT:
        controller->state = VGR_MCU_TRACKING;
        controller->motor_intent = VGR_MOTOR_TURN_LEFT;
        break;
    case VGR_CMD_TURN_RIGHT:
        controller->state = VGR_MCU_TRACKING;
        controller->motor_intent = VGR_MOTOR_TURN_RIGHT;
        break;
    case VGR_CMD_FORWARD:
        controller->state = VGR_MCU_TRACKING;
        controller->motor_intent = VGR_MOTOR_FORWARD;
        break;
    case VGR_CMD_READ_ENCODERS:
        if (controller->state == VGR_MCU_IDLE) {
            controller->state = VGR_MCU_ARMED;
        }
        break;
    default:
        controller->state = VGR_MCU_SAFE_STOP;
        controller->last_error = VGR_ERR_INVALID_COMMAND;
        controller->motor_intent = VGR_MOTOR_STOP;
        break;
    }
    return controller->last_error;
}

vgr_error_t vgr_controller_tick(vgr_controller_t *controller, uint32_t now_ms) {
    if (!controller->has_sequence || controller->state == VGR_MCU_FAULT) {
        return controller->last_error;
    }
    if ((uint32_t)(now_ms - controller->last_command_ms) > controller->command_timeout_ms) {
        controller->state = VGR_MCU_SAFE_STOP;
        controller->last_error = VGR_ERR_COMMAND_TIMEOUT;
        controller->motor_intent = VGR_MOTOR_STOP;
    }
    return controller->last_error;
}
