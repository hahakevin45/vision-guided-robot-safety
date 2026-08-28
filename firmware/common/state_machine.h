#ifndef VGR_STATE_MACHINE_H
#define VGR_STATE_MACHINE_H

#include "protocol.h"

typedef struct {
    vgr_mcu_state_t state;
    vgr_error_t last_error;
    vgr_motor_intent_t motor_intent;
    uint8_t last_sequence;
    bool has_sequence;
    uint32_t last_command_ms;
    uint32_t command_timeout_ms;
} vgr_controller_t;

void vgr_controller_init(vgr_controller_t *controller, uint32_t command_timeout_ms);
vgr_error_t vgr_controller_apply(vgr_controller_t *controller, const vgr_command_packet_t *packet, uint32_t now_ms);
vgr_error_t vgr_controller_tick(vgr_controller_t *controller, uint32_t now_ms);

#endif
