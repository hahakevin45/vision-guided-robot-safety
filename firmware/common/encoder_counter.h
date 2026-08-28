#ifndef VGR_ENCODER_COUNTER_H
#define VGR_ENCODER_COUNTER_H

#include <stdint.h>

#define VGR_ENCODER_FLAG_INVALID_TRANSITION 0x01u

typedef struct {
    int32_t count;
    uint8_t previous_state;
    uint8_t flags;
} vgr_encoder_counter_t;

void vgr_encoder_counter_init(vgr_encoder_counter_t *counter, uint8_t initial_state);
int8_t vgr_encoder_counter_update(vgr_encoder_counter_t *counter, uint8_t current_state);
int8_t vgr_encoder_counter_update_a_edge(vgr_encoder_counter_t *counter, uint8_t current_state);

#endif
