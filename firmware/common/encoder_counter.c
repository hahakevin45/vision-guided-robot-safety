#include "encoder_counter.h"

void vgr_encoder_counter_init(vgr_encoder_counter_t *counter, uint8_t initial_state) {
    counter->count = 0;
    counter->previous_state = initial_state & 0x03u;
    counter->flags = 0u;
}

int8_t vgr_encoder_counter_update(vgr_encoder_counter_t *counter, uint8_t current_state) {
    static const int8_t table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0,
    };
    uint8_t current = current_state & 0x03u;
    uint8_t transition = (uint8_t)(((counter->previous_state & 0x03u) << 2) | current);
    int8_t delta = table[transition];

    if (delta == 0 && current != counter->previous_state) {
        counter->flags |= VGR_ENCODER_FLAG_INVALID_TRANSITION;
    }
    counter->count += delta;
    counter->previous_state = current;
    return delta;
}

int8_t vgr_encoder_counter_update_a_edge(vgr_encoder_counter_t *counter, uint8_t current_state) {
    uint8_t current = current_state & 0x03u;
    uint8_t a = (current >> 1) & 0x01u;
    uint8_t b = current & 0x01u;
    int8_t delta = (a != b) ? 1 : -1;

    counter->count += delta;
    counter->previous_state = current;
    return delta;
}
