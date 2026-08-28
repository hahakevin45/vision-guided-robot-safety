#ifndef VGR_MOTOR_OUTPUT_H
#define VGR_MOTOR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol.h"

#ifndef VGR_MOTOR_BENCH_DUTY_PERCENT
#define VGR_MOTOR_BENCH_DUTY_PERCENT 25u
#endif

#ifndef VGR_LEFT_MOTOR_INVERTED
#define VGR_LEFT_MOTOR_INVERTED 0
#endif

#ifndef VGR_RIGHT_MOTOR_INVERTED
#define VGR_RIGHT_MOTOR_INVERTED 0
#endif

#ifndef VGR_SWAP_MOTOR_CHANNELS
#define VGR_SWAP_MOTOR_CHANNELS 0
#endif

typedef enum {
    VGR_MOTOR_DIR_STOP = 0,
    VGR_MOTOR_DIR_FORWARD = 1,
    VGR_MOTOR_DIR_REVERSE = 2,
} vgr_motor_direction_t;

typedef struct {
    vgr_motor_direction_t left_direction;
    vgr_motor_direction_t right_direction;
    uint8_t left_duty_percent;
    uint8_t right_duty_percent;
    bool standby_enabled;
} vgr_motor_output_t;

vgr_motor_output_t vgr_motor_output_from_intent(vgr_motor_intent_t intent);

#endif
