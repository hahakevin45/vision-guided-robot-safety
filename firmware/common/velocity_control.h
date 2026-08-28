#ifndef VGR_VELOCITY_CONTROL_H
#define VGR_VELOCITY_CONTROL_H

#include <stdint.h>

#include "protocol.h"
#include "motor_output.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} vgr_velocity_controller_t;

typedef struct {
    uint8_t duty_percent;
    vgr_motor_direction_t direction;
} vgr_velocity_output_t;

void vgr_velocity_init(vgr_velocity_controller_t *controller, float kp, float ki, float kd);
vgr_velocity_output_t vgr_velocity_step(
    vgr_velocity_controller_t *controller,
    int16_t target_counts_per_s,
    int32_t delta_counts,
    float dt_s,
    uint32_t cmd_age_ms);

#endif
