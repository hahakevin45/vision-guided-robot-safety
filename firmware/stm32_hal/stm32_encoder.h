#ifndef STM32_ENCODER_H
#define STM32_ENCODER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
    int32_t left_count;
    int32_t right_count;
    uint8_t flags;
} stm32_encoder_snapshot_t;

void stm32_encoder_init(void);
void stm32_encoder_update(void);
stm32_encoder_snapshot_t stm32_encoder_snapshot(void);

#endif
