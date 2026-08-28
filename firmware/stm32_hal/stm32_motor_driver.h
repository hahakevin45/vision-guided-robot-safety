#ifndef STM32_MOTOR_DRIVER_H
#define STM32_MOTOR_DRIVER_H

#include "stm32f4xx_hal.h"
#include "motor_output.h"

void stm32_motor_driver_init(void);
void stm32_motor_driver_apply(vgr_motor_output_t output);
void stm32_motor_driver_stop(void);

#endif
