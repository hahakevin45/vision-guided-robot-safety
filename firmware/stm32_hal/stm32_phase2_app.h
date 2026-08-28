#ifndef STM32_PHASE2_APP_H
#define STM32_PHASE2_APP_H

#include "stm32f4xx_hal.h"

void stm32_phase2_app_init(UART_HandleTypeDef *uart);
void stm32_phase2_app_poll(void);

#endif
