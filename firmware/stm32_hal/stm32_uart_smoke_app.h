#ifndef STM32_UART_SMOKE_APP_H
#define STM32_UART_SMOKE_APP_H

#include "stm32f4xx_hal.h"

void stm32_uart_smoke_app_init(UART_HandleTypeDef *uart);
void stm32_uart_smoke_app_poll(void);

#endif
