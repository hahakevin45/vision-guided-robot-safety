#include "stm32_uart_smoke_app.h"

#include "main.h"
#include <string.h>

static UART_HandleTypeDef *smoke_uart = 0;
static uint32_t last_tx_ms = 0u;

void stm32_uart_smoke_app_init(UART_HandleTypeDef *uart) {
    smoke_uart = uart;
}

void stm32_uart_smoke_app_poll(void) {
    static const char message[] = "VGR_READY\r\n";
    uint32_t now_ms = HAL_GetTick();

    if (smoke_uart == 0) {
        return;
    }

    if ((uint32_t)(now_ms - last_tx_ms) < 500u) {
        return;
    }

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    (void)HAL_UART_Transmit(smoke_uart, (uint8_t *)message, (uint16_t)strlen(message), 50u);
    last_tx_ms = now_ms;
}
