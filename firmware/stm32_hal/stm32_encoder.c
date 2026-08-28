#include "stm32_encoder.h"
#include "encoder_counter.h"

#define LEFT_A_PORT GPIOA
#define LEFT_A_PIN GPIO_PIN_0
#define LEFT_B_PORT GPIOA
#define LEFT_B_PIN GPIO_PIN_1
#define RIGHT_A_PORT GPIOA
#define RIGHT_A_PIN GPIO_PIN_4
#define RIGHT_B_PORT GPIOB
#define RIGHT_B_PIN GPIO_PIN_0

static vgr_encoder_counter_t left_counter;
static vgr_encoder_counter_t right_counter;

static uint8_t read_pair(GPIO_TypeDef *a_port, uint16_t a_pin, GPIO_TypeDef *b_port, uint16_t b_pin);
static void update_left_from_a_edge(void);
static void update_right_from_a_edge(void);

void stm32_encoder_init(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = LEFT_B_PIN;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = RIGHT_B_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pin = LEFT_A_PIN | RIGHT_A_PIN;
    HAL_GPIO_Init(GPIOA, &gpio);

    vgr_encoder_counter_init(
        &left_counter,
        read_pair(LEFT_A_PORT, LEFT_A_PIN, LEFT_B_PORT, LEFT_B_PIN)
    );
    vgr_encoder_counter_init(
        &right_counter,
        read_pair(RIGHT_A_PORT, RIGHT_A_PIN, RIGHT_B_PORT, RIGHT_B_PIN)
    );

    HAL_NVIC_SetPriority(EXTI0_IRQn, 5u, 0u);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 5u, 0u);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void stm32_encoder_update(void) {
    /* Counts are updated from EXTI handlers. Keep this hook for app compatibility. */
}

stm32_encoder_snapshot_t stm32_encoder_snapshot(void) {
    stm32_encoder_snapshot_t snapshot;

    __disable_irq();
    snapshot.left_count = left_counter.count;
    snapshot.right_count = right_counter.count;
    snapshot.flags = (uint8_t)(left_counter.flags | right_counter.flags);
    __enable_irq();

    return snapshot;
}

void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(LEFT_A_PIN);
}

void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(RIGHT_A_PIN);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == LEFT_A_PIN) {
        update_left_from_a_edge();
    } else if (GPIO_Pin == RIGHT_A_PIN) {
        update_right_from_a_edge();
    }
}

static uint8_t read_pair(GPIO_TypeDef *a_port, uint16_t a_pin, GPIO_TypeDef *b_port, uint16_t b_pin) {
    uint8_t a = HAL_GPIO_ReadPin(a_port, a_pin) == GPIO_PIN_SET ? 1u : 0u;
    uint8_t b = HAL_GPIO_ReadPin(b_port, b_pin) == GPIO_PIN_SET ? 1u : 0u;
    return (uint8_t)((a << 1) | b);
}

static void update_left_from_a_edge(void) {
    uint8_t state = read_pair(LEFT_A_PORT, LEFT_A_PIN, LEFT_B_PORT, LEFT_B_PIN);
    /* The physical LEFT encoder is wired with reversed A/B polarity relative to
       the right one: forward motion decoded to a negative delta on the bench
       (2026-07-02, +297 right vs -301 left for the same forward direction).
       Negate here so left_count forward = positive, matching right_count and
       giving a single sign convention across the whole stack. */
    int8_t delta = vgr_encoder_counter_update_a_edge(&left_counter, state);
    left_counter.count -= 2 * (int32_t)delta;
}

static void update_right_from_a_edge(void) {
    uint8_t state = read_pair(RIGHT_A_PORT, RIGHT_A_PIN, RIGHT_B_PORT, RIGHT_B_PIN);
    (void)vgr_encoder_counter_update_a_edge(&right_counter, state);
}
