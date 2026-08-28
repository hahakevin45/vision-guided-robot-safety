#include "stm32_motor_driver.h"

static void configure_pwm_gpio(void);
static void configure_direction_gpio(void);
static void configure_tim3(void);
static void set_direction(GPIO_TypeDef *in1_port, uint16_t in1_pin, GPIO_TypeDef *in2_port, uint16_t in2_pin, vgr_motor_direction_t direction);
static uint32_t pulse_from_percent(uint8_t duty_percent);

void stm32_motor_driver_init(void) {
    configure_pwm_gpio();
    configure_direction_gpio();
    configure_tim3();
    stm32_motor_driver_stop();
}

void stm32_motor_driver_apply(vgr_motor_output_t output) {
    if (!output.standby_enabled) {
        stm32_motor_driver_stop();
        return;
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    /* Hardware wiring verified on the bench 2026-07-02 (operator watched each
       channel spin a physical wheel):
         physical LEFT  motor  = TB6612 channel B: PWM TIM3_CH2/PA7, dir PB6/PA6
         physical RIGHT motor  = TB6612 channel A: PWM TIM3_CH1/PB4, dir PB5/PB10
       So motor_output.left_* must drive channel B and right_* channel A. This
       is the single mapping point for BOTH the discrete-intent and the
       velocity paths (both reach stm32_motor_driver_apply). */
    set_direction(GPIOB, GPIO_PIN_6, GPIOA, GPIO_PIN_6, output.left_direction);
    set_direction(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_10, output.right_direction);
    TIM3->CCR2 = pulse_from_percent(output.left_duty_percent);
    TIM3->CCR1 = pulse_from_percent(output.right_duty_percent);
}

void stm32_motor_driver_stop(void) {
    TIM3->CCR1 = 0u;
    TIM3->CCR2 = 0u;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
}

static void configure_pwm_gpio(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;

    gpio.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void configure_direction_gpio(void) {
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static void configure_tim3(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();

    TIM3->CR1 = 0u;
    TIM3->PSC = (uint32_t)((SystemCoreClock / 1000000u) - 1u);
    TIM3->ARR = 999u;
    TIM3->CCR1 = 0u;
    TIM3->CCR2 = 0u;

    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM3->CCMR1 |= (6u << TIM_CCMR1_OC1M_Pos) | (6u << TIM_CCMR1_OC2M_Pos);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void set_direction(GPIO_TypeDef *in1_port, uint16_t in1_pin, GPIO_TypeDef *in2_port, uint16_t in2_pin, vgr_motor_direction_t direction) {
    switch (direction) {
    case VGR_MOTOR_DIR_FORWARD:
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        break;
    case VGR_MOTOR_DIR_REVERSE:
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        break;
    case VGR_MOTOR_DIR_STOP:
    default:
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        break;
    }
}

static uint32_t pulse_from_percent(uint8_t duty_percent) {
    if (duty_percent > 100u) {
        duty_percent = 100u;
    }
    return ((uint32_t)duty_percent * 999u) / 100u;
}
