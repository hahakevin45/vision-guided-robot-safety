#include "stm32_phase2_app.h"

#include "../common/protocol.h"
#include "../common/state_machine.h"
#include "../common/motor_output.h"
#include "../common/velocity_control.h"
#include "stm32_motor_driver.h"
#include "stm32_encoder.h"

static UART_HandleTypeDef *phase2_uart = 0;
static vgr_controller_t controller;
static vgr_velocity_controller_t velocity_left;
static vgr_velocity_controller_t velocity_right;
static uint8_t rx_buffer[VGR_SET_WHEEL_SPEED_PACKET_LEN];
static uint8_t rx_count = 0u;
static uint32_t last_ready_ms = 0u;
static bool command_seen = false;

/* Dual-wheel velocity mode: set by SET_WHEEL_SPEED (left+right targets),
   cleared by any legacy discrete command (STOP/TURN/FORWARD/HEARTBEAT). */
static bool velocity_mode_active = false;
static int16_t velocity_target_left = 0;
static int16_t velocity_target_right = 0;
static uint32_t velocity_last_cmd_ms = 0u;
static int32_t velocity_last_left_count = 0;
static int32_t velocity_last_right_count = 0;
static uint32_t velocity_last_tick_ms = 0u;
static vgr_motor_output_t velocity_last_output = {VGR_MOTOR_DIR_STOP, VGR_MOTOR_DIR_STOP, 0u, 0u, false};

/* Conservative gains: reach target smoothly, no aggressive auto-tune.
   PID runs on a fixed cadence (not every poll tick) so dt and the encoder
   delta it measures against are large enough to avoid quantization noise
   from the raw per-tick encoder count. */
#define VGR_VELOCITY_KP 0.15f
/* KI chosen from raised-wheel step-response comparisons: lower gain converged
   too slowly, while higher gain amplified encoder quantization into a limit
   cycle. Revalidate these gains under any materially different drivetrain
   load. */
#define VGR_VELOCITY_KI 0.12f
#define VGR_VELOCITY_KD 0.0f
#define VGR_VELOCITY_UPDATE_INTERVAL_MS 20u

void stm32_phase2_app_init(UART_HandleTypeDef *uart) {
    phase2_uart = uart;
    vgr_controller_init(&controller, 500u);
    vgr_velocity_init(&velocity_left, VGR_VELOCITY_KP, VGR_VELOCITY_KI, VGR_VELOCITY_KD);
    vgr_velocity_init(&velocity_right, VGR_VELOCITY_KP, VGR_VELOCITY_KI, VGR_VELOCITY_KD);
    stm32_motor_driver_init();
    stm32_encoder_init();
}

static vgr_motor_output_t stm32_phase2_compute_output(uint32_t now_ms) {
    if (!velocity_mode_active) {
        return vgr_motor_output_from_intent(controller.motor_intent);
    }

    uint32_t elapsed_ms = now_ms - velocity_last_tick_ms;
    if (velocity_last_tick_ms != 0u && elapsed_ms < VGR_VELOCITY_UPDATE_INTERVAL_MS) {
        /* Deadman watchdog must still be checked every tick even between
           PID updates, so a stale command still forces STOP promptly. */
        if ((now_ms - velocity_last_cmd_ms) > VGR_CMD_TIMEOUT_MS) {
            velocity_last_output.left_direction = VGR_MOTOR_DIR_STOP;
            velocity_last_output.left_duty_percent = 0u;
            velocity_last_output.right_direction = VGR_MOTOR_DIR_STOP;
            velocity_last_output.right_duty_percent = 0u;
            velocity_last_output.standby_enabled = false;
        }
        return velocity_last_output;
    }

    stm32_encoder_snapshot_t encoder_snapshot = stm32_encoder_snapshot();
    int32_t delta_left = encoder_snapshot.left_count - velocity_last_left_count;
    int32_t delta_right = encoder_snapshot.right_count - velocity_last_right_count;
    velocity_last_left_count = encoder_snapshot.left_count;
    velocity_last_right_count = encoder_snapshot.right_count;

    float dt_s = (float)elapsed_ms / 1000.0f;
    velocity_last_tick_ms = now_ms;
    if (dt_s <= 0.0f) {
        dt_s = (float)VGR_VELOCITY_UPDATE_INTERVAL_MS / 1000.0f;
    }

    uint32_t cmd_age_ms = now_ms - velocity_last_cmd_ms;

    /* Dual-wheel velocity control: independent PID per wheel. left_* drives
       the physical left motor and right_* the physical right (mapping fixed
       in stm32_motor_driver_apply); each reads its own encoder delta with
       forward = positive. */
    vgr_velocity_output_t vout_left =
        vgr_velocity_step(&velocity_left, velocity_target_left, delta_left, dt_s, cmd_age_ms);
    vgr_velocity_output_t vout_right =
        vgr_velocity_step(&velocity_right, velocity_target_right, delta_right, dt_s, cmd_age_ms);

    velocity_last_output.left_direction = vout_left.direction;
    velocity_last_output.left_duty_percent = vout_left.duty_percent;
    velocity_last_output.right_direction = vout_right.direction;
    velocity_last_output.right_duty_percent = vout_right.duty_percent;
    velocity_last_output.standby_enabled =
        (vout_left.duty_percent > 0u) || (vout_right.duty_percent > 0u);
    return velocity_last_output;
}

void stm32_phase2_app_poll(void) {
    uint8_t tx[VGR_STATE_PACKET_LEN];
    uint8_t encoder_tx[VGR_ENCODER_PACKET_LEN];
    uint8_t byte = 0u;
    vgr_command_packet_t command = {0};
    vgr_set_wheel_speed_t speed_command = {0};
    vgr_state_packet_t state = {0};
    vgr_encoder_packet_t encoder = {0};
    stm32_encoder_snapshot_t encoder_snapshot = {0};
    vgr_error_t error = VGR_ERR_OK;
    uint32_t now_ms = HAL_GetTick();
    uint8_t expected_len = VGR_COMMAND_PACKET_LEN;

    if (phase2_uart == 0) {
        return;
    }

    stm32_encoder_update();
    vgr_controller_tick(&controller, now_ms);
    stm32_motor_driver_apply(stm32_phase2_compute_output(now_ms));

    if (!command_seen && (uint32_t)(now_ms - last_ready_ms) >= 1000u) {
        state.sequence = controller.last_sequence;
        state.state = controller.state;
        state.error = controller.last_error;
        state.motor_intent = controller.motor_intent;
        state.uptime_ms = (uint16_t)(HAL_GetTick() & 0xFFFFu);
        vgr_encode_state(&state, tx);
        (void)HAL_UART_Transmit(phase2_uart, tx, VGR_STATE_PACKET_LEN, 50u);
        last_ready_ms = now_ms;
    }

    if (HAL_UART_Receive(phase2_uart, &byte, 1u, 1u) != HAL_OK) {
        return;
    }

    if (rx_count == 0u && byte != VGR_HEADER) {
        return;
    }

    rx_buffer[rx_count] = byte;
    rx_count++;

    if (rx_count < 5u) {
        return;
    }

    expected_len = (uint8_t)(6u + rx_buffer[4]);
    if (expected_len > VGR_SET_WHEEL_SPEED_PACKET_LEN) {
        /* Unsupported payload length: resync on next header byte. */
        rx_count = 0u;
        return;
    }

    if (rx_count < expected_len) {
        return;
    }

    rx_count = 0u;
    command_seen = true;

    if (expected_len == VGR_SET_WHEEL_SPEED_PACKET_LEN && rx_buffer[3] == (uint8_t)VGR_CMD_SET_WHEEL_SPEED) {
        if (vgr_decode_set_wheel_speed(rx_buffer, &speed_command, &error)) {
            uint8_t seq = rx_buffer[2];
            bool seq_ok = true;
            if (controller.has_sequence) {
                uint8_t expected = (uint8_t)(controller.last_sequence + 1u);
                seq_ok = (seq == expected);
            }
            if (!seq_ok) {
                /* Same BAD_SEQUENCE contract legacy commands enforce via
                   vgr_controller_apply: speed packets must not bypass it. */
                velocity_mode_active = false;
                velocity_target_left = 0;
                velocity_target_right = 0;
                velocity_last_tick_ms = 0u;
                controller.last_sequence = seq;
                controller.last_error = VGR_ERR_BAD_SEQUENCE;
                controller.state = VGR_MCU_SAFE_STOP;
                controller.motor_intent = VGR_MOTOR_STOP;
            } else {
                velocity_mode_active = true;
                velocity_target_left = speed_command.left_counts_per_s;
                velocity_target_right = speed_command.right_counts_per_s;
                velocity_last_cmd_ms = now_ms;
                controller.has_sequence = true;
                controller.last_sequence = seq;
                /* Keep controller.last_command_ms fresh so vgr_controller_tick's
                   independent COMMAND_TIMEOUT check does not fire a stale
                   SAFE_STOP telemetry state while velocity commands keep
                   arriving (it would otherwise desync from the real motor
                   output, which velocity mode drives separately). */
                controller.last_command_ms = now_ms;
                controller.last_error = VGR_ERR_OK;
                bool any_target = (velocity_target_left != 0) || (velocity_target_right != 0);
                controller.motor_intent = any_target ? VGR_MOTOR_FORWARD : VGR_MOTOR_STOP;
                controller.state = any_target ? VGR_MCU_TRACKING : VGR_MCU_ARMED;
            }
        } else {
            velocity_mode_active = false;
            velocity_target_left = 0;
            velocity_target_right = 0;
            velocity_last_tick_ms = 0u;
            controller.state = VGR_MCU_SAFE_STOP;
            controller.last_error = error;
            controller.motor_intent = VGR_MOTOR_STOP;
        }
        stm32_motor_driver_apply(stm32_phase2_compute_output(now_ms));

        state.sequence = rx_buffer[2];
        state.state = controller.state;
        state.error = controller.last_error;
        state.motor_intent = controller.motor_intent;
        state.uptime_ms = (uint16_t)(HAL_GetTick() & 0xFFFFu);
        vgr_encode_state(&state, tx);
        (void)HAL_UART_Transmit(phase2_uart, tx, VGR_STATE_PACKET_LEN, 50u);
        return;
    }

    if (expected_len != VGR_COMMAND_PACKET_LEN) {
        /* Unsupported legacy payload length: drop and resync. */
        return;
    }

    /* Any legacy discrete *motion* command exits velocity mode (mutually
       exclusive control modes, no stale velocity target lingers).
       READ_ENCODERS is a passive telemetry query interleaved by the bench
       harness between SET_WHEEL_SPEED updates and must not disturb whichever
       control mode is currently driving the motor. */
    if (rx_buffer[3] != (uint8_t)VGR_CMD_READ_ENCODERS) {
        velocity_mode_active = false;
        velocity_target_left = 0;
        velocity_target_right = 0;
        velocity_last_tick_ms = 0u;
        vgr_velocity_init(&velocity_left, VGR_VELOCITY_KP, VGR_VELOCITY_KI, VGR_VELOCITY_KD);
        vgr_velocity_init(&velocity_right, VGR_VELOCITY_KP, VGR_VELOCITY_KI, VGR_VELOCITY_KD);
    }

    if (vgr_decode_command(rx_buffer, &command, &error)) {
        error = vgr_controller_apply(&controller, &command, now_ms);
    } else {
        controller.state = VGR_MCU_SAFE_STOP;
        controller.last_error = error;
        controller.motor_intent = VGR_MOTOR_STOP;
    }
    stm32_motor_driver_apply(stm32_phase2_compute_output(now_ms));

    if (command.command == VGR_CMD_READ_ENCODERS && error == VGR_ERR_OK) {
        encoder_snapshot = stm32_encoder_snapshot();
        encoder.sequence = command.sequence;
        encoder.left_count = encoder_snapshot.left_count;
        encoder.right_count = encoder_snapshot.right_count;
        encoder.flags = encoder_snapshot.flags;
        vgr_encode_encoder(&encoder, encoder_tx);
        (void)HAL_UART_Transmit(phase2_uart, encoder_tx, VGR_ENCODER_PACKET_LEN, 50u);
        return;
    }

    state.sequence = command.sequence;
    state.state = controller.state;
    state.error = error;
    state.motor_intent = controller.motor_intent;
    state.uptime_ms = (uint16_t)(HAL_GetTick() & 0xFFFFu);
    vgr_encode_state(&state, tx);
    (void)HAL_UART_Transmit(phase2_uart, tx, VGR_STATE_PACKET_LEN, 50u);
}
