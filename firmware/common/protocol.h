#ifndef VGR_PROTOCOL_H
#define VGR_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define VGR_HEADER 0xA5u
#define VGR_VERSION 1u
#define VGR_STATE_PACKET_TYPE 0x80u
#define VGR_ENCODER_PACKET_TYPE 0x81u
#define VGR_COMMAND_PACKET_LEN 6u
#define VGR_STATE_PACKET_LEN 10u
#define VGR_ENCODER_PACKET_LEN 14u
#define VGR_SET_WHEEL_SPEED_PAYLOAD_LEN 4u
#define VGR_SET_WHEEL_SPEED_PACKET_LEN 10u

#ifndef VGR_MOTOR_MAX_DUTY_PERCENT
#define VGR_MOTOR_MAX_DUTY_PERCENT 80u
#endif

#ifndef VGR_CMD_TIMEOUT_MS
#define VGR_CMD_TIMEOUT_MS 500u
#endif

#ifndef VGR_MAX_TARGET_COUNTS_PER_S
#define VGR_MAX_TARGET_COUNTS_PER_S 900
#endif

typedef enum {
    VGR_CMD_STOP = 0,
    VGR_CMD_TURN_LEFT = 1,
    VGR_CMD_TURN_RIGHT = 2,
    VGR_CMD_FORWARD = 3,
    VGR_CMD_HEARTBEAT = 4,
    VGR_CMD_READ_ENCODERS = 5,
    VGR_CMD_SET_WHEEL_SPEED = 6,
} vgr_command_id_t;

typedef enum {
    VGR_MCU_IDLE = 0,
    VGR_MCU_ARMED = 1,
    VGR_MCU_TRACKING = 2,
    VGR_MCU_SAFE_STOP = 3,
    VGR_MCU_FAULT = 4,
} vgr_mcu_state_t;

typedef enum {
    VGR_ERR_OK = 0,
    VGR_ERR_BAD_HEADER = 1,
    VGR_ERR_BAD_CHECKSUM = 2,
    VGR_ERR_BAD_SEQUENCE = 3,
    VGR_ERR_INVALID_COMMAND = 4,
    VGR_ERR_COMMAND_TIMEOUT = 5,
    VGR_ERR_FORCED_FAULT = 6,
} vgr_error_t;

typedef enum {
    VGR_MOTOR_STOP = 0,
    VGR_MOTOR_FORWARD = 1,
    VGR_MOTOR_TURN_LEFT = 2,
    VGR_MOTOR_TURN_RIGHT = 3,
} vgr_motor_intent_t;

typedef struct {
    uint8_t sequence;
    vgr_command_id_t command;
} vgr_command_packet_t;

typedef struct {
    uint8_t sequence;
    vgr_mcu_state_t state;
    vgr_error_t error;
    vgr_motor_intent_t motor_intent;
    uint16_t uptime_ms;
} vgr_state_packet_t;

typedef struct {
    uint8_t sequence;
    int32_t left_count;
    int32_t right_count;
    uint8_t flags;
} vgr_encoder_packet_t;

typedef struct {
    int16_t left_counts_per_s;
    int16_t right_counts_per_s;
} vgr_set_wheel_speed_t;

uint8_t vgr_checksum(const uint8_t *data, uint8_t len);
bool vgr_decode_command(const uint8_t raw[VGR_COMMAND_PACKET_LEN], vgr_command_packet_t *packet, vgr_error_t *error);
void vgr_encode_state(const vgr_state_packet_t *packet, uint8_t raw[VGR_STATE_PACKET_LEN]);
void vgr_encode_encoder(const vgr_encoder_packet_t *packet, uint8_t raw[VGR_ENCODER_PACKET_LEN]);
bool vgr_decode_set_wheel_speed(const uint8_t raw[VGR_SET_WHEEL_SPEED_PACKET_LEN], vgr_set_wheel_speed_t *packet, vgr_error_t *error);

#endif
