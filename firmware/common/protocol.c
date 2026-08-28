#include "protocol.h"

uint8_t vgr_checksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0u;
    for (uint8_t i = 0u; i < len; ++i) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

bool vgr_decode_command(const uint8_t raw[VGR_COMMAND_PACKET_LEN], vgr_command_packet_t *packet, vgr_error_t *error) {
    if (raw[0] != VGR_HEADER) {
        *error = VGR_ERR_BAD_HEADER;
        return false;
    }
    if (vgr_checksum(raw, VGR_COMMAND_PACKET_LEN - 1u) != raw[VGR_COMMAND_PACKET_LEN - 1u]) {
        *error = VGR_ERR_BAD_CHECKSUM;
        return false;
    }
    if (raw[1] != VGR_VERSION || raw[4] != 0u || raw[3] > VGR_CMD_SET_WHEEL_SPEED) {
        *error = VGR_ERR_INVALID_COMMAND;
        return false;
    }
    packet->sequence = raw[2];
    packet->command = (vgr_command_id_t)raw[3];
    *error = VGR_ERR_OK;
    return true;
}

void vgr_encode_state(const vgr_state_packet_t *packet, uint8_t raw[VGR_STATE_PACKET_LEN]) {
    raw[0] = VGR_HEADER;
    raw[1] = VGR_VERSION;
    raw[2] = packet->sequence;
    raw[3] = VGR_STATE_PACKET_TYPE;
    raw[4] = (uint8_t)packet->state;
    raw[5] = (uint8_t)packet->error;
    raw[6] = (uint8_t)packet->motor_intent;
    raw[7] = (uint8_t)(packet->uptime_ms & 0xFFu);
    raw[8] = (uint8_t)((packet->uptime_ms >> 8) & 0xFFu);
    raw[9] = vgr_checksum(raw, VGR_STATE_PACKET_LEN - 1u);
}

static void encode_i32_le(int32_t value, uint8_t *out) {
    uint32_t uvalue = (uint32_t)value;
    out[0] = (uint8_t)(uvalue & 0xFFu);
    out[1] = (uint8_t)((uvalue >> 8) & 0xFFu);
    out[2] = (uint8_t)((uvalue >> 16) & 0xFFu);
    out[3] = (uint8_t)((uvalue >> 24) & 0xFFu);
}

void vgr_encode_encoder(const vgr_encoder_packet_t *packet, uint8_t raw[VGR_ENCODER_PACKET_LEN]) {
    raw[0] = VGR_HEADER;
    raw[1] = VGR_VERSION;
    raw[2] = packet->sequence;
    raw[3] = VGR_ENCODER_PACKET_TYPE;
    encode_i32_le(packet->left_count, &raw[4]);
    encode_i32_le(packet->right_count, &raw[8]);
    raw[12] = packet->flags;
    raw[13] = vgr_checksum(raw, VGR_ENCODER_PACKET_LEN - 1u);
}

bool vgr_decode_set_wheel_speed(const uint8_t raw[VGR_SET_WHEEL_SPEED_PACKET_LEN], vgr_set_wheel_speed_t *packet, vgr_error_t *error) {
    if (raw[0] != VGR_HEADER) {
        *error = VGR_ERR_BAD_HEADER;
        return false;
    }
    if (vgr_checksum(raw, VGR_SET_WHEEL_SPEED_PACKET_LEN - 1u) != raw[VGR_SET_WHEEL_SPEED_PACKET_LEN - 1u]) {
        *error = VGR_ERR_BAD_CHECKSUM;
        return false;
    }
    if (raw[1] != VGR_VERSION || raw[3] != (uint8_t)VGR_CMD_SET_WHEEL_SPEED || raw[4] != VGR_SET_WHEEL_SPEED_PAYLOAD_LEN) {
        *error = VGR_ERR_INVALID_COMMAND;
        return false;
    }
    uint16_t left_raw = (uint16_t)(raw[5] | ((uint16_t)raw[6] << 8));
    uint16_t right_raw = (uint16_t)(raw[7] | ((uint16_t)raw[8] << 8));
    packet->left_counts_per_s = (int16_t)left_raw;
    packet->right_counts_per_s = (int16_t)right_raw;
    *error = VGR_ERR_OK;
    return true;
}
