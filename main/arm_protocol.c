/**
 * @file arm_protocol.c
 * @brief Implementation of wireless protocol packet building and parsing.
 *
 * This file is SHARED between ESP and PIC projects. Keep them in sync!
 */

#include "arm_protocol.h"
#include <stddef.h>
#include <string.h>

// =============================================================================
// Checksum Implementation (Fletcher-16)
// =============================================================================

uint16_t arm_checksum16(const uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    uint16_t sum1 = 0;
    uint16_t sum2 = 0;

    for (uint8_t i = 0; i < len; ++i) {
        sum1 = (uint16_t)((sum1 + data[i]) % 255u);
        sum2 = (uint16_t)((sum2 + sum1) % 255u);
    }

    return (uint16_t)((sum2 << 8) | sum1);
}

// =============================================================================
// Internal: Header + Checksum Verification
// =============================================================================

static bool arm_verify_packet(const uint8_t *buf,
                              uint8_t len,
                              uint8_t expected_type,
                              uint8_t expected_len)
{
    if (buf == NULL || len < sizeof(arm_header_t) + sizeof(uint16_t)) {
        return false;
    }

    // Check length matches expected
    if (len != expected_len) {
        return false;
    }

    const arm_header_t *hdr = (const arm_header_t *)buf;

    // Verify header fields
    if (hdr->magic != ARM_PROTO_MAGIC ||
        hdr->version != ARM_PROTO_VERSION ||
        hdr->type != expected_type) {
        return false;
    }

    // Verify checksum (last 2 bytes)
    uint8_t len_no_chk = (uint8_t)(len - sizeof(uint16_t));
    uint16_t calc = arm_checksum16(buf, len_no_chk);

    // Checksum is stored little-endian
    uint16_t recv = (uint16_t)buf[len_no_chk] |
                    ((uint16_t)buf[len_no_chk + 1] << 8);

    return (calc == recv);
}

// =============================================================================
// Packet Builders
// =============================================================================

uint8_t arm_build_motion_packet(arm_motion_packet_t *pkt,
                                const float joints_deg[ARM_MAX_JOINTS],
                                uint8_t gripper,
                                uint8_t flags,
                                uint8_t seq)
{
    if (pkt == NULL || joints_deg == NULL) {
        return 0;
    }

    // Header
    pkt->hdr.magic   = ARM_PROTO_MAGIC;
    pkt->hdr.version = ARM_PROTO_VERSION;
    pkt->hdr.type    = (uint8_t)ARM_MSG_MOTION;
    pkt->hdr.seq     = seq;

    // Convert degrees to int16 (degrees * 10)
    for (uint8_t i = 0; i < ARM_MAX_JOINTS; ++i) {
        float v = joints_deg[i] * 10.0f;

        // Clamp to int16 range
        if (v > 32767.0f)  v = 32767.0f;
        if (v < -32768.0f) v = -32768.0f;

        pkt->joint_deg_x10[i] = (int16_t)v;
    }

    // Gripper and flags
    pkt->gripper = gripper;
    pkt->flags   = flags;

    // Calculate checksum over everything except checksum field
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint8_t len_no_chk = (uint8_t)(sizeof(arm_motion_packet_t) - sizeof(uint16_t));
    pkt->checksum = arm_checksum16(bytes, len_no_chk);

    return (uint8_t)sizeof(arm_motion_packet_t);
}

uint8_t arm_build_estop_packet(arm_estop_packet_t *pkt, uint8_t seq)
{
    if (pkt == NULL) {
        return 0;
    }

    // Header
    pkt->hdr.magic   = ARM_PROTO_MAGIC;
    pkt->hdr.version = ARM_PROTO_VERSION;
    pkt->hdr.type    = (uint8_t)ARM_MSG_ESTOP;
    pkt->hdr.seq     = seq;

    // Calculate checksum
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint8_t len_no_chk = (uint8_t)(sizeof(arm_estop_packet_t) - sizeof(uint16_t));
    pkt->checksum = arm_checksum16(bytes, len_no_chk);

    return (uint8_t)sizeof(arm_estop_packet_t);
}

uint8_t arm_build_sensors_packet(arm_sensors_packet_t *pkt,
                                 uint16_t battery_mv,
                                 int16_t temp_x10,
                                 uint16_t pressure_raw,
                                 uint8_t status_flags,
                                 uint8_t op_state,
                                 uint8_t last_rx_seq)
{
    if (pkt == NULL) {
        return 0u;
    }

    // Header
    pkt->hdr.magic   = ARM_PROTO_MAGIC;
    pkt->hdr.version = ARM_PROTO_VERSION;
    pkt->hdr.type    = (uint8_t)ARM_MSG_SENSORS;
    pkt->hdr.seq     = last_rx_seq;  // Use rx seq as our seq for correlation

    // Battery voltage
    pkt->battery_mv = battery_mv;

    // Temperature already in 0.1°C units (°C * 10)
    pkt->temperature_x10 = temp_x10;

    // Pressure (raw units, e.g., ADC counts)
    pkt->pressure_raw = pressure_raw;

    // Status and state
    pkt->status_flags = status_flags;
    pkt->op_state     = op_state;
    pkt->last_rx_seq  = last_rx_seq;
    pkt->reserved     = 0u;

    // Calculate checksum
    const uint8_t *bytes = (const uint8_t *)pkt;
    uint8_t len_no_chk =
        (uint8_t)(sizeof(arm_sensors_packet_t) - sizeof(uint16_t));
    pkt->checksum = arm_checksum16(bytes, len_no_chk);

    return (uint8_t)sizeof(arm_sensors_packet_t);
}

// =============================================================================
// Packet Parsers
// =============================================================================

arm_msg_type_t arm_peek_type(const uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len < sizeof(arm_header_t)) {
        return ARM_MSG_NONE;
    }

    const arm_header_t *hdr = (const arm_header_t *)buf;

    if (hdr->magic != ARM_PROTO_MAGIC ||
        hdr->version != ARM_PROTO_VERSION) {
        return ARM_MSG_NONE;
    }

    return (arm_msg_type_t)hdr->type;
}

bool arm_parse_motion_packet(const uint8_t *buf, uint8_t len,
                             arm_motion_packet_t *out)
{
    if (buf == NULL || out == NULL) {
        return false;
    }

    if (!arm_verify_packet(buf, len, (uint8_t)ARM_MSG_MOTION,
                           (uint8_t)sizeof(arm_motion_packet_t))) {
        return false;
    }

    // Copy validated packet
    memcpy(out, buf, sizeof(arm_motion_packet_t));
    return true;
}

bool arm_parse_estop_packet(const uint8_t *buf, uint8_t len,
                            arm_estop_packet_t *out)
{
    if (buf == NULL || out == NULL) {
        return false;
    }

    if (!arm_verify_packet(buf, len, (uint8_t)ARM_MSG_ESTOP,
                           (uint8_t)sizeof(arm_estop_packet_t))) {
        return false;
    }

    memcpy(out, buf, sizeof(arm_estop_packet_t));
    return true;
}

bool arm_parse_sensors_packet(const uint8_t *buf, uint8_t len,
                              arm_sensors_packet_t *out)
{
    if (buf == NULL || out == NULL) {
        return false;
    }

    if (!arm_verify_packet(buf, len, (uint8_t)ARM_MSG_SENSORS,
                           (uint8_t)sizeof(arm_sensors_packet_t))) {
        return false;
    }

    memcpy(out, buf, sizeof(arm_sensors_packet_t));
    return true;
}


