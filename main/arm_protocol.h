/**
 * @file arm_protocol.h
 * @brief Wireless protocol definitions for ESP32-S3 <-> PIC18F57Q43 communication.
 *
 * This file is SHARED between ESP and PIC projects. Keep them in sync!
 *
 * Protocol version: 2
 */

#ifndef ARM_PROTOCOL_H
#define ARM_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Protocol Constants
// =============================================================================

#define ARM_PROTO_MAGIC        0xA5u
#define ARM_PROTO_VERSION      2u           // Version 2: added gripper, e-stop, battery

#define ARM_MAX_JOINTS         6u
#define ARM_MAX_PACKET_SIZE    32u

// =============================================================================
/* Timing Constants (CRITICAL - both sides MUST agree) */
// =============================================================================

#define ARM_LINK_RATE_HZ            50      // Packets per second from ESP
#define ARM_LINK_PERIOD_MS          20      // Milliseconds between packets (1000/50)
#define ARM_LINK_TIMEOUT_MS         100     // PIC enters ESTOP if no packet for 100ms

// Interpolation timing (PIC side servo control)
#define ARM_INTERP_RATE_HZ          200     // Servo update rate
#define ARM_INTERP_PERIOD_MS        5       // Milliseconds between interpolation steps
#define ARM_INTERP_STEPS            4       // Steps per packet (20ms / 5ms = 4)

// =============================================================================
// Message Types
// =============================================================================

typedef enum {
    ARM_MSG_NONE        = 0x00,     // Invalid / not set

    // Commands: ESP -> PIC
    ARM_MSG_MOTION      = 0x01,     // Motion command with joint angles + gripper
    ARM_MSG_ESTOP       = 0x02,     // Emergency stop (immediate, minimal packet)

    // Responses: PIC -> ESP (in ACK payload)
    ARM_MSG_SENSORS     = 0x80,     // Sensor telemetry + status
} arm_msg_type_t;

// Backward compatibility alias
#define ARM_MSG_ANGLES  ARM_MSG_MOTION

// =============================================================================
// Status Flags (reported in sensor response)
// =============================================================================

#define ARM_STATUS_OK               0x00

// Bit 0-1: Warnings (system can continue operating)
#define ARM_STATUS_WARN_TEMP        (1u << 0)   // Temperature above normal
#define ARM_STATUS_WARN_BATTERY     (1u << 1)   // Battery low (<20%)

// Bit 2-4: Faults (system should stop or has stopped)
#define ARM_STATUS_FAULT_SERVO      (1u << 2)   // Servo communication error
#define ARM_STATUS_FAULT_TIMEOUT    (1u << 3)   // Link timeout triggered ESTOP
#define ARM_STATUS_FAULT_OVERTEMP   (1u << 4)   // Over-temperature shutdown

// Bit 5: Reserved

// Bit 6-7: State indicators
#define ARM_STATUS_ESTOP_ACTIVE     (1u << 6)   // E-stop is currently engaged
#define ARM_STATUS_MOTORS_ENABLED   (1u << 7)   // Servos are powered and active

// =============================================================================
// Operational States (reported in sensor response)
// =============================================================================

typedef enum {
    ARM_OP_IDLE     = 0,    // Powered on, servos disabled, waiting for enable
    ARM_OP_ACTIVE   = 1,    // Servos enabled, following motion commands
    ARM_OP_ESTOP    = 2,    // Emergency stopped (link timeout or e-stop cmd)
} arm_op_state_t;

// =============================================================================
// Motion Flags (in motion command)
// =============================================================================

#define ARM_MOTION_FLAG_ENABLE      (1u << 0)   // Enable servos (transition IDLE -> ACTIVE)
#define ARM_MOTION_FLAG_RESET       (1u << 1)   // Reset from ESTOP (with ENABLE: -> ACTIVE)
#define ARM_MOTION_FLAG_DISABLE     (1u << 2)   // Disable servos (-> IDLE)
// Bits 3-7: Reserved

// =============================================================================
// Packet Structures
// =============================================================================

#pragma pack(push, 1)

/**
 * Common header for all packets (4 bytes)
 */
typedef struct {
    uint8_t magic;          // Must be ARM_PROTO_MAGIC (0xA5)
    uint8_t version;        // Must be ARM_PROTO_VERSION
    uint8_t type;           // arm_msg_type_t
    uint8_t seq;            // Sequence number (0-255, wraps)
} arm_header_t;

// -----------------------------------------------------------------------------
// Motion Command: ESP -> PIC (20 bytes)
// -----------------------------------------------------------------------------

/**
 * Motion command packet.
 * Sent at 50 Hz from ESP to PIC with joint angles and gripper position.
 */
typedef struct {
    arm_header_t hdr;                           // 4 bytes
    int16_t joint_deg_x10[ARM_MAX_JOINTS];      // 12 bytes: target angles (degrees * 10)
    uint8_t gripper;                            // 1 byte: 0=open, 255=closed
    uint8_t flags;                              // 1 byte: ARM_MOTION_FLAG_*
    uint16_t checksum;                          // 2 bytes: Fletcher-16
} arm_motion_packet_t;                          // Total: 20 bytes

// Backward compatibility
typedef arm_motion_packet_t arm_angles_packet_t;

// -----------------------------------------------------------------------------
// E-Stop Command: ESP -> PIC (6 bytes)
// -----------------------------------------------------------------------------

/**
 * Emergency stop packet.
 * Minimal size for fastest possible transmission.
 * PIC must immediately disable servos upon receipt.
 */
typedef struct {
    arm_header_t hdr;                           // 4 bytes
    uint16_t checksum;                          // 2 bytes
} arm_estop_packet_t;                           // Total: 6 bytes

// -----------------------------------------------------------------------------
// Sensor Response: PIC -> ESP (16 bytes, in ACK payload)
// -----------------------------------------------------------------------------

/**
 * Sensor telemetry packet.
 * Sent as ACK payload from PIC to ESP.
 *
 * temperature_x10 is temperature in 0.1°C units (°C * 10).
 * pressure_raw is the raw sensor reading (e.g., ADC counts); no float math.
 */
typedef struct {
    arm_header_t hdr;                           // 4 bytes
    uint16_t battery_mv;                        // 2 bytes: battery voltage in millivolts
    int16_t temperature_x10;                    // 2 bytes: temperature (°C * 10)
    uint16_t pressure_raw;                      // 2 bytes: raw pressure reading
    uint8_t status_flags;                       // 1 byte: ARM_STATUS_* flags
    uint8_t op_state;                           // 1 byte: arm_op_state_t
    uint8_t last_rx_seq;                        // 1 byte: echo of last received seq
    uint8_t reserved;                           // 1 byte: for future use
    uint16_t checksum;                          // 2 bytes: Fletcher-16
} arm_sensors_packet_t;                         // Total: 16 bytes

#pragma pack(pop)

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * Calculate Fletcher-16 checksum.
 * @param data  Pointer to data buffer
 * @param len   Length of data
 * @return      16-bit checksum (sum2 << 8 | sum1)
 */
uint16_t arm_checksum16(const uint8_t *data, uint8_t len);

// =============================================================================
// Packet Builders
// =============================================================================

/**
 * Build motion command packet.
 */
uint8_t arm_build_motion_packet(arm_motion_packet_t *pkt,
                                const float joints_deg[ARM_MAX_JOINTS],
                                uint8_t gripper,
                                uint8_t flags,
                                uint8_t seq);

/**
 * Build e-stop command packet.
 */
uint8_t arm_build_estop_packet(arm_estop_packet_t *pkt, uint8_t seq);

/**
 * Build sensor response packet.
 *
 * @param temp_x10 temperature in 0.1°C units (°C * 10)
 */
uint8_t arm_build_sensors_packet(arm_sensors_packet_t *pkt,
                                 uint16_t battery_mv,
                                 int16_t temp_x10,
                                 uint16_t pressure_raw,
                                 uint8_t status_flags,
                                 uint8_t op_state,
                                 uint8_t last_rx_seq);

// =============================================================================
// Packet Parsers
// =============================================================================

/**
 * Peek at packet type without full validation.
 */
arm_msg_type_t arm_peek_type(const uint8_t *buf, uint8_t len);

/**
 * Parse motion packet with full validation.
 */
bool arm_parse_motion_packet(const uint8_t *buf, uint8_t len,
                             arm_motion_packet_t *out);

/**
 * Parse e-stop packet with full validation.
 */
bool arm_parse_estop_packet(const uint8_t *buf, uint8_t len,
                            arm_estop_packet_t *out);

/**
 * Parse sensor packet with full validation.
 */
bool arm_parse_sensors_packet(const uint8_t *buf, uint8_t len,
                              arm_sensors_packet_t *out);

// Backward compatibility
#define arm_parse_angles_packet arm_parse_motion_packet

#ifdef __cplusplus
}
#endif

#endif // ARM_PROTOCOL_H
