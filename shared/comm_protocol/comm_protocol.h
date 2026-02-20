/**
 * @file comm_protocol.h
 * @brief Inter-Node Message Format Definitions
 * 
 * Defines the wire format for UART messages between CAN Interface
 * and Display nodes. Both nodes include this to ensure identical
 * serialization/deserialization.
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Frame Constants
// ============================================================================

#define COMM_START_BYTE     0xAA
#define COMM_HEADER_SIZE    8
#define COMM_CRC_SIZE       2
#define COMM_MAX_PAYLOAD    1024

// ============================================================================
// Message Types
// ============================================================================

typedef enum {
    MSG_PID_DATA_SINGLE = 0x01,
    MSG_PID_DATA_BATCH  = 0x02,
    MSG_VEHICLE_INFO    = 0x03,
    MSG_DTC_LIST        = 0x04,
    MSG_PID_METADATA    = 0x05,
    MSG_HEARTBEAT       = 0x10,
    MSG_CONFIG_CMD      = 0x20,
    MSG_CONFIG_RESP     = 0x21,
    MSG_LOG_CONTROL     = 0x30,
    MSG_SCAN_STATUS     = 0x40,
    MSG_ERROR_ALERT     = 0x41,
    MSG_TIME_SYNC       = 0x42,
} comm_msg_type_t;

// ============================================================================
// Message Header (8 bytes, packed)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;      // comm_msg_type_t
    uint8_t  sequence;      // Rolling sequence (0-255)
    uint32_t timestamp;     // Sender uptime in ms
    uint16_t payload_len;   // Payload byte count
} comm_header_t;

// ============================================================================
// Heartbeat Payload (8 bytes)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t  node_state;    // 0=idle, 1=scanning, 2=streaming
    uint8_t  can_status;    // 0=bus off, 1=bus on, 2=error
    uint16_t free_heap_kb;  // Free heap in KB
    uint32_t uptime_ms;     // Sender uptime
} comm_heartbeat_t;

// ============================================================================
// PID Value (11 bytes, packed)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint16_t pid_id;        // PID number
    float    value;         // Decoded value
    uint8_t  unit;          // Unit enum
    uint32_t timestamp;     // When decoded (ms)
} comm_pid_value_t;

// ============================================================================
// Config Command Subtypes
// ============================================================================

typedef enum {
    CMD_SCAN_VEHICLE    = 0x01,     // Request ECU scan (VIN, supported PIDs)
    CMD_SET_POLL_LIST   = 0x02,     // Set PIDs to poll continuously
    CMD_CLEAR_POLL_LIST = 0x03,     // Stop polling
    CMD_SET_POLL_RATE   = 0x04,     // Set polling rate (Hz)
    CMD_CLEAR_DTCS      = 0x10,     // Clear DTCs on vehicle
    CMD_READ_DTCS       = 0x11,     // Request DTC list
} config_cmd_type_t;

// ============================================================================
// Config Command Payload (variable length)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t  cmd_type;      // config_cmd_type_t
    uint8_t  reserved;      // Padding for alignment
    uint16_t data_len;      // Length of cmd_data[]
    uint8_t  cmd_data[];    // Command-specific data (flexible array)
} comm_config_cmd_t;

// ============================================================================
// Scan Status / Response
// ============================================================================

typedef enum {
    SCAN_STATUS_IDLE        = 0x00,
    SCAN_STATUS_IN_PROGRESS = 0x01,
    SCAN_STATUS_COMPLETE    = 0x02,
    SCAN_STATUS_FAILED      = 0x03,
    SCAN_STATUS_NO_RESPONSE = 0x04,
} scan_status_t;

typedef struct __attribute__((packed)) {
    uint8_t  status;        // scan_status_t
    uint8_t  protocol;      // OBD protocol detected (ISO 15765-4, etc.)
    uint16_t ecu_count;     // Number of responding ECUs
    uint16_t pid_count;     // Number of supported PIDs found
    uint16_t reserved;
} comm_scan_status_t;

// ============================================================================
// Vehicle Info (sent after successful scan)
// ============================================================================

#define VIN_LENGTH          17
#define MAX_SUPPORTED_PIDS  96      // Mode 01 PIDs 0x00-0x60

typedef struct __attribute__((packed)) {
    char     vin[VIN_LENGTH + 1];   // VIN string (null-terminated)
    uint8_t  protocol;              // OBD protocol
    uint8_t  ecu_count;             // Number of ECUs
    uint8_t  supported_pids[MAX_SUPPORTED_PIDS / 8];  // Bitmap (12 bytes)
    uint16_t dtc_count;             // Number of stored DTCs
} comm_vehicle_info_t;

// ============================================================================
// Poll List Configuration
// ============================================================================

#define MAX_POLL_PIDS       20      // Max PIDs in active poll list

typedef struct __attribute__((packed)) {
    uint8_t  pid_count;                     // Number of PIDs in list
    uint8_t  poll_rate_hz;                  // Polling rate (1-50 Hz)
    uint16_t pids[MAX_POLL_PIDS];           // PID IDs to poll
} comm_poll_list_t;

// ============================================================================
// DTC Entry
// ============================================================================

typedef struct __attribute__((packed)) {
    uint16_t code;          // DTC code (P0XXX, etc.)
    uint8_t  type;          // 0=stored, 1=pending, 2=permanent
    uint8_t  system;        // 0=powertrain, 1=chassis, 2=body, 3=network
} comm_dtc_entry_t;

#define MAX_DTCS            32

typedef struct __attribute__((packed)) {
    uint8_t  dtc_count;                 // Number of DTCs
    uint8_t  reserved[3];               // Padding
    comm_dtc_entry_t dtcs[MAX_DTCS];    // DTC list
} comm_dtc_list_t;

// ============================================================================
// PID Metadata (sent after scan, stored in RAM on Display)
// ============================================================================

#define PID_META_NAME_LEN   32      // Max PID name length (null-terminated)
#define PID_META_UNIT_LEN   8       // Max unit string length (null-terminated)

typedef struct __attribute__((packed)) {
    uint16_t pid_id;                        // PID number
    uint8_t  unit;                          // pid_unit_t enum value (base unit)
    char     name[PID_META_NAME_LEN];       // Human-readable name
    char     unit_str[PID_META_UNIT_LEN];   // Unit display string ("RPM", "kPa", etc.)
} comm_pid_meta_t;
