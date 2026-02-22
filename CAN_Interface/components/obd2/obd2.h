/**
 * @file obd2.h
 * @brief OBD-II Protocol Stack
 * 
 * Provides OBD-II request/response handling over CAN bus including:
 * - Mode 01: Current Data
 * - Mode 02: Freeze Frame Data
 * - Mode 03: Read DTCs
 * - Mode 04: Clear DTCs
 * - Mode 07: Pending DTCs
 * - Mode 09: Vehicle Information (VIN, ECU names)
 * - Mode 22: Enhanced/Extended PIDs (manufacturer-specific)
 * 
 * Supports both single-frame and ISO-TP multi-frame transport.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// OBD-II Service Modes
// ============================================================================

#define OBD2_MODE_CURRENT_DATA      0x01
#define OBD2_MODE_FREEZE_FRAME      0x02
#define OBD2_MODE_READ_DTC          0x03
#define OBD2_MODE_CLEAR_DTC         0x04
#define OBD2_MODE_PENDING_DTC       0x07
#define OBD2_MODE_VEHICLE_INFO      0x09
#define OBD2_MODE_PERMANENT_DTC     0x0A
#define OBD2_MODE_EXTENDED          0x22

// Response mode = request mode + 0x40
#define OBD2_RESPONSE_OFFSET        0x40
#define OBD2_NEGATIVE_RESPONSE      0x7F

// ============================================================================
// CAN IDs for OBD-II
// ============================================================================

#define OBD2_CAN_BROADCAST_ID       0x7DF   // Functional addressing (all ECUs)
#define OBD2_CAN_ECM_TX_ID          0x7E0   // ECM request (physical)
#define OBD2_CAN_ECM_RX_ID          0x7E8   // ECM response
#define OBD2_CAN_TCM_TX_ID          0x7E1   // TCM request
#define OBD2_CAN_TCM_RX_ID          0x7E9   // TCM response
// Additional ECUs: 0x7E2-0x7E7 → 0x7EA-0x7EF

// ============================================================================
// ISO-TP Frame Types
// ============================================================================

#define ISOTP_FRAME_SINGLE          0x00
#define ISOTP_FRAME_FIRST           0x10
#define ISOTP_FRAME_CONSECUTIVE     0x20
#define ISOTP_FRAME_FLOW_CONTROL    0x30

// Flow Control flags
#define ISOTP_FC_CONTINUE           0x00
#define ISOTP_FC_WAIT               0x01
#define ISOTP_FC_ABORT              0x02

// ============================================================================
// Configuration
// ============================================================================

#define OBD2_MAX_PAYLOAD_LEN        256     // Max multi-frame reassembly buffer
#define OBD2_DEFAULT_TIMEOUT_MS     100     // Default response timeout
#define OBD2_ISOTP_SEP_TIME_MS      10      // Default separation time
#define OBD2_MAX_RETRIES            2       // Max retry on timeout

// ============================================================================
// Response Status
// ============================================================================

typedef enum {
    OBD2_STATUS_OK = 0,             // Valid response received
    OBD2_STATUS_TIMEOUT,            // No response within timeout
    OBD2_STATUS_PENDING,            // ECU sent 0x78 (response pending)
    OBD2_STATUS_NEGATIVE,           // Negative response (0x7F)
    OBD2_STATUS_INVALID_FRAME,      // Frame format error
    OBD2_STATUS_BUFFER_OVERFLOW,    // Response too large
    OBD2_STATUS_ISOTP_ERROR,        // ISO-TP sequence error
    OBD2_STATUS_CAN_ERROR,          // CAN bus error
    OBD2_STATUS_NOT_SUPPORTED,      // PID not supported by ECU
} obd2_status_t;

// ============================================================================
// Request Structure
// ============================================================================

typedef struct {
    uint8_t     mode;               // OBD-II mode (0x01, 0x22, etc.)
    uint16_t    pid;                // PID to request (8 or 16 bit)
    uint32_t    tx_id;              // CAN ID to send request (0x7DF for broadcast)
    uint32_t    timeout_ms;         // Response timeout (0 = use default)
} obd2_request_t;

// ============================================================================
// Response Structure
// ============================================================================

typedef struct {
    obd2_status_t   status;         // Response status
    uint8_t         mode;           // Response mode (request mode + 0x40)
    uint16_t        pid;            // Echoed PID
    uint32_t        ecu_id;         // Responding ECU's CAN ID
    uint8_t         data[OBD2_MAX_PAYLOAD_LEN];  // Raw response data (after mode/pid)
    uint8_t         data_len;       // Bytes of valid data
    uint8_t         negative_code;  // If status == NEGATIVE, the NRC
} obd2_response_t;

// ============================================================================
// ISO-TP Session State (internal)
// ============================================================================

typedef struct {
    bool        active;             // Multi-frame session in progress
    uint32_t    rx_id;              // Expected response CAN ID
    uint8_t     buffer[OBD2_MAX_PAYLOAD_LEN];
    uint16_t    total_len;          // Total expected bytes
    uint16_t    received_len;       // Bytes received so far
    uint8_t     next_seq;           // Next expected sequence number
    uint32_t    last_frame_tick;    // For timeout detection
} isotp_session_t;

// ============================================================================
// OBD-II Stack Configuration
// ============================================================================

typedef struct {
    uint32_t    default_timeout_ms; // Default response timeout
    uint8_t     max_retries;        // Retry count on timeout
    uint8_t     isotp_block_size;   // Flow control BS (0 = no limit)
    uint8_t     isotp_sep_time_ms;  // Flow control STmin
} obd2_config_t;

// ============================================================================
// API Functions
// ============================================================================

/**
 * @brief Initialize OBD-II stack
 * @param config Configuration (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t obd2_init(const obd2_config_t *config);

/**
 * @brief Send OBD-II request and wait for response
 * @param request Request parameters
 * @param response Response output (caller allocated)
 * @return ESP_OK if response received (check response->status)
 */
esp_err_t obd2_request(const obd2_request_t *request, obd2_response_t *response);

/**
 * @brief Request a single PID (convenience wrapper)
 * @param mode OBD-II mode (0x01, 0x22, etc.)
 * @param pid PID number
 * @param response Response output
 * @return ESP_OK if response received
 */
esp_err_t obd2_request_pid(uint8_t mode, uint16_t pid, obd2_response_t *response);

/**
 * @brief Request supported PIDs bitmap
 * @param base_pid Bitmap PID (0x00, 0x20, 0x40, etc.)
 * @param bitmap Output 32-bit bitmap
 * @return ESP_OK if response received
 */
esp_err_t obd2_get_supported_pids(uint8_t base_pid, uint32_t *bitmap);

/**
 * @brief Check if a specific PID is supported
 * @param pid PID to check
 * @param supported Output: true if supported
 * @return ESP_OK on success
 */
esp_err_t obd2_is_pid_supported(uint16_t pid, bool *supported);

/**
 * @brief Read VIN (Vehicle Identification Number)
 * @param vin_out Buffer for VIN string (17 chars + null)
 * @param buf_len Buffer length (should be >= 18)
 * @return ESP_OK if VIN retrieved
 */
esp_err_t obd2_read_vin(char *vin_out, size_t buf_len);

/**
 * @brief Read ECU name (Mode 09 PID 0x0A)
 * @param name_out Buffer for ECU name string (20 chars + null)
 * @param buf_len Buffer length (should be >= 21)
 * @return ESP_OK if ECU name retrieved
 */
esp_err_t obd2_read_ecu_name(char *name_out, size_t buf_len);

/**
 * @brief Read Calibration ID (Mode 09 PID 0x04)
 * @param cal_id_out Buffer for CalID string (first 16 chars + null)
 * @param buf_len Buffer length (should be >= 17)
 * @return ESP_OK if CalID retrieved
 */
esp_err_t obd2_read_cal_id(char *cal_id_out, size_t buf_len);

/**
 * @brief Read Calibration Verification Number (Mode 09 PID 0x06)
 * @param cvn_out Buffer for CVN hex string (first CVN as 8 hex chars + null)
 * @param buf_len Buffer length (should be >= 9)
 * @return ESP_OK if CVN retrieved
 */
esp_err_t obd2_read_cvn(char *cvn_out, size_t buf_len);

/**
 * @brief Read stored DTCs (Mode 03)
 * @param dtc_array Array to store DTC codes (each is uint16_t)
 * @param max_dtcs Maximum DTCs to read
 * @param dtc_count Output: number of DTCs found
 * @return ESP_OK if request successful
 */
esp_err_t obd2_read_dtcs(uint16_t *dtc_array, size_t max_dtcs, size_t *dtc_count);

/**
 * @brief Read pending DTCs (Mode 07)
 * @param dtc_array Array to store DTC codes (each is uint16_t)
 * @param max_dtcs Maximum DTCs to read
 * @param dtc_count Output: number of DTCs found
 * @return ESP_OK if request successful
 */
esp_err_t obd2_read_pending_dtcs(uint16_t *dtc_array, size_t max_dtcs, size_t *dtc_count);

/**
 * @brief Read permanent DTCs (Mode 0A) - survive clear
 * @param dtc_array Array to store DTC codes (each is uint16_t)
 * @param max_dtcs Maximum DTCs to read
 * @param dtc_count Output: number of DTCs found
 * @return ESP_OK if request successful
 */
esp_err_t obd2_read_permanent_dtcs(uint16_t *dtc_array, size_t max_dtcs, size_t *dtc_count);

/**
 * @brief Read monitor status (PID 0x01) — MIL lamp + DTC count
 * @param mil_on Output: true if MIL (check engine) is on
 * @param dtc_count Output: number of emission DTCs flagged
 * @return ESP_OK if request successful
 */
esp_err_t obd2_read_monitor_status(bool *mil_on, uint8_t *dtc_count);

/**
 * @brief Clear DTCs (Mode 04)
 * @return ESP_OK if clear command accepted
 */
esp_err_t obd2_clear_dtcs(void);

/**
 * @brief Process incoming CAN frame (called from RX task)
 * @param id CAN ID
 * @param data Frame data
 * @param len Frame length
 */
void obd2_process_frame(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Get string description of status code
 */
const char *obd2_status_str(obd2_status_t status);

/**
 * @brief Format DTC code as string (P0123, B1234, etc.)
 * @param dtc_raw Raw 2-byte DTC
 * @param out Output buffer (6 chars + null)
 */
void obd2_format_dtc(uint16_t dtc_raw, char *out);

