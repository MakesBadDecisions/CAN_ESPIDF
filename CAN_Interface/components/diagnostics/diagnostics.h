/**
 * @file diagnostics.h
 * @brief Vehicle Diagnostics - VIN, DTCs, ECU Discovery
 * 
 * High-level diagnostic functions wrapping the OBD2 protocol stack.
 * Provides caching, async operations, and comm_link integration.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// DTC Info
// ============================================================================

#define DIAG_MAX_DTCS       64
#define DIAG_VIN_LEN        17

typedef struct {
    char        code[6];    // "P0123" format
    uint16_t    raw;        // Raw DTC value
    uint8_t     type;       // 0=stored, 1=pending, 2=permanent
} diag_dtc_t;

typedef struct {
    uint32_t    ecu_id;     // ECU response CAN ID (0x7E8+)
    char        name[16];   // Friendly name
    bool        responded;  // True if ECU responded to scan
} diag_ecu_t;

// ============================================================================
// Diagnostic State
// ============================================================================

typedef enum {
    DIAG_STATE_IDLE = 0,
    DIAG_STATE_SCANNING,
    DIAG_STATE_READING_VIN,
    DIAG_STATE_READING_DTCS,
    DIAG_STATE_CLEARING_DTCS,
} diag_state_t;

// ============================================================================
// Cached Vehicle Info
// ============================================================================

typedef struct {
    char        vin[DIAG_VIN_LEN + 1];
    bool        vin_valid;
    
    diag_dtc_t  dtcs[DIAG_MAX_DTCS];
    uint8_t     dtc_count;
    bool        dtcs_valid;
    
    diag_ecu_t  ecus[8];
    uint8_t     ecu_count;
    
    uint32_t    supported_pids[8];  // Bitmasks for PIDs 0x01-0xFF
    
    bool        mil_on;             // MIL (check engine) lamp status
    uint8_t     emission_dtc_count; // DTCs flagged by ECU (PID 0x01)
} diag_vehicle_info_t;

// ============================================================================
// API Functions
// ============================================================================

/**
 * @brief Initialize diagnostics module
 */
esp_err_t diagnostics_init(void);

/**
 * @brief Get current diagnostic state
 */
diag_state_t diagnostics_get_state(void);

/**
 * @brief Request VIN (async or cached)
 * @param vin Output buffer (18 bytes min)
 * @param force_refresh If true, re-read from vehicle
 * @return ESP_OK on success
 */
esp_err_t diagnostics_get_vin(char *vin, bool force_refresh);

/**
 * @brief Read DTCs from vehicle
 * @param dtcs Output array
 * @param max_count Max DTCs to return
 * @param out_count Actual count returned
 * @return ESP_OK on success
 */
esp_err_t diagnostics_read_dtcs(diag_dtc_t *dtcs, uint8_t max_count, uint8_t *out_count);

/**
 * @brief Clear DTCs
 * @return ESP_OK on success
 */
esp_err_t diagnostics_clear_dtcs(void);

/**
 * @brief Scan for ECUs that respond on OBD
 * @param ecus Output array
 * @param max_count Max ECUs
 * @param out_count Actual count found
 */
esp_err_t diagnostics_scan_ecus(diag_ecu_t *ecus, uint8_t max_count, uint8_t *out_count);

/**
 * @brief Check if specific PID is supported
 * @param pid OBD-II PID (0x01-0xFF)
 * @return true if supported
 */
bool diagnostics_is_pid_supported(uint8_t pid);

/**
 * @brief Read monitor status (PID 0x01) — MIL lamp + DTC count
 * @param mil_on Output: true if check engine light is on
 * @param dtc_count Output: number of emission DTCs
 * @return ESP_OK on success
 */
esp_err_t diagnostics_read_monitor_status(bool *mil_on, uint8_t *dtc_count);

/**
 * @brief Get cached vehicle info
 */
const diag_vehicle_info_t *diagnostics_get_vehicle_info(void);

/**
 * @brief Auto-scan routine: discover ECUs, read VIN, check supported PIDs
 * @return ESP_OK on success
 */
esp_err_t diagnostics_auto_scan(void);

