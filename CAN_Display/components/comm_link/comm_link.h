/**
 * @file comm_link.h
 * @brief Display Node UART Communication (RX from CAN Interface)
 * 
 * Handles bi-directional UART communication with the CAN Interface Node:
 * - RX: PID data, heartbeats, vehicle info, DTC lists
 * - TX: Config commands, heartbeats, log control
 */

#pragma once

#include "esp_err.h"
#include "comm_protocol.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

#define COMM_LINK_BAUD_RATE     2000000     // 2 Mbps (must match Interface)
#define COMM_LINK_TX_QUEUE_LEN  16
#define COMM_LINK_RX_BUFFER     2048
#define COMM_LINK_PID_STORE_MAX 64          // Max cached PID values
#define PID_META_STORE_MAX      96          // Max PID metadata entries (RAM only)

// ============================================================================
// Link State
// ============================================================================

typedef enum {
    COMM_LINK_DISCONNECTED = 0, // No heartbeat from Interface
    COMM_LINK_CONNECTED,        // Heartbeat received
    COMM_LINK_ERROR,            // UART error
} comm_link_state_t;

// ============================================================================
// Cached PID Value
// ============================================================================

typedef struct {
    uint16_t    pid_id;         // PID identifier
    float       value;          // Decoded value
    uint8_t     unit;           // Unit enum
    uint32_t    timestamp;      // When received (local tick)
    bool        valid;          // Entry in use
} pid_value_t;

// ============================================================================
// PID Metadata (RAM cache from scan, not persisted to flash)
// ============================================================================

typedef struct {
    uint16_t    pid_id;                         // PID number
    uint8_t     unit;                           // pid_unit_t enum value (base unit)
    char        name[PID_META_NAME_LEN];        // Human-readable name
    char        unit_str[PID_META_UNIT_LEN];    // Unit display string
    bool        valid;                          // Entry in use
} pid_meta_entry_t;

// ============================================================================
// Statistics
// ============================================================================

typedef struct {
    uint32_t    tx_frames;      // Frames transmitted
    uint32_t    rx_frames;      // Frames received
    uint32_t    tx_errors;      // TX failures
    uint32_t    rx_errors;      // RX CRC/framing errors
    uint32_t    rx_overflows;   // RX buffer overflows
    uint32_t    pid_updates;    // PID values received
    uint32_t    last_rx_tick;   // Last frame received (tick)
} comm_link_stats_t;

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief Callback for received PID data (optional, for real-time updates)
 */
typedef void (*comm_pid_callback_t)(const comm_pid_value_t *pid);

// ============================================================================
// API Functions
// ============================================================================

/**
 * @brief Initialize communication link
 * @return ESP_OK on success
 */
esp_err_t comm_link_init(void);

/**
 * @brief Start communication (UART and tasks)
 */
esp_err_t comm_link_start(void);

/**
 * @brief Stop communication
 */
esp_err_t comm_link_stop(void);

/**
 * @brief Get current link state
 */
comm_link_state_t comm_link_get_state(void);

/**
 * @brief Get link statistics
 */
const comm_link_stats_t* comm_link_get_stats(void);

/**
 * @brief Get cached PID value
 * @param pid_id PID to look up
 * @param out_value Output value struct
 * @return true if found and valid
 */
bool comm_link_get_pid(uint16_t pid_id, pid_value_t *out_value);

/**
 * @brief Get all cached PID values
 * @param out_values Array to fill (min COMM_LINK_PID_STORE_MAX)
 * @param max_count Maximum entries to return
 * @return Number of valid entries
 */
int comm_link_get_all_pids(pid_value_t *out_values, int max_count);

/**
 * @brief Register callback for real-time PID updates
 * @param cb Callback function (NULL to disable)
 */
void comm_link_register_pid_callback(comm_pid_callback_t cb);

/**
 * @brief Send heartbeat to Interface
 */
esp_err_t comm_link_send_heartbeat(void);

// ============================================================================
// Vehicle Scan API
// ============================================================================

/**
 * @brief Callback for scan completion
 * @param status Scan result status
 * @param info Vehicle info (NULL if scan failed)
 */
typedef void (*comm_scan_callback_t)(scan_status_t status, const comm_vehicle_info_t *info);

/**
 * @brief Request vehicle scan (ECU discovery, VIN, supported PIDs)
 * @param callback Function to call when scan completes
 * @return ESP_OK if request sent
 */
esp_err_t comm_link_request_scan(comm_scan_callback_t callback);

/**
 * @brief Get current scan status
 */
scan_status_t comm_link_get_scan_status(void);

/**
 * @brief Get last received vehicle info
 * @param out_info Output buffer
 * @return true if valid vehicle info available
 */
bool comm_link_get_vehicle_info(comm_vehicle_info_t *out_info);

/**
 * @brief Check if a specific PID is supported
 * @param pid_id PID number (0x00-0x60)
 * @return true if supported by vehicle
 */
bool comm_link_is_pid_supported(uint16_t pid_id);

/**
 * @brief Get list of all supported PIDs
 * @param out_pids Array to fill with PID numbers
 * @param max_count Size of array
 * @return Number of supported PIDs
 */
int comm_link_get_supported_pids(uint16_t *out_pids, int max_count);

// ============================================================================
// Poll List API
// ============================================================================

/**
 * @brief Send poll list to Interface (start streaming selected PIDs)
 * @param pids Array of PID numbers to poll
 * @param count Number of PIDs (max MAX_POLL_PIDS)
 * @param rate_hz Polling rate (1-50 Hz)
 * @return ESP_OK if sent
 */
esp_err_t comm_link_set_poll_list(const uint16_t *pids, uint8_t count, uint8_t rate_hz);

/**
 * @brief Stop polling (clear poll list)
 */
esp_err_t comm_link_clear_poll_list(void);

// ============================================================================
// PID Metadata API (populated from scan, stored in RAM)
// ============================================================================

/**
 * @brief Get PID name from metadata store
 * @param pid_id PID number
 * @return Name string or NULL if not found
 */
const char *comm_link_get_pid_name(uint16_t pid_id);

/**
 * @brief Get PID unit display string from metadata store
 * @param pid_id PID number
 * @return Unit string or NULL if not found
 */
const char *comm_link_get_pid_unit_str(uint16_t pid_id);

/**
 * @brief Get PID base unit enum from metadata store
 * @param pid_id PID number
 * @return pid_unit_t value, or PID_UNIT_NONE if not found
 */
pid_unit_t comm_link_get_pid_unit(uint16_t pid_id);

/**
 * @brief Get PID number by metadata index (for dropdown mapping)
 * @param index Index into valid metadata entries (0-based)
 * @return PID number or 0xFFFF if index out of range
 */
uint16_t comm_link_get_meta_pid_id(int index);

/**
 * @brief Get count of valid PID metadata entries
 */
int comm_link_get_pid_meta_count(void);
