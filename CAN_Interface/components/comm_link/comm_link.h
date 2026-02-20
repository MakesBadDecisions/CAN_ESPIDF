/**
 * @file comm_link.h
 * @brief UART Communication Link to Display Node
 * 
 * Handles bi-directional UART communication with the Display Node:
 * - TX: PID data, heartbeats, DTC lists, vehicle info
 * - RX: Config commands, poll list updates, log control
 */

#pragma once

#include "esp_err.h"
#include "comm_protocol.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

#define COMM_LINK_BAUD_RATE     2000000     // 2 Mbps
#define COMM_LINK_TX_QUEUE_LEN  32
#define COMM_LINK_RX_BUFFER     2048

// ============================================================================
// Link State
// ============================================================================

typedef enum {
    COMM_LINK_DISCONNECTED = 0, // No heartbeat from peer
    COMM_LINK_CONNECTED,        // Heartbeat received
    COMM_LINK_ERROR,            // UART error
} comm_link_state_t;

// ============================================================================
// Statistics
// ============================================================================

typedef struct {
    uint32_t    tx_frames;      // Frames transmitted
    uint32_t    rx_frames;      // Frames received
    uint32_t    tx_errors;      // TX failures
    uint32_t    rx_errors;      // RX CRC/framing errors
    uint32_t    rx_overflows;   // RX buffer overflows
    uint32_t    last_rx_tick;   // Last frame received (tick)
} comm_link_stats_t;

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief Callback for received config commands
 */
typedef void (*comm_cmd_callback_t)(comm_msg_type_t type, const uint8_t *payload, uint16_t len);

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
 * @brief Send a PID value to display node
 * @param pid PID number
 * @param value Decoded value
 * @param unit Unit enum
 * @return ESP_OK on success
 */
esp_err_t comm_link_send_pid(uint16_t pid, float value, uint8_t unit);

/**
 * @brief Send multiple PID values in a batch
 * @param values Array of PID values
 * @param count Number of values
 * @return ESP_OK on success
 */
esp_err_t comm_link_send_pid_batch(const comm_pid_value_t *values, uint8_t count);

/**
 * @brief Send heartbeat
 */
esp_err_t comm_link_send_heartbeat(void);

/**
 * @brief Send DTC list
 * @param dtcs Array of raw DTC codes
 * @param count Number of DTCs
 */
esp_err_t comm_link_send_dtcs(const uint16_t *dtcs, uint8_t count);

/**
 * @brief Send vehicle info (VIN)
 * @param vin VIN string (17 chars)
 */
esp_err_t comm_link_send_vin(const char *vin);

/**
 * @brief Send full vehicle info (VIN, supported PIDs, etc.)
 * @param info Vehicle info structure
 */
esp_err_t comm_link_send_vehicle_info(const comm_vehicle_info_t *info);

/**
 * @brief Send scan status update
 * @param status Current scan status
 * @param ecu_count Number of ECUs found
 * @param pid_count Number of supported PIDs found
 */
esp_err_t comm_link_send_scan_status(scan_status_t status, uint16_t ecu_count, uint16_t pid_count);

/**
 * @brief Send PID metadata batch to Display Node
 * @param entries Array of metadata entries
 * @param count Number of entries in batch
 * @return ESP_OK on success
 */
esp_err_t comm_link_send_pid_metadata(const comm_pid_meta_t *entries, uint8_t count);

/**
 * @brief Register callback for received commands
 */
esp_err_t comm_link_register_cmd_callback(comm_cmd_callback_t callback);

/**
 * @brief Get communication statistics
 */
esp_err_t comm_link_get_stats(comm_link_stats_t *stats);

