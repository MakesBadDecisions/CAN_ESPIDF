/**
 * @file can_driver.h
 * @brief CAN Hardware Abstraction Layer
 * 
 * Unified interface for CAN communication regardless of backend:
 * - MCP2515 (Phase 1)
 * - MCP2518FD (Phase 7)  
 * - TWAI (1-Wire GM)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ============================================================================
// CAN Frame Structure
// ============================================================================

/**
 * @brief CAN frame structure
 */
typedef struct {
    uint32_t id;            ///< CAN ID (11-bit standard or 29-bit extended)
    uint8_t  data[8];       ///< Frame data bytes
    uint8_t  dlc;           ///< Data length code (0-8)
    bool     extended;      ///< True if 29-bit extended ID
    bool     rtr;           ///< Remote transmission request flag
} can_frame_t;

// ============================================================================
// CAN Driver Backend Types
// ============================================================================

/**
 * @brief CAN controller backend type
 */
typedef enum {
    CAN_BACKEND_MCP2515,    ///< MCP2515 SPI CAN 2.0B controller
    CAN_BACKEND_MCP2518FD,  ///< MCP2518FD SPI CAN FD controller
    CAN_BACKEND_TWAI,       ///< ESP32 native TWAI peripheral
} can_backend_t;

/**
 * @brief CAN bus bitrate
 */
typedef enum {
    CAN_BITRATE_125K  = 125000,
    CAN_BITRATE_250K  = 250000,
    CAN_BITRATE_500K  = 500000,
    CAN_BITRATE_1M    = 1000000,
} can_bitrate_t;

/**
 * @brief Crystal oscillator frequency (for MCP2515/MCP2518FD)
 */
typedef enum {
    CAN_CRYSTAL_8MHZ  = 8000000,
    CAN_CRYSTAL_16MHZ = 16000000,
    CAN_CRYSTAL_20MHZ = 20000000,
    CAN_CRYSTAL_40MHZ = 40000000,
} can_crystal_t;

// ============================================================================
// CAN Driver Status
// ============================================================================

/**
 * @brief CAN driver operational state
 */
typedef enum {
    CAN_STATE_UNINIT,       ///< Not initialized
    CAN_STATE_STOPPED,      ///< Initialized but not running
    CAN_STATE_RUNNING,      ///< Normal operation
    CAN_STATE_BUS_OFF,      ///< Bus-off due to errors
    CAN_STATE_ERROR_PASSIVE,///< Error-passive state
    CAN_STATE_ERROR_WARNING,///< Error warning state
} can_state_t;

/**
 * @brief CAN driver statistics
 */
typedef struct {
    uint32_t tx_frames;     ///< Total frames transmitted
    uint32_t rx_frames;     ///< Total frames received
    uint32_t tx_errors;     ///< Transmit errors
    uint32_t rx_errors;     ///< Receive errors
    uint32_t rx_overflows;  ///< RX buffer overflows
    uint8_t  tec;           ///< Transmit error counter
    uint8_t  rec;           ///< Receive error counter
} can_stats_t;

/**
 * @brief CAN driver status structure
 */
typedef struct {
    can_state_t state;      ///< Current operational state
    can_stats_t stats;      ///< Statistics counters
    bool        bus_active; ///< True if bus activity detected
} can_status_t;

// ============================================================================
// CAN Driver Configuration
// ============================================================================

/**
 * @brief CAN driver configuration
 */
typedef struct {
    can_backend_t backend;      ///< Controller backend type
    can_bitrate_t bitrate;      ///< CAN bus bitrate
    can_crystal_t crystal;      ///< Crystal frequency (SPI controllers)
    uint8_t       rx_queue_len; ///< RX queue length (frames)
    bool          loopback;     ///< Enable loopback mode for testing
    bool          listen_only;  ///< Enable listen-only mode
} can_config_t;

/**
 * @brief Default configuration for MCP2515 @ 500kbps
 * Change to CAN_BACKEND_MCP2518FD + CAN_CRYSTAL_40MHZ when hardware is swapped
 */
#define CAN_CONFIG_DEFAULT() { \
    .backend = CAN_BACKEND_MCP2515, \
    .bitrate = CAN_BITRATE_500K, \
    .crystal = CAN_CRYSTAL_8MHZ, \
    .rx_queue_len = 32, \
    .loopback = false, \
    .listen_only = false, \
}

// ============================================================================
// CAN Driver API
// ============================================================================

/**
 * @brief Initialize the CAN driver
 * @return ESP_OK on success
 */
esp_err_t can_driver_init(void);

/**
 * @brief Initialize with custom configuration
 * @param config Driver configuration
 * @return ESP_OK on success
 */
esp_err_t can_driver_init_with_config(const can_config_t *config);

/**
 * @brief Deinitialize the CAN driver
 * @return ESP_OK on success
 */
esp_err_t can_driver_deinit(void);

/**
 * @brief Start CAN communication
 * @return ESP_OK on success
 */
esp_err_t can_driver_start(void);

/**
 * @brief Stop CAN communication
 * @return ESP_OK on success
 */
esp_err_t can_driver_stop(void);

/**
 * @brief Send a CAN frame
 * @param frame Pointer to frame to send
 * @param timeout_ms Timeout in milliseconds (0 = no wait)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if TX buffers full
 */
esp_err_t can_driver_send(const can_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Receive a CAN frame from the RX queue
 * @param frame Pointer to store received frame
 * @param timeout_ms Timeout in milliseconds (portMAX_DELAY for infinite)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no frame available
 */
esp_err_t can_driver_receive(can_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Get the number of frames waiting in RX queue
 * @return Number of frames in queue
 */
uint32_t can_driver_get_rx_queue_count(void);

/**
 * @brief Get driver status
 * @param status Pointer to store status
 * @return ESP_OK on success
 */
esp_err_t can_driver_get_status(can_status_t *status);

/**
 * @brief Abort all pending TX buffer transmissions
 * Call after a request timeout to free TX buffers for the next request.
 * @return ESP_OK on success
 */
esp_err_t can_driver_abort_tx(void);

/**
 * @brief Reset error counters and clear error state
 * @return ESP_OK on success
 */
esp_err_t can_driver_clear_errors(void);

/**
 * @brief Get RX queue handle for direct use with xQueueReceive
 * @return Queue handle or NULL if not initialized
 */
QueueHandle_t can_driver_get_rx_queue(void);

// ============================================================================
// RX Callback Registration
// ============================================================================

/**
 * @brief Callback function type for received CAN frames
 * @param frame Pointer to received frame (valid only during callback)
 */
typedef void (*can_rx_callback_t)(const can_frame_t *frame);

/**
 * @brief Register a callback for received CAN frames
 * 
 * The callback is invoked from the CAN driver's RX task context for each
 * received frame. Multiple callbacks can be registered (up to 4).
 * 
 * @param callback Function to call on frame reception
 * @return ESP_OK on success, ESP_ERR_NO_MEM if max callbacks reached
 */
esp_err_t can_driver_register_rx_callback(can_rx_callback_t callback);

/**
 * @brief Unregister a previously registered callback
 * @param callback Function to remove
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not registered
 */
esp_err_t can_driver_unregister_rx_callback(can_rx_callback_t callback);

/**
 * @brief Dispatch frame to all registered callbacks (internal use)
 * @param frame Received frame
 */
void can_driver_dispatch_rx_callbacks(const can_frame_t *frame);

