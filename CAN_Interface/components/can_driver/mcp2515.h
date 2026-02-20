/**
 * @file mcp2515.h
 * @brief MCP2515 SPI CAN Controller Driver
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "can_driver.h"

// ============================================================================
// MCP2515 Driver Configuration
// ============================================================================

/**
 * @brief MCP2515 driver configuration
 */
typedef struct {
    spi_host_device_t spi_host;     ///< SPI host (SPI2_HOST or SPI3_HOST)
    int               pin_mosi;      ///< MOSI GPIO
    int               pin_miso;      ///< MISO GPIO
    int               pin_sck;       ///< SCK GPIO
    int               pin_cs;        ///< Chip select GPIO
    int               pin_int;       ///< Interrupt GPIO
    int               spi_clock_hz;  ///< SPI clock frequency (max 10MHz)
    can_crystal_t     crystal_freq;  ///< Crystal/oscillator frequency
    can_bitrate_t     can_bitrate;   ///< Desired CAN bus bitrate
    uint8_t           rx_queue_len;  ///< RX queue length
    bool              loopback;      ///< Loopback mode
    bool              listen_only;   ///< Listen-only mode
} mcp2515_config_t;

/**
 * @brief Default MCP2515 configuration using ESP32-S3-N16R8-DevKitC pins
 */
#define MCP2515_CONFIG_DEFAULT() { \
    .spi_host = SPI2_HOST, \
    .pin_mosi = 11, \
    .pin_miso = 13, \
    .pin_sck = 12, \
    .pin_cs = 10, \
    .pin_int = 9, \
    .spi_clock_hz = 8000000, \
    .crystal_freq = CAN_CRYSTAL_8MHZ, \
    .can_bitrate = CAN_BITRATE_500K, \
    .rx_queue_len = 32, \
    .loopback = false, \
    .listen_only = false, \
}

// ============================================================================
// MCP2515 Driver API
// ============================================================================

/**
 * @brief Initialize the MCP2515 driver
 * @param config Configuration structure
 * @return ESP_OK on success
 */
esp_err_t mcp2515_init(const mcp2515_config_t *config);

/**
 * @brief Deinitialize the MCP2515 driver
 * @return ESP_OK on success
 */
esp_err_t mcp2515_deinit(void);

/**
 * @brief Set MCP2515 to normal operation mode
 * @return ESP_OK on success
 */
esp_err_t mcp2515_start(void);

/**
 * @brief Put MCP2515 into configuration mode (stops CAN)
 * @return ESP_OK on success
 */
esp_err_t mcp2515_stop(void);

/**
 * @brief Send a CAN frame
 * @param frame Frame to send
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t mcp2515_send(const can_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Receive a CAN frame from the RX queue
 * @param frame Pointer to store received frame
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t mcp2515_receive(can_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Get number of frames in RX queue
 * @return Queue count
 */
uint32_t mcp2515_get_rx_queue_count(void);

/**
 * @brief Get the RX queue handle
 * @return Queue handle
 */
QueueHandle_t mcp2515_get_rx_queue(void);

/**
 * @brief Get driver statistics
 * @param stats Pointer to store statistics
 * @return ESP_OK on success
 */
esp_err_t mcp2515_get_stats(can_stats_t *stats);

/**
 * @brief Get current operational state
 * @return Current state
 */
can_state_t mcp2515_get_state(void);

/**
 * @brief Abort all pending TX buffer transmissions
 * Use after request timeout to free TX buffers for the next request.
 * @return ESP_OK on success
 */
esp_err_t mcp2515_abort_all_tx(void);

/**
 * @brief Clear error counters and flags
 * @return ESP_OK on success
 */
esp_err_t mcp2515_clear_errors(void);

/**
 * @brief Read a single register
 * @param addr Register address
 * @return Register value
 */
uint8_t mcp2515_read_register(uint8_t addr);

/**
 * @brief Write a single register
 * @param addr Register address
 * @param value Value to write
 */
void mcp2515_write_register(uint8_t addr, uint8_t value);

/**
 * @brief Check if MCP2515 is responding on SPI bus
 * @return true if device responds correctly
 */
bool mcp2515_probe(void);

/**
 * @brief Configure hardware acceptance filters for specific CAN IDs
 *
 * Sets the MCP2515 masks and filters to accept only the specified standard
 * (11-bit) CAN IDs. Requires entering config mode briefly. Both RX buffers
 * share the same filter set. Up to 6 IDs can be filtered (RXF0-RXF5).
 * To accept more IDs, use a looser mask that covers the range.
 *
 * @param ids      Array of standard CAN IDs to accept
 * @param count    Number of IDs (1-6, or 0 to accept all)
 * @return ESP_OK on success
 */
esp_err_t mcp2515_set_filters(const uint32_t *ids, uint8_t count);

/**
 * @brief Clear hardware filters (accept all CAN frames)
 * @return ESP_OK on success
 */
esp_err_t mcp2515_clear_filters(void);
