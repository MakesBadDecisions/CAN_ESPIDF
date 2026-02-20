/**
 * @file mcp2518fd.h
 * @brief MCP2518FD CAN FD Controller Driver (Classic CAN Mode)
 *
 * SPI-based driver for the Microchip MCP2518FD / MCP251863 CAN FD
 * controller, operating in CAN 2.0 (classic) mode for OBD-II
 * compatibility.
 *
 * Architecture:
 *   - FIFO1: TX (8 deep, 8-byte payload, unlimited retransmit)
 *   - FIFO2: RX (16 deep, 8-byte payload, timestamp enabled)
 *   - Filter 0 → FIFO2 with mask=0 (accept all messages)
 *   - GPIO ISR → FreeRTOS task notification → ISR task
 *   - Incoming frames queued to FreeRTOS queue
 *
 * Designed as a drop-in replacement for the MCP2515 backend in the
 * can_driver HAL. Shares the same SPI2 bus, CS, and INT pins.
 *
 * @note SPI clock up to 20 MHz. Crystal: 20 or 40 MHz.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can_driver.h"         // can_frame_t, can_stats_t, can_state_t

/**
 * @brief MCP2518FD driver configuration
 *
 * Mirrors mcp2515_config_t for consistent HAL integration.
 * Same SPI bus, same CS/INT pins — physical swap only.
 */
typedef struct {
    spi_host_device_t spi_host;     ///< SPI host (SPI2_HOST)
    int pin_mosi;                   ///< MOSI GPIO
    int pin_miso;                   ///< MISO GPIO
    int pin_sck;                    ///< SCK GPIO
    int pin_cs;                     ///< Chip Select GPIO
    int pin_int;                    ///< Interrupt GPIO (active-low)
    uint32_t spi_clock_hz;          ///< SPI clock frequency (max 20 MHz)
    uint32_t crystal_freq;          ///< Oscillator crystal frequency (Hz)
    uint32_t can_bitrate;           ///< CAN bus bitrate (125000/250000/500000)
    uint8_t  rx_queue_len;          ///< FreeRTOS RX queue depth
    bool     loopback;              ///< Enable internal loopback mode
    bool     listen_only;           ///< Enable listen-only mode
} mcp2518fd_config_t;

// ============================================================================
// API — Same surface as mcp2515.h for HAL dispatch
// ============================================================================

/**
 * @brief Initialize the MCP2518FD driver
 *
 * Configures SPI bus, resets device, verifies communication,
 * sets bit timing, configures FIFOs and filters, creates ISR task.
 * Device remains in Configuration mode after init (call start() to go live).
 *
 * @param config  Driver configuration
 * @return ESP_OK on success
 */
esp_err_t mcp2518fd_init(const mcp2518fd_config_t *config);

/**
 * @brief Deinitialize the MCP2518FD driver
 *
 * Stops the ISR task, puts device in config mode, frees all resources.
 *
 * @return ESP_OK on success
 */
esp_err_t mcp2518fd_deinit(void);

/**
 * @brief Start CAN communication
 *
 * Transitions the device from Configuration to Normal CAN 2.0,
 * Listen-Only, or Loopback mode depending on configuration.
 *
 * @return ESP_OK on success
 */
esp_err_t mcp2518fd_start(void);

/**
 * @brief Stop CAN communication
 *
 * Transitions the device back to Configuration mode.
 *
 * @return ESP_OK on success
 */
esp_err_t mcp2518fd_stop(void);

/**
 * @brief Send a CAN frame
 *
 * Loads the frame into the TX FIFO and requests transmission.
 * Returns ESP_ERR_TIMEOUT if TX FIFO is full.
 *
 * @param frame      Frame to transmit
 * @param timeout_ms Unused (non-blocking check)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if FIFO full
 */
esp_err_t mcp2518fd_send(const can_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Receive a CAN frame from the RX queue
 *
 * Blocks up to timeout_ms waiting for a frame from the ISR task.
 *
 * @param frame      Output frame buffer
 * @param timeout_ms Maximum wait time (UINT32_MAX = forever)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no frame received
 */
esp_err_t mcp2518fd_receive(can_frame_t *frame, uint32_t timeout_ms);

/**
 * @brief Get the number of frames waiting in the RX queue
 */
uint32_t mcp2518fd_get_rx_queue_count(void);

/**
 * @brief Get the FreeRTOS RX queue handle
 */
QueueHandle_t mcp2518fd_get_rx_queue(void);

/**
 * @brief Get driver statistics (frame counts, error counters)
 */
esp_err_t mcp2518fd_get_stats(can_stats_t *stats);

/**
 * @brief Get the current CAN bus state
 */
can_state_t mcp2518fd_get_state(void);

/**
 * @brief Abort all pending TX operations
 *
 * Sets ABAT in CiCON, waits for TX FIFOs to drain,
 * then clears ABAT.
 */
esp_err_t mcp2518fd_abort_all_tx(void);

/**
 * @brief Clear errors and recover from bus-off
 *
 * Clears diagnostic registers and error flags.
 * If bus-off, cycles through config mode for recovery.
 */
esp_err_t mcp2518fd_clear_errors(void);

/**
 * @brief Probe for MCP2518FD on the SPI bus
 *
 * Sends a reset, waits for oscillator ready, and verifies
 * the device enters Configuration mode.
 *
 * @return true if device responds correctly
 */
bool mcp2518fd_probe(void);

/**
 * @brief Read a 32-bit register (exposed for diagnostics)
 */
uint32_t mcp2518fd_read_register(uint16_t addr);

/**
 * @brief Write a 32-bit register (exposed for diagnostics)
 */
void mcp2518fd_write_register(uint16_t addr, uint32_t value);
