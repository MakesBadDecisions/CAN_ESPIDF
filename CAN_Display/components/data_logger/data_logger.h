/**
 * @file data_logger.h
 * @brief SD Card CSV Logging
 *
 * Logs PID data to SD card in a structured CSV format:
 *   - Header: version, creation time, vehicle info
 *   - Channel info: PID numbers, names, units
 *   - Channel data: time offset + values per poll cycle
 *
 * Files are named sequentially: log1.csv, log2.csv, ...
 * SD card shares SPI2 bus with touch controller (different CS pins).
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

#define LOGGER_MAX_CHANNELS     20      // Max PIDs to log simultaneously
#define LOGGER_WRITE_BUF_SIZE   4096    // Write buffer before flush
#define LOGGER_MOUNT_POINT      "/sdcard"
#define LOGGER_LOG_DIR          "/sdcard/logs"

// ============================================================================
// Logger State
// ============================================================================

typedef enum {
    LOGGER_STATE_UNINIT = 0,    // Not initialized (no SD card mounted)
    LOGGER_STATE_READY,         // SD mounted, no active session
    LOGGER_STATE_LOGGING,       // Actively writing to CSV
    LOGGER_STATE_ERROR,         // SD card error
} logger_state_t;

typedef struct {
    logger_state_t  state;
    uint32_t        rows_written;       // Data rows in current session
    uint32_t        bytes_written;      // Bytes written in current session
    uint32_t        file_number;        // Current log file number
    uint64_t        sd_total_bytes;     // SD card total capacity
    uint64_t        sd_free_bytes;      // SD card free space
} logger_status_t;

// ============================================================================
// API Functions
// ============================================================================

/**
 * @brief Initialize SD card (mount FAT filesystem on shared SPI bus)
 *
 * Must be called AFTER touch_init() since touch owns the SPI bus init.
 * Adds SD card as second device on SPI2_HOST with SD_CS_PIN.
 *
 * @return ESP_OK if SD card mounted, ESP_FAIL if no card or mount error
 */
esp_err_t logger_init(void);

/**
 * @brief Start a new logging session
 *
 * Creates a new CSV file (logN.csv), writes the header sections and
 * channel info based on currently polled PIDs, then begins capturing
 * data rows via the PID callback.
 *
 * IMU G-load columns (Lateral G, Longitudinal G) are automatically
 * appended when IMU hardware is present (HAS_IMU).
 *
 * @param pids      Array of PID IDs to log (columns)
 * @param pid_count Number of PIDs
 * @param vin       Vehicle VIN string (NULL if unavailable)
 * @return ESP_OK on success
 */
esp_err_t logger_start(const uint16_t *pids, uint8_t pid_count, const char *vin);

/**
 * @brief Stop logging session, flush buffers, close file
 */
void logger_stop(void);

/**
 * @brief Log one row of data (called from PID callback context)
 *
 * Reads current cached values for all session PIDs and appends a CSV row.
 * Uses buffered writes — actual SD flush happens when buffer is full or
 * on periodic timer.
 */
void logger_log_row(void);

/**
 * @brief Get current logger status
 */
logger_status_t logger_get_status(void);

/**
 * @brief Get current logger state
 */
logger_state_t logger_get_state(void);

/**
 * @brief Check if SD card is present and mounted
 */
bool logger_is_sd_mounted(void);

