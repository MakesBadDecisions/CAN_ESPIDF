/**
 * @file pcf85063.h
 * @brief PCF85063A RTC Driver for Waveshare ESP32-S3-Touch-LCD-2.1
 *
 * I2C real-time clock at address 0x51 on the shared bus.
 * Stores UTC time — timezone offset is applied at display time.
 *
 * Time registers (BCD encoded):
 *   0x04 Seconds  (0-59, bit 7 = OS flag)
 *   0x05 Minutes  (0-59)
 *   0x06 Hours    (0-23, 24h mode)
 *   0x07 Days     (1-31)
 *   0x08 Weekdays (0-6, Sunday=0)
 *   0x09 Months   (1-12)
 *   0x0A Years    (0-99, relative to 2000)
 *
 * Design: RTC stores UTC. Timezone offset from the client browser is
 * saved to NVS and applied when formatting local time for display/logging.
 */

#pragma once

#include "esp_err.h"
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// API
// ============================================================================

/**
 * @brief Initialize PCF85063 RTC
 *
 * Verifies communication, clears oscillator-stop flag if set.
 * On boards without HAS_RTC, returns ESP_ERR_NOT_SUPPORTED.
 *
 * @return ESP_OK, ESP_ERR_NOT_SUPPORTED, or I2C error
 */
esp_err_t pcf85063_init(void);

/**
 * @brief Check if the RTC has been initialized
 */
bool pcf85063_is_initialized(void);

/**
 * @brief Set RTC time from a UTC struct tm
 *
 * Converts each field to BCD and writes to the PCF85063 time registers.
 * Also sets the ESP32 system clock via settimeofday().
 *
 * @param utc Broken-down UTC time
 * @return ESP_OK on success
 */
esp_err_t pcf85063_set_time(const struct tm *utc);

/**
 * @brief Set RTC time from a Unix epoch (UTC seconds)
 *
 * Convenience wrapper: converts epoch to struct tm, then calls
 * pcf85063_set_time() and settimeofday().
 *
 * @param epoch_sec Unix epoch in seconds (UTC)
 * @return ESP_OK on success
 */
esp_err_t pcf85063_set_epoch(time_t epoch_sec);

/**
 * @brief Read current time from the RTC
 *
 * Reads BCD registers, converts to struct tm (UTC).
 *
 * @param utc Output: broken-down UTC time
 * @return ESP_OK on success
 */
esp_err_t pcf85063_get_time(struct tm *utc);

/**
 * @brief Read current time from the RTC as Unix epoch (UTC)
 *
 * @param epoch_sec Output: Unix epoch in seconds
 * @return ESP_OK on success
 */
esp_err_t pcf85063_get_epoch(time_t *epoch_sec);

/**
 * @brief Sync ESP32 system clock from the RTC
 *
 * Reads the RTC and calls settimeofday() to align the system clock.
 * Call once during boot after pcf85063_init().
 *
 * @return ESP_OK on success
 */
esp_err_t pcf85063_sync_to_system(void);

/**
 * @brief Save timezone offset to NVS
 * @param offset_min Offset from UTC in minutes (e.g. -300 for EST, +60 for CET)
 *                   NOTE: JavaScript getTimezoneOffset() returns the INVERSE sign.
 *                   Caller should negate it before passing here.
 */
esp_err_t pcf85063_save_tz_offset(int16_t offset_min);

/**
 * @brief Load timezone offset from NVS
 * @return Offset in minutes from UTC (0 if not set)
 */
int16_t pcf85063_get_tz_offset(void);
