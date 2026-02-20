/**
 * @file touch_driver.h
 * @brief Touch Input Driver — HAL for Multiple Touch Controllers
 *
 * Supports multiple touch controllers via compile-time device selection:
 *   - XPT2046 (SPI, resistive) — CrowPanel 4.3"
 *   - CST820 (I2C, capacitive) — Waveshare 2.1" Round
 *   - GT911 (I2C, capacitive) — CrowPanel 5"/7" (future)
 *
 * All backends expose the same public API. Calibration functions are
 * only meaningful for resistive (XPT2046) — they are no-ops on capacitive.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize touch hardware and register LVGL input device
 *
 * Device-specific setup (SPI or I2C) is handled internally based on
 * the active device header. Starts a touch polling task on Core 0
 * and registers an LVGL input device.
 *
 * @return ESP_OK on success
 */
esp_err_t touch_init(void);

/**
 * @brief Check if touch calibration data exists in NVS
 *
 * Always returns true for capacitive touch (no calibration needed).
 *
 * @return true if valid calibration is stored (or not needed)
 */
bool touch_has_calibration(void);

/**
 * @brief Run interactive touch calibration (resistive only)
 *
 * For XPT2046: Creates a temporary LVGL screen with crosshair targets
 * at each corner. Blocks until the user touches all 4 corners.
 * For capacitive touch: Returns ESP_OK immediately (no-op).
 *
 * Caller must hold the display lock before calling this function.
 *
 * @return ESP_OK on success
 */
esp_err_t touch_start_calibration(void);

/**
 * @brief Erase stored calibration data from NVS (resistive only)
 *
 * For capacitive touch: Returns ESP_OK immediately (no-op).
 *
 * @return ESP_OK on success
 */
esp_err_t touch_clear_calibration(void);
