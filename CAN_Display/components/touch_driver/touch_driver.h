/**
 * @file touch_driver.h
 * @brief Touch Input Driver for XPT2046 Resistive Touch Controller
 *
 * Runs touch SPI reads on Core 0 in a dedicated task to avoid
 * contention with the LCD bounce buffer DMA ISR on Core 1.
 * Provides 4-corner calibration with NVS persistence.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize touch hardware and register LVGL input device
 *
 * Sets up SPI bus for XPT2046, starts the touch polling task on Core 0,
 * and registers an LVGL input device. Loads calibration from NVS if available.
 *
 * @return ESP_OK on success
 */
esp_err_t touch_init(void);

/**
 * @brief Check if touch calibration data exists in NVS
 * @return true if valid calibration is stored
 */
bool touch_has_calibration(void);

/**
 * @brief Run interactive 4-corner touch calibration
 *
 * Creates a temporary LVGL screen with crosshair targets at each corner.
 * Blocks until the user touches all 4 corners, then saves calibration to NVS.
 * Caller must hold the display lock before calling this function.
 *
 * @return ESP_OK on success
 */
esp_err_t touch_start_calibration(void);

/**
 * @brief Erase stored calibration data from NVS
 *
 * Forces recalibration on next boot.
 *
 * @return ESP_OK on success
 */
esp_err_t touch_clear_calibration(void);
