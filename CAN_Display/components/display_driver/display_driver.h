/**
 * @file display_driver.h
 * @brief Display Hardware Abstraction Layer
 * 
 * Manages RGB LCD panel and LVGL integration for CrowPanel displays.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize display hardware (RGB parallel LCD) and LVGL
 * @return ESP_OK on success
 */
esp_err_t display_init(void);

/**
 * @brief Clear display to background color
 * @param color RGB565 color value
 */
void display_clear(uint16_t color);

/**
 * @brief Flush framebuffer to display (handled automatically by LVGL task)
 */
void display_flush(void);

/**
 * @brief Acquire LVGL mutex for thread-safe UI operations
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if lock acquired, false on timeout
 */
bool display_lock(uint32_t timeout_ms);

/**
 * @brief Release LVGL mutex
 */
void display_unlock(void);

