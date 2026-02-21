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

/**
 * @brief Set backlight brightness (0-100%)
 *
 * On PWM-capable boards (Waveshare): smooth LEDC duty control.
 * On GPIO-only boards (CrowPanel): 0 = off, 1-100 = on.
 * Value is persisted to NVS (namespace "display", key "bl_pct").
 *
 * @param percent Brightness percentage (0 = off, 100 = full)
 * @return ESP_OK on success
 */
esp_err_t display_set_brightness(uint8_t percent);

/**
 * @brief Get current backlight brightness percentage
 * @return Brightness 0-100
 */
uint8_t display_get_brightness(void);

