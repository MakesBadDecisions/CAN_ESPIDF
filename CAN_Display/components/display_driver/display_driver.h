/**
 * @file display_driver.h
 * @brief Display Hardware Abstraction Layer
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize display hardware (RGB parallel LCD)
 * @return ESP_OK on success
 */
esp_err_t display_init(void);

/**
 * @brief Clear display to background color
 */
void display_clear(uint16_t color);

/**
 * @brief Flush framebuffer to display
 */
void display_flush(void);

