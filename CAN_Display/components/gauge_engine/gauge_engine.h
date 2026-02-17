/**
 * @file gauge_engine.h
 * @brief Gauge Rendering & Layout
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Initialize gauge engine
 * @return ESP_OK on success
 */
esp_err_t gauge_init(void);

/**
 * @brief Render all gauges with current values
 */
void gauge_render(void);

