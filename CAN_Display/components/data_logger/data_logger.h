/**
 * @file data_logger.h
 * @brief SD Card CSV Logging
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Initialize SD card and logging system
 * @return ESP_OK on success
 */
esp_err_t logger_init(void);

/**
 * @brief Start logging session
 * @return ESP_OK on success
 */
esp_err_t logger_start(void);

/**
 * @brief Stop logging session and flush data
 */
void logger_stop(void);

