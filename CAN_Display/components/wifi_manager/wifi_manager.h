/**
 * @file wifi_manager.h
 * @brief WiFi AP & Configuration Server
 */

#pragma once

#include "esp_err.h"

/**
 * @brief Initialize WiFi in AP mode
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Start HTTP config server
 */
esp_err_t wifi_manager_start(void);

