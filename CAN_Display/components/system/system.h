/**
 * @file system.h
 * @brief Display Node System Infrastructure
 * 
 * Core system services for the display node - logging, NVS, timing.
 * Similar to CAN Interface system but with display-specific config.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"

// ============================================================================
// Log Macros
// ============================================================================

#define SYS_LOGE(fmt, ...) ESP_LOGE("sys", fmt, ##__VA_ARGS__)
#define SYS_LOGW(fmt, ...) ESP_LOGW("sys", fmt, ##__VA_ARGS__)
#define SYS_LOGI(fmt, ...) ESP_LOGI("sys", fmt, ##__VA_ARGS__)
#define SYS_LOGD(fmt, ...) ESP_LOGD("sys", fmt, ##__VA_ARGS__)
#define SYS_LOGV(fmt, ...) ESP_LOGV("sys", fmt, ##__VA_ARGS__)

// ============================================================================
// System API
// ============================================================================

/**
 * @brief Initialize system (NVS, logging levels)
 * @return ESP_OK on success
 */
esp_err_t system_init(void);

/**
 * @brief Print device info (chip, flash, PSRAM, MAC)
 */
void sys_print_device_info(void);

/**
 * @brief Get current timestamp in microseconds
 */
int64_t sys_time_us(void);

/**
 * @brief Get current timestamp in milliseconds
 */
uint32_t sys_time_ms(void);

/**
 * @brief Get free heap memory
 */
uint32_t sys_get_free_heap(void);

