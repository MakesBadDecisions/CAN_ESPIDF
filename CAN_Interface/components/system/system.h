/**
 * @file system.h
 * @brief Core System Infrastructure - Logging, Config, Timing
 * 
 * Provides foundational services used by all components:
 * - Leveled logging with per-tag control
 * - NVS configuration storage
 * - Timing utilities
 * - Task helpers
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"

// ============================================================================
// Logging Macros - Wrappers around esp_log with consistent formatting
// ============================================================================

#define SYS_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
#define SYS_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define SYS_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define SYS_LOGD(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#define SYS_LOGV(tag, fmt, ...) ESP_LOGV(tag, fmt, ##__VA_ARGS__)

// ============================================================================
// System Initialization
// ============================================================================

/**
 * @brief Initialize system services (NVS, logging, timing)
 * @return ESP_OK on success
 */
esp_err_t system_init(void);

// ============================================================================
// Timing Utilities
// ============================================================================

/**
 * @brief Get current timestamp in microseconds since boot
 * @return Microseconds since boot (64-bit, overflow-safe)
 */
int64_t sys_time_us(void);

/**
 * @brief Get current timestamp in milliseconds since boot
 * @return Milliseconds since boot (32-bit, wraps after ~49 days)
 */
uint32_t sys_time_ms(void);

/**
 * @brief Check if timeout has elapsed (overflow-safe)
 * @param start_us Start timestamp from sys_time_us()
 * @param timeout_us Timeout duration in microseconds
 * @return true if elapsed, false otherwise
 */
bool sys_timeout_check(int64_t start_us, int64_t timeout_us);

/**
 * @brief Delay for specified milliseconds (FreeRTOS vTaskDelay wrapper)
 * @param ms Milliseconds to delay
 */
void sys_delay_ms(uint32_t ms);

// ============================================================================
// NVS Configuration
// ============================================================================

/**
 * @brief Get uint32 value from NVS config
 * @param key Configuration key
 * @param default_val Value to return if key not found
 * @return Stored value or default_val
 */
uint32_t sys_config_get_u32(const char *key, uint32_t default_val);

/**
 * @brief Set uint32 value in NVS config
 * @param key Configuration key
 * @param value Value to store
 * @return ESP_OK on success
 */
esp_err_t sys_config_set_u32(const char *key, uint32_t value);

/**
 * @brief Get string value from NVS config
 * @param key Configuration key
 * @param out_buf Buffer to receive string
 * @param buf_len Buffer length
 * @param default_val Value to copy if key not found (can be NULL)
 * @return ESP_OK on success, ESP_ERR_NVS_NOT_FOUND if not found
 */
esp_err_t sys_config_get_str(const char *key, char *out_buf, size_t buf_len, const char *default_val);

/**
 * @brief Set string value in NVS config
 * @param key Configuration key  
 * @param value String to store
 * @return ESP_OK on success
 */
esp_err_t sys_config_set_str(const char *key, const char *value);

// ============================================================================
// Device Information
// ============================================================================

/**
 * @brief Print device info to log (chip, flash, PSRAM, MAC)
 */
void sys_print_device_info(void);

/**
 * @brief Get free heap size in bytes
 * @return Free heap bytes
 */
uint32_t sys_get_free_heap(void);

/**
 * @brief Get minimum free heap since boot
 * @return Minimum free heap bytes
 */
uint32_t sys_get_min_free_heap(void);

