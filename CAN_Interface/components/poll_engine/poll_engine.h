/**
 * @file poll_engine.h
 * @brief Smart Polling Scheduler
 * 
 * Manages PID polling with configurable priorities:
 *   Priority 1: Slow (~5s interval)
 *   Priority 2: Low (~1s interval)
 *   Priority 3: Normal (~200ms interval)
 *   Priority 4: High (~100ms interval)
 *   Priority 5: Realtime (~100ms, 10Hz)
 * 
 * Priorities are stored in NVS and user-adjustable via display node.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Priority Levels
// ============================================================================

typedef enum {
    POLL_PRIORITY_SLOW     = 1,  // ~5 second interval
    POLL_PRIORITY_LOW      = 2,  // ~1 second interval
    POLL_PRIORITY_NORMAL   = 3,  // ~200ms interval (default)
    POLL_PRIORITY_HIGH     = 4,  // ~100ms interval
    POLL_PRIORITY_REALTIME = 5,  // ~100ms (10Hz)
} poll_priority_t;

#define POLL_PRIORITY_MIN   1
#define POLL_PRIORITY_MAX   5
#define POLL_PRIORITY_DEFAULT POLL_PRIORITY_NORMAL

// ============================================================================
// Poll Job Structure
// ============================================================================

typedef struct {
    uint16_t    pid;            // PID number
    uint8_t     mode;           // OBD mode (0x01, 0x22)
    uint8_t     priority;       // 1-5 priority level
    uint32_t    interval_ms;    // Current polling interval
    uint32_t    next_poll_tick; // Next poll time (tick count)
    uint32_t    last_poll_tick; // Last successful poll time
    float       last_value;     // Last decoded value
    bool        value_changed;  // True if value changed since last read
    bool        enabled;        // True if polling enabled
    uint16_t    success_count;  // Successful polls
    uint16_t    fail_count;     // Failed polls
} poll_job_t;

// ============================================================================
// Poll Engine Statistics
// ============================================================================

typedef struct {
    uint32_t    polls_per_second;   // Current poll rate
    uint32_t    total_polls;        // Total polls since start
    uint32_t    total_failures;     // Total failed polls
    uint32_t    active_jobs;        // Number of active poll jobs
    uint32_t    queue_depth;        // Pending polls in queue
} poll_stats_t;

// ============================================================================
// Configuration
// ============================================================================

#define POLL_ENGINE_MAX_JOBS    64  // Maximum PIDs to poll

typedef struct {
    uint8_t     max_outstanding;    // Max concurrent requests (default 3)
    bool        adaptive_interval;  // Adjust interval based on success rate
    bool        persist_priorities; // Save priorities to NVS
} poll_engine_config_t;

#define POLL_ENGINE_CONFIG_DEFAULT() { \
    .max_outstanding = 3, \
    .adaptive_interval = true, \
    .persist_priorities = true, \
}

// ============================================================================
// Value Callback
// ============================================================================

/**
 * @brief Callback invoked when a PID value is received
 * @param pid PID number
 * @param value Decoded value
 * @param unit Unit enum
 */
typedef void (*poll_value_callback_t)(uint16_t pid, float value, uint8_t unit);

// ============================================================================
// API Functions
// ============================================================================

/**
 * @brief Initialize poll engine
 * @return ESP_OK on success
 */
esp_err_t poll_engine_init(void);

/**
 * @brief Initialize with custom configuration
 */
esp_err_t poll_engine_init_with_config(const poll_engine_config_t *config);

/**
 * @brief Start polling
 */
esp_err_t poll_engine_start(void);

/**
 * @brief Stop polling
 */
esp_err_t poll_engine_stop(void);

/**
 * @brief Add a PID to the poll list
 * @param mode OBD mode (0x01 or 0x22)
 * @param pid PID number
 * @param priority Initial priority (1-5), or 0 for default
 * @return ESP_OK on success
 */
esp_err_t poll_engine_add_pid(uint8_t mode, uint16_t pid, uint8_t priority);

/**
 * @brief Remove a PID from the poll list
 */
esp_err_t poll_engine_remove_pid(uint16_t pid);

/**
 * @brief Enable/disable polling for a specific PID
 */
esp_err_t poll_engine_set_pid_enabled(uint16_t pid, bool enabled);

/**
 * @brief Set priority for a PID (persists to NVS if enabled)
 */
esp_err_t poll_engine_set_pid_priority(uint16_t pid, uint8_t priority);

/**
 * @brief Get current priority for a PID
 */
esp_err_t poll_engine_get_pid_priority(uint16_t pid, uint8_t *priority);

/**
 * @brief Get the latest value for a PID
 * @param pid PID number
 * @param value Output: decoded value
 * @param changed Output: true if value changed since last read
 * @return ESP_OK if value available
 */
esp_err_t poll_engine_get_value(uint16_t pid, float *value, bool *changed);

/**
 * @brief Get poll engine statistics
 */
esp_err_t poll_engine_get_stats(poll_stats_t *stats);

/**
 * @brief Run PID discovery sweep
 * 
 * Queries supported PID bitmaps (0x00, 0x20, 0x40...) and automatically
 * adds supported PIDs to the poll list with default priority.
 * 
 * @param mode OBD mode to discover (0x01 for standard, 0x22 for extended)
 * @return Number of PIDs discovered, or negative error
 */
int poll_engine_discover_pids(uint8_t mode);

/**
 * @brief Clear all poll jobs
 */
esp_err_t poll_engine_clear_all(void);

/**
 * @brief Register callback for PID values
 * @param callback Function to call when values are received
 * @return ESP_OK on success
 */
esp_err_t poll_engine_register_value_callback(poll_value_callback_t callback);

