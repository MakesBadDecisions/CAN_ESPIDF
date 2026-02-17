/**
 * @file sys_mgr.h
 * @brief System Manager - Runtime Coordination & Health Monitoring
 *
 * Manages task handoffs, data direction, NVS changes, and supervises all
 * FreeRTOS tasks. Provides state machine for system lifecycle.
 *
 * State Machine:
 *   INIT → CONNECTING_CAN → CONNECTING_COMM → RUNNING ↔ ERROR
 *
 * Responsibilities:
 * - Supervise registered tasks (stack watermarks, responsiveness)
 * - Feed task watchdog
 * - Monitor heap and error counters
 * - Coordinate bus-off recovery
 * - Signal state transitions to tasks
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================================
// System States
// ============================================================================

typedef enum {
    SYS_STATE_INIT,             // Components initializing
    SYS_STATE_CONNECTING_CAN,   // Waiting for CAN bus / ECU discovery
    SYS_STATE_CONNECTING_COMM,  // Waiting for comm link to Display Node
    SYS_STATE_RUNNING,          // Normal operation
    SYS_STATE_ERROR,            // Recoverable error state
    SYS_STATE_FATAL,            // Unrecoverable, requires reset
} sys_state_t;

// ============================================================================
// Error Codes (for sys_mgr_report_error)
// ============================================================================

typedef enum {
    SYS_ERR_NONE = 0,
    SYS_ERR_CAN_BUS_OFF,        // CAN controller bus-off
    SYS_ERR_CAN_TX_FAIL,        // CAN TX queue overflow / timeout
    SYS_ERR_CAN_NO_RESPONSE,    // No ECU response during discovery
    SYS_ERR_COMM_LINK_DOWN,     // UART link to display lost
    SYS_ERR_HEAP_LOW,           // Free heap below threshold
    SYS_ERR_TASK_STUCK,         // Task watchdog timeout
    SYS_ERR_NVS_FAIL,           // NVS read/write error
} sys_error_t;

// ============================================================================
// Task Registration
// ============================================================================

#define SYS_MGR_MAX_TASKS 8

typedef struct {
    const char     *name;           // Task name for logging
    TaskHandle_t    handle;         // FreeRTOS task handle
    uint32_t        wdt_timeout_ms; // Watchdog timeout (0 = no WDT)
    uint32_t        last_checkin;   // Last sys_mgr_task_checkin() timestamp
} sys_task_info_t;

// ============================================================================
// Statistics
// ============================================================================

typedef struct {
    uint32_t uptime_sec;
    uint32_t can_frames_rx;
    uint32_t can_frames_tx;
    uint32_t obd_responses;
    uint32_t comm_frames_tx;
    uint32_t error_count;
    uint32_t state_changes;
    uint32_t heap_min_free;
} sys_stats_t;

// ============================================================================
// API
// ============================================================================

/**
 * @brief Initialize system manager (call before registering tasks)
 * @return ESP_OK on success
 */
esp_err_t sys_mgr_init(void);

/**
 * @brief Start system manager monitor task and transition to CONNECTING_CAN
 * Call this after all component inits and task creates are done.
 * @return ESP_OK on success
 */
esp_err_t sys_mgr_start(void);

/**
 * @brief Get current system state
 * @return Current sys_state_t
 */
sys_state_t sys_mgr_get_state(void);

/**
 * @brief Get state name string (for logging)
 * @param state State to convert
 * @return Static string name
 */
const char* sys_mgr_state_name(sys_state_t state);

/**
 * @brief Register a task for supervision
 * @param name Task name
 * @param handle FreeRTOS task handle
 * @param wdt_timeout_ms Watchdog timeout (0 to disable)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if table full
 */
esp_err_t sys_mgr_register_task(const char *name, TaskHandle_t handle, uint32_t wdt_timeout_ms);

/**
 * @brief Task check-in (call periodically from supervised tasks)
 * Resets the task's watchdog timer.
 * @param handle Task handle (use NULL for calling task)
 */
void sys_mgr_task_checkin(TaskHandle_t handle);

/**
 * @brief Report an error to sys_mgr
 * @param error Error code
 * @param details Optional detail string (can be NULL)
 */
void sys_mgr_report_error(sys_error_t error, const char *details);

/**
 * @brief Request state transition (sys_mgr validates transition)
 * @param new_state Requested state
 * @return ESP_OK if transition valid, ESP_ERR_INVALID_STATE otherwise
 */
esp_err_t sys_mgr_request_state(sys_state_t new_state);

/**
 * @brief Request bus-off recovery (triggers CAN driver reinit)
 */
void sys_mgr_request_can_recovery(void);

/**
 * @brief Get current statistics
 * @param stats Output struct
 */
void sys_mgr_get_stats(sys_stats_t *stats);

/**
 * @brief Increment a stat counter (called by other components)
 * @param stat_offset Offset into sys_stats_t (use offsetof)
 * @param value Amount to add
 */
void sys_mgr_stat_add(size_t stat_offset, uint32_t value);

// Convenience macros for stat updates
#define SYS_STAT_CAN_RX(n)    sys_mgr_stat_add(offsetof(sys_stats_t, can_frames_rx), (n))
#define SYS_STAT_CAN_TX(n)    sys_mgr_stat_add(offsetof(sys_stats_t, can_frames_tx), (n))
#define SYS_STAT_OBD_RESP(n)  sys_mgr_stat_add(offsetof(sys_stats_t, obd_responses), (n))
#define SYS_STAT_COMM_TX(n)   sys_mgr_stat_add(offsetof(sys_stats_t, comm_frames_tx), (n))

