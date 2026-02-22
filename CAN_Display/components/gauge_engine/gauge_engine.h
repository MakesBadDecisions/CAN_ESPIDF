/**
 * @file gauge_engine.h
 * @brief Gauge Data Manager
 *
 * Owns per-gauge state (PID assignment, unit selection, converted values).
 * Aggregates active gauges into a single poll list sent to the CAN Interface.
 * No LVGL dependency -- the UI layer reads formatted values from here.
 *
 * Designed for up to GAUGE_MAX_SLOTS gauges spread across multiple screens.
 */

#pragma once

#include "esp_err.h"
#include "pid_types.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Limits
// ============================================================================

/** Maximum number of gauge slots across all screens */
#define GAUGE_MAX_SLOTS     20

/** Maximum formatted value string length */
#define GAUGE_VALUE_STR_LEN 16

/** Maximum unit options string length (for dropdown) */
#define GAUGE_UNIT_OPTS_LEN 128

// ============================================================================
// Virtual PIDs (local sensors, not from CAN bus)
// ============================================================================

/** Virtual PID base — IDs 0xFF00+ are local sensor channels, not CAN PIDs */
#define VPID_BASE           0xFF00

/** IMU virtual PID — 6-axis accelerometer + gyroscope */
#define VPID_IMU            0xFF00

/** Check if a PID ID is a virtual (non-CAN) channel */
#define GAUGE_IS_VIRTUAL(pid_id) ((pid_id) >= VPID_BASE && (pid_id) != 0xFFFF)

/** IMU display mode — selected via unit dropdown */
typedef enum {
    IMU_MODE_G_LOAD = 0,    /**< G-force bubble (lateral/longitudinal accel) */
    IMU_MODE_TILT   = 1,    /**< Tilt inclinometer (pitch/roll angles) */
} imu_display_mode_t;

// ============================================================================
// Alert Thresholds
// ============================================================================

/** Alert severity level, evaluated per-tick in gauge_engine_update() */
typedef enum {
    ALERT_NONE     = 0,   /**< Normal — value within safe range */
    ALERT_WARN     = 1,   /**< Warning — yellow/amber border (blends toward red) */
    ALERT_CRITICAL = 2,   /**< Critical — flashing red border + red text */
    ALERT_MAX      = 3,   /**< MAX exceeded — flashing red + buzzer */
} alert_level_t;

/** Maximum number of PIDs with alert thresholds */
#define ALERT_MAX_ENTRIES  32

/**
 * Per-PID alert threshold config (stored in NVS).
 *
 * Three numbers define the alert zones:
 *   value < warn            → NORMAL
 *   warn ≤ value < crit     → WARNING  (yellow, blending to red)
 *   crit ≤ value < max      → CRITICAL (flashing red + red text)
 *   value ≥ max             → MAX      (flashing red + red text + buzzer)
 *
 * Threshold direction is implicit:
 *   warn < crit  → alert on HIGH values (temp, RPM, boost)
 *   warn > crit  → alert on LOW values  (oil pressure, voltage)
 */
typedef struct __attribute__((packed)) {
    uint16_t pid_id;        /**< PID ID (0xFFFF = unused entry) */
    uint16_t _pad;          /**< Alignment padding */
    float    warn;          /**< Start of warning zone (base unit) */
    float    crit;          /**< Start of critical zone (base unit) */
    float    max;           /**< Critical maximum (base unit) */
} pid_alert_config_t;       /* 16 bytes total */

// ============================================================================
// Gauge Slot State
// ============================================================================

typedef struct {
    // --- Configuration (set by UI) ---
    uint16_t    pid_id;             // Selected PID (0xFFFF = none)
    pid_unit_t  base_unit;          // PID's native unit
    pid_unit_t  display_unit;       // Currently selected display unit

    // --- Live data (updated by gauge_engine_update) ---
    float       raw_value;          // Last value in base units
    float       display_value;      // Converted to display_unit
    char        value_str[GAUGE_VALUE_STR_LEN]; // Formatted string "123.4"
    bool        value_valid;        // Got at least one reading
    bool        stale;              // PID data not updated within timeout
    uint32_t    last_update_tick;   // tick of last update

    // --- Alert state (evaluated by gauge_engine_update) ---
    alert_level_t alert_level;      // Current alert severity
    float         alert_progress;   // 0-1 position within warn zone (for color blend)
} gauge_slot_t;

// ============================================================================
// Init / Teardown
// ============================================================================

/**
 * @brief Initialize gauge engine (zeroes all slots)
 */
esp_err_t gauge_engine_init(void);

// ============================================================================
// Slot Configuration (called from UI on dropdown changes)
// ============================================================================

/**
 * @brief Assign a PID to a gauge slot
 *
 * Looks up the PID's base unit from comm_link metadata, populates
 * the slot, and rebuilds the poll list if polling is active.
 *
 * @param slot      Gauge index (0 .. GAUGE_MAX_SLOTS-1)
 * @param pid_index Index into comm_link metadata (dropdown selection index)
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if slot/index out of range
 */
esp_err_t gauge_engine_set_pid(int slot, int pid_index);

/**
 * @brief Change the display unit for a gauge slot
 *
 * @param slot       Gauge index
 * @param unit_index Index into the unit options list (0 = base unit)
 * @return ESP_OK, or ESP_ERR_INVALID_ARG
 */
esp_err_t gauge_engine_set_unit(int slot, int unit_index);

/**
 * @brief Clear a gauge slot (un-assign PID)
 */
esp_err_t gauge_engine_clear_slot(int slot);

// ============================================================================
// Slot Query (called from UI to read current state)
// ============================================================================

/**
 * @brief Get read-only pointer to a gauge slot
 * @return NULL if slot index out of range
 */
const gauge_slot_t *gauge_engine_get_slot(int slot);

/**
 * @brief Build unit dropdown options string for a slot's current PID
 *
 * Returns a newline-separated string like "C\nF" suitable for
 * lv_dropdown_set_options(). The first entry is always the base unit.
 *
 * @param slot    Gauge index
 * @param buf     Output buffer
 * @param buf_len Buffer size
 * @return Number of options (0 if slot has no PID)
 */
int gauge_engine_get_unit_options(int slot, char *buf, int buf_len);

// ============================================================================
// PID List String (shared dropdown content, built once after scan)
// ============================================================================

/**
 * @brief Build PID dropdown options string from comm_link metadata
 *
 * Returns a newline-separated string like "0x0C RPM\n0x0D Speed\n..."
 * suitable for lv_dropdown_set_options(). Same for all gauge slots.
 *
 * @param buf     Output buffer (recommend 2048+)
 * @param buf_len Buffer size
 * @return Number of PID entries, or 0 if no metadata available
 */
int gauge_engine_build_pid_options(char *buf, int buf_len);

/**
 * @brief Notify that PID dropdown options have changed (new poll list)
 *
 * Sets interior flag so the UI can repopulate dropdowns on next tick.
 */
void gauge_engine_notify_pid_options_changed(void);

/**
 * @brief Check and clear the "PID options dirty" flag
 *
 * Called from the gauge timer (LVGL task context) to trigger
 * dropdown repopulation when the poll list changes.
 *
 * @return true if options changed since last check
 */
bool gauge_engine_pid_options_dirty(void);

// ============================================================================
// Polling Control
// ============================================================================

/**
 * @brief Start polling -- aggregate all assigned gauge PIDs into one poll list
 * @param rate_hz  Polling rate (1-50 Hz)
 * @return ESP_OK, or ESP_ERR_INVALID_STATE if no PIDs assigned
 */
esp_err_t gauge_engine_start_polling(uint8_t rate_hz);

/**
 * @brief Stop polling and clear poll list
 */
esp_err_t gauge_engine_stop_polling(void);

/**
 * @brief Check if polling is active
 */
bool gauge_engine_is_polling(void);

/**
 * @brief Check if new PID data has arrived since last update
 *
 * Set by the comm_link PID callback when data arrives.
 * Cleared by gauge_engine_update(). Useful for the UI timer to
 * skip redundant update cycles.
 *
 * @return true if new data is available
 */
bool gauge_engine_has_new_data(void);

/**
 * @brief Update all active gauge slots with latest values from comm_link
 *
 * Call this periodically (e.g. from an LVGL timer at 10 Hz).  For each
 * slot with an assigned PID, reads the cached value, applies unit
 * conversion, and formats the display string.
 *
 * @return Number of slots that received fresh data
 */
int gauge_engine_update(void);

/**
 * @brief Rebuild and resend the poll list
 *
 * Called internally when a PID assignment changes during active polling.
 * Can also be called externally to force a refresh.
 */
esp_err_t gauge_engine_rebuild_poll_list(void);

/**
 * @brief Get the number of unique PIDs currently being polled
 */
int gauge_engine_get_active_pid_count(void);

// ============================================================================
// NVS Persistence
// ============================================================================

/**
 * @brief Save current gauge slot config (PID + unit) to NVS
 *
 * Called automatically on set_pid, set_unit, clear_slot.
 * Can also be called explicitly.
 */
esp_err_t gauge_engine_save_config(void);

/**
 * @brief Load saved gauge config from NVS and restore slot assignments
 *
 * Must be called AFTER scan completes (needs comm_link metadata to map
 * PID IDs back to dropdown indices). Returns the number of slots restored.
 *
 * @return Number of slots successfully restored, or -1 on error
 */
int gauge_engine_load_config(void);

// ============================================================================
// Alert Threshold API
// ============================================================================

/**
 * @brief Set alert thresholds for a PID (in base/native units)
 *
 * If warn < crit: alerts trigger when value goes HIGH (temp, RPM, boost).
 * If warn > crit: alerts trigger when value goes LOW (oil pressure, voltage).
 * Set all three to 0 to disable alerts for this PID.
 */
esp_err_t gauge_engine_set_alert(uint16_t pid_id, float warn, float crit, float max);

/**
 * @brief Get alert thresholds for a PID
 * @return true if thresholds are configured for this PID
 */
bool gauge_engine_get_alert(uint16_t pid_id, float *warn, float *crit, float *max);

/**
 * @brief Clear alert thresholds for a PID
 */
esp_err_t gauge_engine_clear_alert(uint16_t pid_id);

/**
 * @brief Save all alert configs to NVS
 */
esp_err_t gauge_engine_save_alerts(void);

/**
 * @brief Load alert configs from NVS (called automatically during init)
 * @return Number of PIDs with alerts loaded
 */
int gauge_engine_load_alerts(void);

