/**
 * @file pid_store.h
 * @brief Selected PID list manager — persistence and query
 *
 * The "selected PIDs" are the user's curated list of PIDs they want to
 * poll, log, and display.  This list is saved to SD card so it survives
 * reboots.  It replaces the old approach where gauge slot assignments
 * drove everything.
 *
 * Data flow:
 *   scan → all supported PIDs discovered (comm_link metadata)
 *   user checks PIDs in web portal → pid_store_set() + pid_store_save()
 *   gauge dropdowns filter to selected PIDs
 *   logger columns = selected PIDs
 *   poll sends full selected list to CAN Interface
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define PID_STORE_MAX   96      // Max PIDs in selection (matches metadata store)

// ============================================================================
// Lifecycle
// ============================================================================

/**
 * @brief Load saved PID selection from SD card
 *
 * Call after logger_init() (SD must be mounted).
 * If no saved file exists, selection is empty.
 *
 * @return Number of PIDs loaded, or 0 if no saved selection
 */
int pid_store_load(void);

/**
 * @brief Save current PID selection to SD card
 * @return ESP_OK on success
 */
esp_err_t pid_store_save(void);

// ============================================================================
// Selection Management
// ============================================================================

/**
 * @brief Set the selected PID list (replaces entire selection)
 * @param pids  Array of PID IDs
 * @param count Number of PIDs (max PID_STORE_MAX)
 */
void pid_store_set(const uint16_t *pids, int count);

/**
 * @brief Clear entire selection
 */
void pid_store_clear(void);

// ============================================================================
// Queries
// ============================================================================

/**
 * @brief Check if a PID is in the selected list
 * @param pid_id PID to check
 * @return true if selected
 */
bool pid_store_is_selected(uint16_t pid_id);

/**
 * @brief Check if any PIDs are selected
 */
bool pid_store_has_selection(void);

/**
 * @brief Get count of selected PIDs
 */
int pid_store_get_count(void);

/**
 * @brief Copy selected PID list into caller's buffer
 * @param out_pids Output array
 * @param max_count Size of output array
 * @return Number of PIDs copied
 */
int pid_store_get_selected(uint16_t *out_pids, int max_count);

/**
 * @brief Get the stored poll rate in Hz
 */
uint8_t pid_store_get_rate_hz(void);

/**
 * @brief Set the poll rate
 */
void pid_store_set_rate_hz(uint8_t rate_hz);
