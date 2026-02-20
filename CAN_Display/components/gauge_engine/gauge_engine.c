/**
 * @file gauge_engine.c
 * @brief Gauge Data Manager Implementation
 *
 * Manages an array of gauge slots. Each slot holds a PID assignment,
 * unit selection, and the latest converted value. The engine aggregates
 * all assigned PIDs into a single poll list, fetches values from
 * comm_link, performs unit conversion, and formats display strings.
 *
 * Thread safety: all public functions are safe to call from any task.
 * The slots array is protected by a mutex. The LVGL timer callback
 * that calls gauge_engine_update() does NOT hold the LVGL lock --
 * it only reads strings that the UI layer copies out.
 */

#include "gauge_engine.h"
#include "comm_link.h"
#include "pid_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "gauge_eng";

// ============================================================================
// Internal State
// ============================================================================

static gauge_slot_t     s_slots[GAUGE_MAX_SLOTS];
static SemaphoreHandle_t s_mutex = NULL;
static bool             s_polling = false;
static uint8_t          s_poll_rate_hz = 10;

#define LOCK()   xSemaphoreTake(s_mutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_mutex)

// ============================================================================
// Helpers
// ============================================================================

/** Resolve unit index to pid_unit_t for a given base unit */
static pid_unit_t resolve_unit(pid_unit_t base, int unit_index)
{
    if (unit_index == 0) return base;

    pid_unit_t alts[4];
    int n = pid_unit_get_alts(base, alts, 4);
    if (unit_index - 1 < n) {
        return alts[unit_index - 1];
    }
    return base;
}

// ============================================================================
// Init
// ============================================================================

esp_err_t gauge_engine_init(void)
{
    if (!s_mutex) {
        s_mutex = xSemaphoreCreateMutex();
        if (!s_mutex) return ESP_ERR_NO_MEM;
    }

    LOCK();
    for (int i = 0; i < GAUGE_MAX_SLOTS; i++) {
        memset(&s_slots[i], 0, sizeof(gauge_slot_t));
        s_slots[i].pid_id = 0xFFFF;
        s_slots[i].base_unit = PID_UNIT_NONE;
        s_slots[i].display_unit = PID_UNIT_NONE;
        strcpy(s_slots[i].value_str, "---");
    }
    s_polling = false;
    UNLOCK();

    ESP_LOGI(TAG, "Gauge engine initialized (%d slots)", GAUGE_MAX_SLOTS);
    return ESP_OK;
}

// ============================================================================
// Slot Configuration
// ============================================================================

esp_err_t gauge_engine_set_pid(int slot, int pid_index)
{
    if (slot < 0 || slot >= GAUGE_MAX_SLOTS) return ESP_ERR_INVALID_ARG;

    uint16_t pid_id = comm_link_get_meta_pid_id(pid_index);
    if (pid_id == 0xFFFF) return ESP_ERR_INVALID_ARG;

    pid_unit_t base = comm_link_get_pid_unit(pid_id);

    LOCK();
    gauge_slot_t *g = &s_slots[slot];
    g->pid_id       = pid_id;
    g->base_unit    = base;
    g->display_unit = base;
    g->value_valid  = false;
    strcpy(g->value_str, "---");
    UNLOCK();

    const char *name = comm_link_get_pid_name(pid_id);
    ESP_LOGI(TAG, "Slot %d -> PID 0x%04X (%s), unit: %s",
             slot, pid_id, name ? name : "?", pid_unit_str(base));

    // If actively polling, rebuild the list to include/drop PIDs
    if (s_polling) {
        gauge_engine_rebuild_poll_list();
    }

    return ESP_OK;
}

esp_err_t gauge_engine_set_unit(int slot, int unit_index)
{
    if (slot < 0 || slot >= GAUGE_MAX_SLOTS) return ESP_ERR_INVALID_ARG;

    LOCK();
    gauge_slot_t *g = &s_slots[slot];
    if (g->pid_id == 0xFFFF) {
        UNLOCK();
        return ESP_ERR_INVALID_STATE;
    }
    g->display_unit = resolve_unit(g->base_unit, unit_index);
    // Re-convert existing value immediately
    if (g->value_valid) {
        if (g->display_unit != g->base_unit) {
            pid_unit_convert(g->raw_value, g->base_unit, g->display_unit,
                             &g->display_value);
        } else {
            g->display_value = g->raw_value;
        }
        snprintf(g->value_str, GAUGE_VALUE_STR_LEN, "%.1f", g->display_value);
    }
    UNLOCK();

    ESP_LOGI(TAG, "Slot %d unit -> %s", slot, pid_unit_str(g->display_unit));
    return ESP_OK;
}

esp_err_t gauge_engine_clear_slot(int slot)
{
    if (slot < 0 || slot >= GAUGE_MAX_SLOTS) return ESP_ERR_INVALID_ARG;

    LOCK();
    gauge_slot_t *g = &s_slots[slot];
    g->pid_id       = 0xFFFF;
    g->base_unit    = PID_UNIT_NONE;
    g->display_unit = PID_UNIT_NONE;
    g->value_valid  = false;
    strcpy(g->value_str, "---");
    UNLOCK();

    if (s_polling) {
        gauge_engine_rebuild_poll_list();
    }

    return ESP_OK;
}

// ============================================================================
// Slot Query
// ============================================================================

const gauge_slot_t *gauge_engine_get_slot(int slot)
{
    if (slot < 0 || slot >= GAUGE_MAX_SLOTS) return NULL;
    return &s_slots[slot];
}

int gauge_engine_get_unit_options(int slot, char *buf, int buf_len)
{
    if (slot < 0 || slot >= GAUGE_MAX_SLOTS || !buf || buf_len < 4) {
        if (buf && buf_len > 0) buf[0] = '\0';
        return 0;
    }

    LOCK();
    pid_unit_t base = s_slots[slot].base_unit;
    uint16_t pid_id = s_slots[slot].pid_id;
    UNLOCK();

    if (pid_id == 0xFFFF) {
        snprintf(buf, buf_len, "---");
        return 0;
    }

    int pos = 0;
    int count = 0;

    // Base unit first
    const char *base_str = pid_unit_str(base);
    pos += snprintf(buf + pos, buf_len - pos, "%s",
                    (base_str && base_str[0]) ? base_str : "raw");
    count++;

    // Alternatives
    pid_unit_t alts[4];
    int n = pid_unit_get_alts(base, alts, 4);
    for (int i = 0; i < n && pos < buf_len - 8; i++) {
        const char *s = pid_unit_str(alts[i]);
        if (s && s[0]) {
            pos += snprintf(buf + pos, buf_len - pos, "\n%s", s);
            count++;
        }
    }

    return count;
}

// ============================================================================
// PID List String
// ============================================================================

int gauge_engine_build_pid_options(char *buf, int buf_len)
{
    if (!buf || buf_len < 8) return 0;

    int meta_count = comm_link_get_pid_meta_count();
    if (meta_count == 0) return 0;

    int pos = 0;
    int count = 0;

    for (int i = 0; i < meta_count && pos < buf_len - 64; i++) {
        uint16_t pid_id = comm_link_get_meta_pid_id(i);
        const char *name = comm_link_get_pid_name(pid_id);
        if (pid_id == 0xFFFF || !name) continue;

        if (count > 0) {
            pos += snprintf(buf + pos, buf_len - pos, "\n");
        }
        pos += snprintf(buf + pos, buf_len - pos, "0x%02X %s", pid_id, name);
        count++;
    }

    return count;
}

// ============================================================================
// Polling Control
// ============================================================================

esp_err_t gauge_engine_start_polling(uint8_t rate_hz)
{
    if (rate_hz == 0 || rate_hz > 50) rate_hz = 10;
    s_poll_rate_hz = rate_hz;

    esp_err_t err = gauge_engine_rebuild_poll_list();
    if (err != ESP_OK) return err;

    s_polling = true;
    ESP_LOGI(TAG, "Polling started @ %d Hz", rate_hz);
    return ESP_OK;
}

esp_err_t gauge_engine_stop_polling(void)
{
    s_polling = false;
    comm_link_clear_poll_list();

    // Reset all value displays
    LOCK();
    for (int i = 0; i < GAUGE_MAX_SLOTS; i++) {
        s_slots[i].value_valid = false;
        strcpy(s_slots[i].value_str, "---");
    }
    UNLOCK();

    ESP_LOGI(TAG, "Polling stopped");
    return ESP_OK;
}

bool gauge_engine_is_polling(void)
{
    return s_polling;
}

esp_err_t gauge_engine_rebuild_poll_list(void)
{
    uint16_t pids[GAUGE_MAX_SLOTS];
    int count = 0;

    LOCK();
    for (int i = 0; i < GAUGE_MAX_SLOTS; i++) {
        if (s_slots[i].pid_id == 0xFFFF) continue;

        // Deduplicate (multiple gauges may watch the same PID)
        bool dup = false;
        for (int j = 0; j < count; j++) {
            if (pids[j] == s_slots[i].pid_id) { dup = true; break; }
        }
        if (!dup && count < GAUGE_MAX_SLOTS) {
            pids[count++] = s_slots[i].pid_id;
        }
    }
    UNLOCK();

    if (count == 0) {
        ESP_LOGW(TAG, "No PIDs assigned to any gauge slot");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = comm_link_set_poll_list(pids, count, s_poll_rate_hz);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Poll list rebuilt: %d unique PIDs @ %d Hz",
                 count, s_poll_rate_hz);
    }
    return err;
}

int gauge_engine_get_active_pid_count(void)
{
    uint16_t seen[GAUGE_MAX_SLOTS];
    int count = 0;

    LOCK();
    for (int i = 0; i < GAUGE_MAX_SLOTS; i++) {
        if (s_slots[i].pid_id == 0xFFFF) continue;
        bool dup = false;
        for (int j = 0; j < count; j++) {
            if (seen[j] == s_slots[i].pid_id) { dup = true; break; }
        }
        if (!dup) seen[count++] = s_slots[i].pid_id;
    }
    UNLOCK();

    return count;
}

// ============================================================================
// Periodic Update
// ============================================================================

int gauge_engine_update(void)
{
    int updated = 0;

    LOCK();
    for (int i = 0; i < GAUGE_MAX_SLOTS; i++) {
        gauge_slot_t *g = &s_slots[i];
        if (g->pid_id == 0xFFFF) continue;

        pid_cache_entry_t val;
        if (!comm_link_get_pid(g->pid_id, &val)) continue;

        g->raw_value = val.value;

        if (g->display_unit != g->base_unit) {
            pid_unit_convert(val.value, g->base_unit, g->display_unit,
                             &g->display_value);
        } else {
            g->display_value = val.value;
        }

        snprintf(g->value_str, GAUGE_VALUE_STR_LEN, "%.1f", g->display_value);
        g->value_valid = true;
        g->last_update_tick = xTaskGetTickCount();
        updated++;
    }
    UNLOCK();

    return updated;
}

