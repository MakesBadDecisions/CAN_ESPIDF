/**
 * @file pid_store.c
 * @brief Selected PID list — storage and persistence
 *
 * Saves the user's PID selection to /sdcard/pids.dat (binary format):
 *   [uint8_t  version]    = 1
 *   [uint8_t  rate_hz]    = poll rate
 *   [uint16_t count]      = number of PIDs
 *   [uint16_t pids[count]]
 *
 * Binary rather than JSON for simplicity — 200 bytes max for 96 PIDs.
 * The file is tiny and written infrequently (only on user "Apply").
 */

#include "pid_store.h"
#include "esp_log.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "pid_store";

#define PID_STORE_FILE  "/sdcard/pids.dat"
#define PID_STORE_VER   1

// ============================================================================
// Internal State
// ============================================================================

static uint16_t s_pids[PID_STORE_MAX];
static int      s_count   = 0;
static uint8_t  s_rate_hz = 10;

// ============================================================================
// Persistence
// ============================================================================

int pid_store_load(void)
{
    FILE *f = fopen(PID_STORE_FILE, "rb");
    if (!f) {
        ESP_LOGI(TAG, "No saved PID selection (%s)", PID_STORE_FILE);
        return 0;
    }

    uint8_t ver = 0;
    if (fread(&ver, 1, 1, f) != 1 || ver != PID_STORE_VER) {
        ESP_LOGW(TAG, "PID store version mismatch (got %u, expected %u)", ver, PID_STORE_VER);
        fclose(f);
        return 0;
    }

    uint8_t rate = 10;
    fread(&rate, 1, 1, f);
    if (rate >= 1 && rate <= 50) s_rate_hz = rate;

    uint16_t count = 0;
    if (fread(&count, sizeof(uint16_t), 1, f) != 1 || count > PID_STORE_MAX) {
        ESP_LOGW(TAG, "PID store invalid count: %u", count);
        fclose(f);
        return 0;
    }

    size_t read = fread(s_pids, sizeof(uint16_t), count, f);
    s_count = (int)read;
    fclose(f);

    ESP_LOGI(TAG, "Loaded %d selected PIDs @ %d Hz from SD", s_count, s_rate_hz);
    return s_count;
}

esp_err_t pid_store_save(void)
{
    FILE *f = fopen(PID_STORE_FILE, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s for writing", PID_STORE_FILE);
        return ESP_FAIL;
    }

    uint8_t ver = PID_STORE_VER;
    fwrite(&ver, 1, 1, f);
    fwrite(&s_rate_hz, 1, 1, f);

    uint16_t count = (uint16_t)s_count;
    fwrite(&count, sizeof(uint16_t), 1, f);
    fwrite(s_pids, sizeof(uint16_t), s_count, f);

    fclose(f);
    ESP_LOGI(TAG, "Saved %d selected PIDs @ %d Hz to SD", s_count, s_rate_hz);
    return ESP_OK;
}

// ============================================================================
// Selection Management
// ============================================================================

void pid_store_set(const uint16_t *pids, int count)
{
    if (count > PID_STORE_MAX) count = PID_STORE_MAX;
    if (count > 0 && pids) {
        memcpy(s_pids, pids, count * sizeof(uint16_t));
    }
    s_count = count;
}

void pid_store_clear(void)
{
    s_count = 0;
}

// ============================================================================
// Queries
// ============================================================================

bool pid_store_is_selected(uint16_t pid_id)
{
    for (int i = 0; i < s_count; i++) {
        if (s_pids[i] == pid_id) return true;
    }
    return false;
}

bool pid_store_has_selection(void)
{
    return s_count > 0;
}

int pid_store_get_count(void)
{
    return s_count;
}

int pid_store_get_selected(uint16_t *out_pids, int max_count)
{
    int n = (s_count < max_count) ? s_count : max_count;
    if (n > 0) memcpy(out_pids, s_pids, n * sizeof(uint16_t));
    return n;
}

uint8_t pid_store_get_rate_hz(void)
{
    return s_rate_hz;
}

void pid_store_set_rate_hz(uint8_t rate_hz)
{
    if (rate_hz >= 1 && rate_hz <= 50) {
        s_rate_hz = rate_hz;
    }
}
