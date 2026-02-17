/**
 * @file poll_engine.c
 * @brief Smart Polling Scheduler Implementation
 */

#include "poll_engine.h"
#include "obd2.h"
#include "pid_db.h"
#include "system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "poll_engine";

// ============================================================================
// Priority Interval Mapping (ms)
// ============================================================================

static const uint32_t PRIORITY_INTERVALS[6] = {
    0,      // Unused (priority 0)
    5000,   // Priority 1: Slow (5 seconds)
    1000,   // Priority 2: Low (1 second)
    200,    // Priority 3: Normal (200ms)
    100,    // Priority 4: High (100ms)
    100,    // Priority 5: Realtime (100ms, 10Hz)
};

// ============================================================================
// Internal State
// ============================================================================

static struct {
    poll_engine_config_t    config;
    poll_job_t              jobs[POLL_ENGINE_MAX_JOBS];
    uint8_t                 job_count;
    TaskHandle_t            task_handle;
    SemaphoreHandle_t       mutex;
    bool                    running;
    bool                    initialized;
    poll_stats_t            stats;
    nvs_handle_t            nvs_handle;
    poll_value_callback_t   value_callback;
} s_poll;

// ============================================================================
// NVS Helpers
// ============================================================================

static void load_pid_priority_from_nvs(uint16_t pid, uint8_t *priority)
{
    if (!s_poll.config.persist_priorities || !s_poll.nvs_handle) {
        return;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "p%04X", pid);
    
    uint8_t stored;
    if (nvs_get_u8(s_poll.nvs_handle, key, &stored) == ESP_OK) {
        if (stored >= POLL_PRIORITY_MIN && stored <= POLL_PRIORITY_MAX) {
            *priority = stored;
        }
    }
}

static void save_pid_priority_to_nvs(uint16_t pid, uint8_t priority)
{
    if (!s_poll.config.persist_priorities || !s_poll.nvs_handle) {
        return;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "p%04X", pid);
    
    nvs_set_u8(s_poll.nvs_handle, key, priority);
    nvs_commit(s_poll.nvs_handle);
}

// ============================================================================
// Job Management
// ============================================================================

static poll_job_t *find_job_by_pid(uint16_t pid)
{
    for (int i = 0; i < s_poll.job_count; i++) {
        if (s_poll.jobs[i].pid == pid) {
            return &s_poll.jobs[i];
        }
    }
    return NULL;
}

static void update_job_interval(poll_job_t *job)
{
    if (job->priority >= POLL_PRIORITY_MIN && job->priority <= POLL_PRIORITY_MAX) {
        job->interval_ms = PRIORITY_INTERVALS[job->priority];
    } else {
        job->interval_ms = PRIORITY_INTERVALS[POLL_PRIORITY_DEFAULT];
    }
}

// ============================================================================
// Poll Task
// ============================================================================

static void poll_engine_task(void *arg)
{
    SYS_LOGI(TAG, "Poll task started");
    
    uint32_t poll_count_last_sec = 0;
    TickType_t last_stats_tick = xTaskGetTickCount();
    
    while (s_poll.running) {
        TickType_t now = xTaskGetTickCount();
        
        // Update polls/second stat every second
        if ((now - last_stats_tick) >= pdMS_TO_TICKS(1000)) {
            s_poll.stats.polls_per_second = poll_count_last_sec;
            poll_count_last_sec = 0;
            last_stats_tick = now;
        }
        
        // Find next job due
        poll_job_t *next_job = NULL;
        TickType_t min_wait = pdMS_TO_TICKS(100);  // Max sleep
        
        xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
        
        for (int i = 0; i < s_poll.job_count; i++) {
            poll_job_t *job = &s_poll.jobs[i];
            
            if (!job->enabled) {
                continue;
            }
            
            if (now >= job->next_poll_tick) {
                // This job is due now
                if (!next_job || job->priority > next_job->priority) {
                    next_job = job;
                }
            } else {
                // Calculate wait time
                TickType_t wait = job->next_poll_tick - now;
                if (wait < min_wait) {
                    min_wait = wait;
                }
            }
        }
        
        if (next_job) {
            // Execute poll
            obd2_response_t resp;
            esp_err_t err = obd2_request_pid(next_job->mode, next_job->pid, &resp);
            
            if (err == ESP_OK && resp.status == OBD2_STATUS_OK) {
                // Decode value
                const pid_entry_t *entry = pid_db_lookup(next_job->pid);
                if (entry) {
                    float new_value = pid_db_decode(entry, resp.data, resp.data_len);
                    
                    if (new_value != next_job->last_value) {
                        next_job->value_changed = true;
                        next_job->last_value = new_value;
                    }
                    
                    // Invoke callback with new value
                    if (s_poll.value_callback) {
                        s_poll.value_callback(next_job->pid, new_value, entry->unit);
                    }
                }
                
                next_job->success_count++;
                next_job->last_poll_tick = now;
                s_poll.stats.total_polls++;
                poll_count_last_sec++;
                
                // Adaptive: speed up on success
                if (s_poll.config.adaptive_interval && next_job->priority < POLL_PRIORITY_REALTIME) {
                    // Could adjust interval here based on change rate
                }
            } else {
                next_job->fail_count++;
                s_poll.stats.total_failures++;
                
                // Adaptive: slow down on failure
                if (s_poll.config.adaptive_interval) {
                    // Temporarily increase interval on failure
                    next_job->interval_ms = next_job->interval_ms * 3 / 2;
                    if (next_job->interval_ms > 10000) {
                        next_job->interval_ms = 10000;  // Cap at 10s
                    }
                }
            }
            
            // Schedule next poll
            next_job->next_poll_tick = now + pdMS_TO_TICKS(next_job->interval_ms);
        }
        
        xSemaphoreGive(s_poll.mutex);
        
        // Wait for next job or timeout
        vTaskDelay(next_job ? 1 : min_wait);
    }
    
    SYS_LOGI(TAG, "Poll task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t poll_engine_init(void)
{
    poll_engine_config_t config = POLL_ENGINE_CONFIG_DEFAULT();
    return poll_engine_init_with_config(&config);
}

esp_err_t poll_engine_init_with_config(const poll_engine_config_t *config)
{
    if (s_poll.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(&s_poll, 0, sizeof(s_poll));
    
    if (config) {
        s_poll.config = *config;
    } else {
        s_poll.config = (poll_engine_config_t)POLL_ENGINE_CONFIG_DEFAULT();
    }
    
    // Create mutex
    s_poll.mutex = xSemaphoreCreateMutex();
    if (!s_poll.mutex) {
        SYS_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Open NVS for priority storage
    if (s_poll.config.persist_priorities) {
        if (nvs_open("poll_pri", NVS_READWRITE, &s_poll.nvs_handle) != ESP_OK) {
            SYS_LOGW(TAG, "Failed to open NVS, priorities won't persist");
            s_poll.nvs_handle = 0;
        }
    }
    
    s_poll.initialized = true;
    SYS_LOGI(TAG, "Poll engine initialized (max_outstanding=%d, adaptive=%d)",
             s_poll.config.max_outstanding, s_poll.config.adaptive_interval);
    
    return ESP_OK;
}

esp_err_t poll_engine_start(void)
{
    if (!s_poll.initialized || s_poll.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_poll.running = true;
    
    BaseType_t ret = xTaskCreate(poll_engine_task, "poll_engine",
                                  4096, NULL, 5, &s_poll.task_handle);
    if (ret != pdPASS) {
        s_poll.running = false;
        SYS_LOGE(TAG, "Failed to create poll task");
        return ESP_ERR_NO_MEM;
    }
    
    SYS_LOGI(TAG, "Poll engine started with %d jobs", s_poll.job_count);
    return ESP_OK;
}

esp_err_t poll_engine_stop(void)
{
    if (!s_poll.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_poll.running = false;
    
    // Wait for task to stop
    vTaskDelay(pdMS_TO_TICKS(200));
    
    SYS_LOGI(TAG, "Poll engine stopped");
    return ESP_OK;
}

esp_err_t poll_engine_add_pid(uint8_t mode, uint16_t pid, uint8_t priority)
{
    if (!s_poll.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    
    // Check if already exists
    poll_job_t *existing = find_job_by_pid(pid);
    if (existing) {
        xSemaphoreGive(s_poll.mutex);
        return ESP_OK;  // Already added
    }
    
    if (s_poll.job_count >= POLL_ENGINE_MAX_JOBS) {
        xSemaphoreGive(s_poll.mutex);
        SYS_LOGW(TAG, "Max poll jobs reached (%d)", POLL_ENGINE_MAX_JOBS);
        return ESP_ERR_NO_MEM;
    }
    
    // Apply priority (from parameter, NVS, or default)
    uint8_t final_priority = (priority >= POLL_PRIORITY_MIN && priority <= POLL_PRIORITY_MAX) 
                              ? priority : POLL_PRIORITY_DEFAULT;
    load_pid_priority_from_nvs(pid, &final_priority);
    
    // Add new job
    poll_job_t *job = &s_poll.jobs[s_poll.job_count++];
    memset(job, 0, sizeof(poll_job_t));
    job->pid = pid;
    job->mode = mode;
    job->priority = final_priority;
    job->enabled = true;
    job->next_poll_tick = xTaskGetTickCount();  // Poll immediately
    update_job_interval(job);
    
    s_poll.stats.active_jobs = s_poll.job_count;
    
    xSemaphoreGive(s_poll.mutex);
    
    SYS_LOGD(TAG, "Added PID 0x%04X (mode 0x%02X, priority %d, interval %lu ms)",
             pid, mode, final_priority, job->interval_ms);
    
    return ESP_OK;
}

esp_err_t poll_engine_remove_pid(uint16_t pid)
{
    if (!s_poll.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    
    for (int i = 0; i < s_poll.job_count; i++) {
        if (s_poll.jobs[i].pid == pid) {
            // Shift remaining jobs down
            for (int j = i; j < s_poll.job_count - 1; j++) {
                s_poll.jobs[j] = s_poll.jobs[j + 1];
            }
            s_poll.job_count--;
            s_poll.stats.active_jobs = s_poll.job_count;
            
            xSemaphoreGive(s_poll.mutex);
            SYS_LOGD(TAG, "Removed PID 0x%04X", pid);
            return ESP_OK;
        }
    }
    
    xSemaphoreGive(s_poll.mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t poll_engine_set_pid_enabled(uint16_t pid, bool enabled)
{
    if (!s_poll.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    
    poll_job_t *job = find_job_by_pid(pid);
    if (job) {
        job->enabled = enabled;
        if (enabled) {
            job->next_poll_tick = xTaskGetTickCount();  // Poll immediately
        }
        xSemaphoreGive(s_poll.mutex);
        return ESP_OK;
    }
    
    xSemaphoreGive(s_poll.mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t poll_engine_set_pid_priority(uint16_t pid, uint8_t priority)
{
    if (!s_poll.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (priority < POLL_PRIORITY_MIN || priority > POLL_PRIORITY_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    
    poll_job_t *job = find_job_by_pid(pid);
    if (job) {
        job->priority = priority;
        update_job_interval(job);
        save_pid_priority_to_nvs(pid, priority);
        
        xSemaphoreGive(s_poll.mutex);
        SYS_LOGD(TAG, "Set PID 0x%04X priority to %d (interval %lu ms)",
                 pid, priority, job->interval_ms);
        return ESP_OK;
    }
    
    xSemaphoreGive(s_poll.mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t poll_engine_get_pid_priority(uint16_t pid, uint8_t *priority)
{
    if (!s_poll.initialized || !priority) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    
    poll_job_t *job = find_job_by_pid(pid);
    if (job) {
        *priority = job->priority;
        xSemaphoreGive(s_poll.mutex);
        return ESP_OK;
    }
    
    xSemaphoreGive(s_poll.mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t poll_engine_get_value(uint16_t pid, float *value, bool *changed)
{
    if (!s_poll.initialized || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    
    poll_job_t *job = find_job_by_pid(pid);
    if (job) {
        *value = job->last_value;
        if (changed) {
            *changed = job->value_changed;
            job->value_changed = false;  // Clear flag after read
        }
        xSemaphoreGive(s_poll.mutex);
        return ESP_OK;
    }
    
    xSemaphoreGive(s_poll.mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t poll_engine_get_stats(poll_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    *stats = s_poll.stats;
    xSemaphoreGive(s_poll.mutex);
    
    return ESP_OK;
}

int poll_engine_discover_pids(uint8_t mode)
{
    if (!s_poll.initialized) {
        return -1;
    }
    
    int discovered = 0;
    
    // Standard OBD-II: check PIDs 0x00, 0x20, 0x40, 0x60, 0x80, 0xA0
    for (uint8_t base = 0x00; base <= 0xA0; base += 0x20) {
        uint32_t bitmap;
        esp_err_t err = obd2_get_supported_pids(base, &bitmap);
        
        if (err != ESP_OK || bitmap == 0) {
            break;  // No more supported PIDs
        }
        
        // Check each bit in the bitmap
        for (int bit = 0; bit < 32; bit++) {
            if (bitmap & (1 << (31 - bit))) {
                uint8_t pid = base + bit + 1;
                
                // Skip special PIDs (supported PID bitmaps)
                if (pid == 0x20 || pid == 0x40 || pid == 0x60 || 
                    pid == 0x80 || pid == 0xA0 || pid == 0xC0) {
                    continue;
                }
                
                if (poll_engine_add_pid(mode, pid, 0) == ESP_OK) {
                    discovered++;
                }
            }
        }
        
        // If bit 31 is not set, no more ranges supported
        if (!(bitmap & 0x01)) {
            break;
        }
    }
    
    SYS_LOGI(TAG, "Discovered %d supported PIDs for mode 0x%02X", discovered, mode);
    return discovered;
}

esp_err_t poll_engine_clear_all(void)
{
    if (!s_poll.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(s_poll.mutex, portMAX_DELAY);
    s_poll.job_count = 0;
    s_poll.stats.active_jobs = 0;
    xSemaphoreGive(s_poll.mutex);
    
    SYS_LOGI(TAG, "Cleared all poll jobs");
    return ESP_OK;
}

esp_err_t poll_engine_register_value_callback(poll_value_callback_t callback)
{
    if (!s_poll.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    s_poll.value_callback = callback;
    return ESP_OK;
}

