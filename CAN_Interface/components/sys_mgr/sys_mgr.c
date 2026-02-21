/**
 * @file sys_mgr.c
 * @brief System Manager Implementation
 */

#include "sys_mgr.h"
#include "system.h"
#include "can_driver.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "sys_mgr";

// ============================================================================
// Internal State
// ============================================================================

static struct {
    sys_state_t       state;
    sys_stats_t       stats;
    sys_task_info_t   tasks[SYS_MGR_MAX_TASKS];
    uint8_t           task_count;
    SemaphoreHandle_t mutex;
    TaskHandle_t      monitor_task;
    bool              initialized;
} s_mgr;

// ============================================================================
// State Names
// ============================================================================

static const char* s_state_names[] = {
    [SYS_STATE_INIT]            = "INIT",
    [SYS_STATE_CONNECTING_CAN]  = "CONNECTING_CAN",
    [SYS_STATE_CONNECTING_COMM] = "CONNECTING_COMM",
    [SYS_STATE_RUNNING]         = "RUNNING",
    [SYS_STATE_ERROR]           = "ERROR",
    [SYS_STATE_FATAL]           = "FATAL",
};

// ============================================================================
// Monitor Task
// ============================================================================

#define MONITOR_PERIOD_MS   1000
#define HEAP_WARN_THRESHOLD 32768  // Warn if free heap below 32KB

static void monitor_task_func(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    
    SYS_LOGI(TAG, "Monitor task started");
    
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MONITOR_PERIOD_MS));
        
        // Update uptime
        s_mgr.stats.uptime_sec++;
        
        // Check heap
        uint32_t free_heap = sys_get_free_heap();
        uint32_t min_heap = sys_get_min_free_heap();

        if (min_heap < s_mgr.stats.heap_min_free || s_mgr.stats.heap_min_free == 0) {
            s_mgr.stats.heap_min_free = min_heap;
        }

        // Sync CAN frame counters from driver (avoids circular dep - driver doesn't know about sys_mgr)
        can_status_t can_status;
        if (can_driver_get_status(&can_status) == ESP_OK) {
            s_mgr.stats.can_frames_rx = can_status.stats.rx_frames;
            s_mgr.stats.can_frames_tx = can_status.stats.tx_frames;
        }
        
        if (free_heap < HEAP_WARN_THRESHOLD) {
            SYS_LOGW(TAG, "Low heap: %lu bytes free", (unsigned long)free_heap);
            sys_mgr_report_error(SYS_ERR_HEAP_LOW, NULL);
        }
        
        // Check task watchdogs
        uint32_t now_ms = sys_time_ms();
        for (int i = 0; i < s_mgr.task_count; i++) {
            sys_task_info_t *t = &s_mgr.tasks[i];
            if (t->wdt_timeout_ms > 0) {
                uint32_t elapsed = now_ms - t->last_checkin;
                if (elapsed > t->wdt_timeout_ms) {
                    SYS_LOGE(TAG, "Task '%s' watchdog timeout (%lu ms)", 
                            t->name, (unsigned long)elapsed);
                    sys_mgr_report_error(SYS_ERR_TASK_STUCK, t->name);
                }
            }
        }
        
        // Periodic status log (every 10 seconds — debug level)
        if (s_mgr.stats.uptime_sec % 10 == 0) {
            SYS_LOGD(TAG, "[%s] up=%lus heap=%lu/%lu rx=%lu tx=%lu err=%lu",
                     sys_mgr_state_name(s_mgr.state),
                     (unsigned long)s_mgr.stats.uptime_sec,
                     (unsigned long)free_heap,
                     (unsigned long)min_heap,
                     (unsigned long)s_mgr.stats.can_frames_rx,
                     (unsigned long)s_mgr.stats.can_frames_tx,
                     (unsigned long)s_mgr.stats.error_count);
        }

        // Detailed heap log (every 60 seconds — info level, for overnight stability tests)
        if (s_mgr.stats.uptime_sec % 60 == 0) {
            static size_t s_initial_free = 0;
            size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            size_t min_internal  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);

            if (s_initial_free == 0) {
                s_initial_free = free_heap;
            }

            SYS_LOGI(TAG, "HEAP [%lum] | int free=%u min=%u | total free=%lu delta=%d | rx=%lu tx=%lu",
                     (unsigned long)(s_mgr.stats.uptime_sec / 60),
                     free_internal, min_internal,
                     (unsigned long)free_heap,
                     (int)(s_initial_free - free_heap),
                     (unsigned long)s_mgr.stats.can_frames_rx,
                     (unsigned long)s_mgr.stats.can_frames_tx);

            if (min_internal < 20480) {
                SYS_LOGW(TAG, "LOW INTERNAL HEAP! min_ever=%u bytes", min_internal);
            }
        }
    }
}

// ============================================================================
// API Implementation
// ============================================================================

esp_err_t sys_mgr_init(void)
{
    if (s_mgr.initialized) {
        return ESP_OK;
    }
    
    memset(&s_mgr, 0, sizeof(s_mgr));
    s_mgr.state = SYS_STATE_INIT;
    
    s_mgr.mutex = xSemaphoreCreateMutex();
    if (!s_mgr.mutex) {
        SYS_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_mgr.initialized = true;
    SYS_LOGI(TAG, "System manager initialized");
    
    return ESP_OK;
}

esp_err_t sys_mgr_start(void)
{
    if (!s_mgr.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create monitor task on Core 0 with low priority
    BaseType_t ret = xTaskCreatePinnedToCore(
        monitor_task_func,
        "sys_monitor",
        4096,  // Increased from 2048 - needs stack for logging
        NULL,
        0,  // Lowest priority
        &s_mgr.monitor_task,
        0   // Core 0
    );
    
    if (ret != pdPASS) {
        SYS_LOGE(TAG, "Failed to create monitor task");
        return ESP_ERR_NO_MEM;
    }
    
    // Transition to CONNECTING_CAN
    sys_mgr_request_state(SYS_STATE_CONNECTING_CAN);
    
    SYS_LOGI(TAG, "System manager started");
    return ESP_OK;
}

sys_state_t sys_mgr_get_state(void)
{
    return s_mgr.state;
}

const char* sys_mgr_state_name(sys_state_t state)
{
    if (state <= SYS_STATE_FATAL) {
        return s_state_names[state];
    }
    return "UNKNOWN";
}

esp_err_t sys_mgr_register_task(const char *name, TaskHandle_t handle, uint32_t wdt_timeout_ms)
{
    if (!s_mgr.initialized || !name || !handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(s_mgr.mutex, portMAX_DELAY);
    
    if (s_mgr.task_count >= SYS_MGR_MAX_TASKS) {
        xSemaphoreGive(s_mgr.mutex);
        SYS_LOGE(TAG, "Task table full, cannot register '%s'", name);
        return ESP_ERR_NO_MEM;
    }
    
    sys_task_info_t *t = &s_mgr.tasks[s_mgr.task_count++];
    t->name = name;
    t->handle = handle;
    t->wdt_timeout_ms = wdt_timeout_ms;
    t->last_checkin = sys_time_ms();
    
    xSemaphoreGive(s_mgr.mutex);
    
    SYS_LOGI(TAG, "Registered task '%s' (wdt=%lu ms)", name, (unsigned long)wdt_timeout_ms);
    return ESP_OK;
}

void sys_mgr_task_checkin(TaskHandle_t handle)
{
    if (!handle) {
        handle = xTaskGetCurrentTaskHandle();
    }
    
    for (int i = 0; i < s_mgr.task_count; i++) {
        if (s_mgr.tasks[i].handle == handle) {
            s_mgr.tasks[i].last_checkin = sys_time_ms();
            return;
        }
    }
}

void sys_mgr_report_error(sys_error_t error, const char *details)
{
    s_mgr.stats.error_count++;
    
    const char *err_names[] = {
        "NONE", "CAN_BUS_OFF", "CAN_TX_FAIL", "CAN_NO_RESPONSE",
        "COMM_LINK_DOWN", "HEAP_LOW", "TASK_STUCK", "NVS_FAIL"
    };
    
    const char *err_name = (error < sizeof(err_names)/sizeof(err_names[0])) 
                          ? err_names[error] : "UNKNOWN";
    
    if (details) {
        SYS_LOGE(TAG, "Error: %s - %s", err_name, details);
    } else {
        SYS_LOGE(TAG, "Error: %s", err_name);
    }
    
    // Handle critical errors
    switch (error) {
        case SYS_ERR_CAN_BUS_OFF:
            sys_mgr_request_can_recovery();
            break;
        case SYS_ERR_COMM_LINK_DOWN:
            if (s_mgr.state == SYS_STATE_RUNNING) {
                sys_mgr_request_state(SYS_STATE_CONNECTING_COMM);
            }
            break;
        default:
            break;
    }
}

esp_err_t sys_mgr_request_state(sys_state_t new_state)
{
    sys_state_t old_state = s_mgr.state;
    
    // Validate transition
    bool valid = false;
    switch (old_state) {
        case SYS_STATE_INIT:
            valid = (new_state == SYS_STATE_CONNECTING_CAN);
            break;
        case SYS_STATE_CONNECTING_CAN:
            valid = (new_state == SYS_STATE_CONNECTING_COMM || 
                     new_state == SYS_STATE_ERROR);
            break;
        case SYS_STATE_CONNECTING_COMM:
            valid = (new_state == SYS_STATE_RUNNING || 
                     new_state == SYS_STATE_ERROR);
            break;
        case SYS_STATE_RUNNING:
            valid = (new_state == SYS_STATE_ERROR ||
                     new_state == SYS_STATE_CONNECTING_CAN ||
                     new_state == SYS_STATE_CONNECTING_COMM);
            break;
        case SYS_STATE_ERROR:
            valid = (new_state == SYS_STATE_CONNECTING_CAN ||
                     new_state == SYS_STATE_FATAL);
            break;
        case SYS_STATE_FATAL:
            valid = false;  // No exit from FATAL
            break;
    }
    
    if (!valid) {
        SYS_LOGW(TAG, "Invalid state transition: %s -> %s",
                 sys_mgr_state_name(old_state), sys_mgr_state_name(new_state));
        return ESP_ERR_INVALID_STATE;
    }
    
    s_mgr.state = new_state;
    s_mgr.stats.state_changes++;
    
    SYS_LOGI(TAG, "State: %s -> %s", 
             sys_mgr_state_name(old_state), sys_mgr_state_name(new_state));
    
    // Execute state entry actions
    switch (new_state) {
        case SYS_STATE_CONNECTING_CAN: {
            // Start the CAN driver (puts MCP2515 into normal/listen mode)
            esp_err_t ret = can_driver_start();
            if (ret != ESP_OK) {
                SYS_LOGE(TAG, "CAN driver start failed: %s", esp_err_to_name(ret));
                sys_mgr_report_error(SYS_ERR_CAN_NO_RESPONSE, "can_driver_start failed");
            } else {
                SYS_LOGI(TAG, "CAN driver started, waiting for bus activity");
            }
            break;
        }
        default:
            break;
    }
    
    return ESP_OK;
}

void sys_mgr_request_can_recovery(void)
{
    SYS_LOGW(TAG, "CAN recovery requested");
    // TODO: Signal CAN driver to reinitialize
    // For now just log it
}

void sys_mgr_get_stats(sys_stats_t *stats)
{
    if (stats) {
        xSemaphoreTake(s_mgr.mutex, portMAX_DELAY);
        memcpy(stats, &s_mgr.stats, sizeof(sys_stats_t));
        xSemaphoreGive(s_mgr.mutex);
    }
}

void sys_mgr_stat_add(size_t stat_offset, uint32_t value)
{
    if (stat_offset < sizeof(sys_stats_t)) {
        uint32_t *counter = (uint32_t*)((uint8_t*)&s_mgr.stats + stat_offset);
        __atomic_fetch_add(counter, value, __ATOMIC_RELAXED);
    }
}
