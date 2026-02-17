/**
 * @file main.c
 * @brief CAN_ESPIDF Display Node - Entry Point
 *
 * Initializes all components and creates FreeRTOS tasks for:
 * - Display rendering (Core 1, Priority 5, ~60fps)
 * - UART comm link RX (Core 0, Priority 4)
 * - Data logger SD writes (Core 0, Priority 3)
 * - WiFi AP / config + log download (Core 0, Priority 1)
 * - System monitor (Core 0, Priority 0)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system.h"
#include "comm_link.h"

void app_main(void)
{
    // Phase 0: System initialization (NVS, logging, device info)
    esp_err_t ret = system_init();
    if (ret != ESP_OK) {
        SYS_LOGE("System init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Print device info to verify hardware
    sys_print_device_info();

    // Phase 3: Initialize comm_link (UART RX from CAN Interface node)
    ret = comm_link_init();
    if (ret != ESP_OK) {
        SYS_LOGE("Comm link init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = comm_link_start();
    if (ret != ESP_OK) {
        SYS_LOGE("Comm link start failed: %s", esp_err_to_name(ret));
        return;
    }

    SYS_LOGI("=== Display Node Ready ===");
    SYS_LOGI("Free heap: %lu bytes", (unsigned long)sys_get_free_heap());

    // TODO: Phase 4 - Initialize display_driver, gauge_engine
    // TODO: Phase 5 - Initialize data_logger (SD card)
    // TODO: Phase 6 - Initialize wifi_manager (WiFi AP for config + log download)

    // Main loop - periodic status
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Log link status
        const comm_link_stats_t *stats = comm_link_get_stats();
        comm_link_state_t state = comm_link_get_state();
        SYS_LOGI("Link: %s | rx=%lu tx=%lu pids=%lu err=%lu",
                 state == COMM_LINK_CONNECTED ? "CONNECTED" : "DISCONNECTED",
                 (unsigned long)stats->rx_frames,
                 (unsigned long)stats->tx_frames,
                 (unsigned long)stats->pid_updates,
                 (unsigned long)stats->rx_errors);
    }
}
