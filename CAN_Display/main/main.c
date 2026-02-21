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
 *
 * Boot sequence adapts to the active device via device.h defines.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system.h"
#include "comm_link.h"
#include "display_driver.h"
#include "touch_driver.h"
#include "gauge_engine.h"
#include "data_logger.h"
#include "ui_events.h"
#include "lvgl.h"
#include "ui.h"
#include "device.h"
#include "i2c_bus.h"
#include "tca9554.h"
#include "qmi8658.h"
#include "boot_splash.h"

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

    // Phase 0.5: I2C bus + GPIO expander (needed before display/touch on some boards)
    ret = i2c_bus_init();
    if (ret != ESP_OK) {
        SYS_LOGW("I2C bus init failed: %s (may not be needed for this device)",
                 esp_err_to_name(ret));
    }
    ret = tca9554_init();
    if (ret != ESP_OK) {
        SYS_LOGW("TCA9554 init failed: %s (may not be present on this device)",
                 esp_err_to_name(ret));
    }

    // Phase 0.6: IMU (if present on this board)
    ret = qmi8658_init();
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        SYS_LOGW("QMI8658 IMU init failed: %s", esp_err_to_name(ret));
    }
    if (ret == ESP_OK) {
        ret = qmi8658_start_task();
        if (ret != ESP_OK) {
            SYS_LOGW("QMI8658 task start failed: %s", esp_err_to_name(ret));
        }
    }

    // Phase 1: Initialize display and LVGL
    ret = display_init();
    if (ret != ESP_OK) {
        SYS_LOGE("Display init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Clear to black immediately — avoids white flash from uninitialized framebuffers
    if (display_lock(500)) {
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
        display_unlock();
    }
    
    // Give LVGL task time to start and render black frame
    vTaskDelay(pdMS_TO_TICKS(100));

    // Phase 2: Initialize touch input (device-specific: SPI or I2C + task on Core 0)
    ret = touch_init();
    bool touch_ok = (ret == ESP_OK);
    if (!touch_ok) {
        SYS_LOGE("Touch init failed: %s", esp_err_to_name(ret));
        // Continue without touch — display still works
    }

    // Phase 3: Initialize data logger (SD card mount — needs SPI bus from touch on CrowPanel)
    ret = logger_init();
    if (ret != ESP_OK) {
        SYS_LOGW("SD card init failed - logging disabled: %s", esp_err_to_name(ret));
        // Non-fatal: continue without logging
    }

    // Phase 3.5: Boot splash from SD card (visible during remaining init)
    ret = boot_splash_show();
    if (ret == ESP_OK) {
        SYS_LOGI("Boot splash loaded from SD card");
    } else if (ret != ESP_ERR_NOT_FOUND) {
        SYS_LOGW("Boot splash failed: %s", esp_err_to_name(ret));
    }

    // Phase 4: Touch calibration check (first boot only — appears over splash)
    if (touch_ok && !touch_has_calibration()) {
        SYS_LOGI("No touch calibration found, starting calibration...");
        if (display_lock(5000)) {
            touch_start_calibration();
            display_unlock();
        }
    }

    // Phase 5: Initialize comm_link (UART RX from CAN Interface node)
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

    // Phase 6: Initialize gauge engine (per-gauge PID/unit state)
    ret = gauge_engine_init();
    if (ret != ESP_OK) {
        SYS_LOGE("Gauge engine init failed: %s", esp_err_to_name(ret));
    }

    // Phase 7: Wait for minimum splash display time, THEN load UI
    boot_splash_wait();

    if (display_lock(1000)) {
        ui_init();
        SYS_LOGI("UI initialized");
        display_unlock();
    } else {
        SYS_LOGE("Failed to acquire display lock for UI init");
    }

    // Phase 7.5: Free splash screen memory (UI has taken over display)
    boot_splash_hide();

    // Phase 8: Create status label and start status polling timer
    if (display_lock(1000)) {
        ui_events_post_init();
        display_unlock();
    }

    SYS_LOGI("=== Display Node Ready ===");
    SYS_LOGI("Free heap: %lu bytes", (unsigned long)sys_get_free_heap());
    // TODO: Initialize wifi_manager (WiFi AP for config + log download)

    // Main loop - periodic status
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Log link status
        const comm_link_stats_t *stats = comm_link_get_stats();
        comm_link_state_t state = comm_link_get_state();
        SYS_LOGD("Link: %s | rx=%lu tx=%lu pids=%lu err=%lu",
                 state == COMM_LINK_CONNECTED ? "CONNECTED" : "DISCONNECTED",
                 (unsigned long)stats->rx_frames,
                 (unsigned long)stats->tx_frames,
                 (unsigned long)stats->pid_updates,
                 (unsigned long)stats->rx_errors);
    }
}
