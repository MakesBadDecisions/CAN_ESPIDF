/**
 * @file main.c
 * @brief CAN_ESPIDF CAN Interface Node - Entry Point
 *
 * Orchestrates component initialization and task creation.
 * Heavy lifting is delegated to sys_mgr for runtime management.
 *
 * Init Order:
 * 1. NVS flash
 * 2. sys_mgr (state machine, task registry)
 * 3. system (logging, timing, config)
 * 4. can_driver (MCP2515 SPI)
 * 5. pid_db (PID lookup tables)
 * 6. obd2 (protocol stack)
 * 7. poll_engine (PID scheduling)
 * 8. diagnostics (DTC/UDS)
 * 9. comm_link (UART to Display Node)
 * 10. Create tasks, register with sys_mgr
 * 11. sys_mgr_start() -> CONNECTING_CAN
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "system.h"
#include "sys_mgr.h"
#include "can_driver.h"
#include "pid_db.h"
#include "obd2.h"
#include "poll_engine.h"
#include "diagnostics.h"
#include "comm_link.h"

static const char *TAG = "main";

// ============================================================================
// Callback: Forward PID values from poll_engine to comm_link
// ============================================================================

static void pid_value_callback(uint16_t pid, float value, uint8_t unit)
{
    comm_link_send_pid(pid, value, unit);
}

// ============================================================================
// Scan Task - Runs OBD2 scan in its own task so rx_task stays responsive
// ============================================================================

static TaskHandle_t s_scan_task_handle = NULL;
static volatile bool s_scan_pending = false;

static void scan_task(void *arg)
{
    while (1) {
        // Wait for scan request from cmd_callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        SYS_LOGI(TAG, "Vehicle scan starting...");
        
        // Check CAN bus state before scanning
        can_status_t can_st;
        if (can_driver_get_status(&can_st) == ESP_OK &&
            can_st.state != CAN_STATE_RUNNING) {
            SYS_LOGW(TAG, "CAN bus not running (state=%d), attempting recovery...", can_st.state);
            can_driver_clear_errors();
            vTaskDelay(pdMS_TO_TICKS(200));
            // Re-check
            if (can_driver_get_status(&can_st) == ESP_OK &&
                can_st.state != CAN_STATE_RUNNING) {
                SYS_LOGE(TAG, "CAN bus still not running after recovery, scan failed");
                comm_link_send_scan_status(SCAN_STATUS_FAILED, 0, 0);
                s_scan_pending = false;
                continue;
            }
        }
        
        // Send scan in-progress status
        comm_link_send_scan_status(SCAN_STATUS_IN_PROGRESS, 0, 0);
        
        // Clear HW filters for discovery (accept all frames)
        can_driver_clear_filters();
        
        // Build vehicle info
        comm_vehicle_info_t info;
        memset(&info, 0, sizeof(info));
        
        // Discover ECUs via physical addressing
        diag_ecu_t ecus[8];
        uint8_t ecu_count = 0;
        diagnostics_scan_ecus(ecus, 8, &ecu_count);
        info.ecu_count = ecu_count ? ecu_count : 1;
        SYS_LOGI(TAG, "Discovered %u ECU(s)", ecu_count);
        
        // Read VIN
        esp_err_t ret = obd2_read_vin(info.vin, sizeof(info.vin));
        if (ret != ESP_OK) {
            SYS_LOGW(TAG, "Failed to read VIN: %s", esp_err_to_name(ret));
            strcpy(info.vin, "UNKNOWN_VIN_12345");
        }
        
        // Read supported PIDs bitmaps
        uint32_t bitmap;
        int pid_count = 0;
        
        // PIDs 0x01-0x20
        if (obd2_get_supported_pids(0x00, &bitmap) == ESP_OK) {
            info.supported_pids[0] = (bitmap >> 24) & 0xFF;
            info.supported_pids[1] = (bitmap >> 16) & 0xFF;
            info.supported_pids[2] = (bitmap >> 8) & 0xFF;
            info.supported_pids[3] = bitmap & 0xFF;
            pid_count += __builtin_popcount(bitmap);
            
            // PIDs 0x21-0x40
            if (bitmap & 0x01) {  // PID 0x20 indicates more PIDs
                if (obd2_get_supported_pids(0x20, &bitmap) == ESP_OK) {
                    info.supported_pids[4] = (bitmap >> 24) & 0xFF;
                    info.supported_pids[5] = (bitmap >> 16) & 0xFF;
                    info.supported_pids[6] = (bitmap >> 8) & 0xFF;
                    info.supported_pids[7] = bitmap & 0xFF;
                    pid_count += __builtin_popcount(bitmap);
                    
                    // PIDs 0x41-0x60
                    if (bitmap & 0x01) {  // PID 0x40 indicates more
                        if (obd2_get_supported_pids(0x40, &bitmap) == ESP_OK) {
                            info.supported_pids[8] = (bitmap >> 24) & 0xFF;
                            info.supported_pids[9] = (bitmap >> 16) & 0xFF;
                            info.supported_pids[10] = (bitmap >> 8) & 0xFF;
                            info.supported_pids[11] = bitmap & 0xFF;
                            pid_count += __builtin_popcount(bitmap);
                        }
                    }
                }
            }
        }
        
        info.protocol = 6;  // ISO 15765-4 CAN
        info.dtc_count = 0; // TODO: read DTC count
        
        SYS_LOGI(TAG, "Scan complete: VIN=%.17s, %d PIDs supported", info.vin, pid_count);
        
        // Apply HW acceptance filters for discovered ECU response IDs
        if (ecu_count > 0) {
            uint32_t filter_ids[8];
            for (uint8_t i = 0; i < ecu_count; i++) {
                filter_ids[i] = ecus[i].ecu_id;
            }
            can_driver_set_filters(filter_ids, ecu_count);
        }
        
        // Send vehicle info to Display
        comm_link_send_vehicle_info(&info);
        
        // Send PID metadata for all supported data PIDs
        #define META_BATCH_MAX  (COMM_MAX_PAYLOAD / sizeof(comm_pid_meta_t))
        comm_pid_meta_t meta_batch[META_BATCH_MAX];
        int meta_idx = 0;
        int meta_total = 0;
        
        for (int pid = 1; pid <= MAX_SUPPORTED_PIDS; pid++) {
            // Check bitmap (bit position = pid - 1)
            uint8_t byte_idx = (pid - 1) / 8;
            uint8_t bit_idx  = 7 - ((pid - 1) % 8);
            if (!(info.supported_pids[byte_idx] & (1 << bit_idx))) {
                continue;
            }
            
            // Look up in PID database
            const pid_entry_t *entry = pid_db_lookup(pid);
            if (!entry) continue;
            
            // Only include data PIDs (formula/enum) for gauges
            if (entry->type != PID_TYPE_FORMULA && entry->type != PID_TYPE_ENUM) {
                continue;
            }
            
            // Build metadata entry
            memset(&meta_batch[meta_idx], 0, sizeof(comm_pid_meta_t));
            meta_batch[meta_idx].pid_id = entry->pid;
            meta_batch[meta_idx].unit   = (uint8_t)entry->unit;
            strncpy(meta_batch[meta_idx].name, entry->name, PID_META_NAME_LEN - 1);
            strncpy(meta_batch[meta_idx].unit_str, pid_db_unit_str(entry->unit), PID_META_UNIT_LEN - 1);
            meta_idx++;
            meta_total++;
            
            // Send batch when full
            if (meta_idx >= (int)META_BATCH_MAX) {
                comm_link_send_pid_metadata(meta_batch, meta_idx);
                meta_idx = 0;
            }
        }
        
        // Send remaining entries
        if (meta_idx > 0) {
            comm_link_send_pid_metadata(meta_batch, meta_idx);
        }
        
        SYS_LOGI(TAG, "Sent %d PID metadata entries to Display", meta_total);
        
        // Signal scan complete (Display waits for this before populating UI)
        comm_link_send_scan_status(SCAN_STATUS_COMPLETE, info.ecu_count, (uint16_t)meta_total);
        
        s_scan_pending = false;
    }
}

// ============================================================================
// Callback: Handle config commands from Display Node
// Runs inside the comm_link rx_task — must return quickly!
// ============================================================================

static void cmd_callback(comm_msg_type_t type, const uint8_t *payload, uint16_t len)
{
    if (type != MSG_CONFIG_CMD || !payload || len < sizeof(comm_config_cmd_t)) {
        return;
    }
    
    const comm_config_cmd_t *cmd = (const comm_config_cmd_t *)payload;
    
    switch ((config_cmd_type_t)cmd->cmd_type) {
        case CMD_SCAN_VEHICLE: {
            SYS_LOGI(TAG, "Vehicle scan requested by Display");
            
            if (s_scan_pending) {
                SYS_LOGW(TAG, "Scan already in progress, ignoring");
                break;
            }
            
            // Notify the scan task — don't block the rx_task with OBD2 calls
            s_scan_pending = true;
            if (s_scan_task_handle) {
                xTaskNotifyGive(s_scan_task_handle);
            } else {
                SYS_LOGE(TAG, "Scan task not created!");
                s_scan_pending = false;
                comm_link_send_scan_status(SCAN_STATUS_FAILED, 0, 0);
            }
            break;
        }
        
        case CMD_SET_POLL_LIST: {
            if (cmd->data_len >= sizeof(comm_poll_list_t)) {
                const comm_poll_list_t *poll = (const comm_poll_list_t *)cmd->cmd_data;
                SYS_LOGI(TAG, "Poll list received: %u PIDs @ %u Hz", 
                         poll->pid_count, poll->poll_rate_hz);
                
                // Clear existing jobs
                poll_engine_clear_all();
                
                // Add each PID as a poll job (mode 0x01 = standard OBD2)
                uint8_t priority = 3;  // Medium priority
                for (int i = 0; i < poll->pid_count && i < MAX_POLL_PIDS; i++) {
                    poll_engine_add_pid(0x01, poll->pids[i], priority);
                }
                
                SYS_LOGI(TAG, "Added %u poll jobs", poll->pid_count);
            }
            break;
        }
        
        case CMD_CLEAR_POLL_LIST:
            SYS_LOGI(TAG, "Clearing poll list");
            poll_engine_clear_all();
            break;
            
        default:
            SYS_LOGD(TAG, "Unhandled config cmd: 0x%02X", cmd->cmd_type);
            break;
    }
}

// ============================================================================
// Component Initialization
// ============================================================================

static esp_err_t init_components(void)
{
    esp_err_t ret;

    // CAN Driver (MCP2515 via SPI)
    ret = can_driver_init();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "CAN driver init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // PID Database
    pid_db_init();

    // OBD-II Protocol Stack
    ret = obd2_init(NULL);
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "OBD2 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Poll Engine
    ret = poll_engine_init();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Poll engine init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Diagnostics
    ret = diagnostics_init();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Diagnostics init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Comm Link (UART to Display Node)
    ret = comm_link_init();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Comm link init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wire up poll_engine -> comm_link for PID values
    poll_engine_register_value_callback(pid_value_callback);
    
    // Wire up comm_link -> main for config commands from Display
    comm_link_register_cmd_callback(cmd_callback);

    return ESP_OK;
}

// ============================================================================
// Task Creation and Component Start
// ============================================================================

static esp_err_t create_tasks(void)
{
    esp_err_t ret;
    
    // NOTE: can_driver_start() is intentionally NOT called here.
    // sys_mgr owns the CAN driver lifecycle and calls can_driver_start()
    // on entry to CONNECTING_CAN state (in sys_mgr_request_state).

    // Create scan task (handles OBD2 scan off the rx_task, Core 1, 8KB stack)
    BaseType_t xret = xTaskCreatePinnedToCore(
        scan_task, "scan", 8192, NULL, 3, &s_scan_task_handle, 1);
    if (xret != pdPASS) {
        SYS_LOGE(TAG, "Failed to create scan task");
        return ESP_ERR_NO_MEM;
    }

    // Start Comm Link (creates TX/RX/heartbeat tasks)
    ret = comm_link_start();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Comm link start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start Poll Engine (creates poll task)
    ret = poll_engine_start();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Poll engine start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SYS_LOGI(TAG, "All tasks created and started");
    return ESP_OK;
}

// ============================================================================
// Entry Point
// ============================================================================

void app_main(void)
{
    esp_err_t ret;

    // ------------------------------------------------------------------------
    // Stage 1: NVS Flash (required before any NVS operations)
    // ------------------------------------------------------------------------
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition invalid, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ------------------------------------------------------------------------
    // Stage 2: System Manager (state machine, task registry)
    // ------------------------------------------------------------------------
    ret = sys_mgr_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sys_mgr init failed: %s", esp_err_to_name(ret));
        return;
    }

    // ------------------------------------------------------------------------
    // Stage 3: System utilities (logging, timing, NVS helpers)
    // ------------------------------------------------------------------------
    ret = system_init();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "System init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Print device info to verify hardware
    sys_print_device_info();

    // ------------------------------------------------------------------------
    // Stage 4: Initialize all components
    // ------------------------------------------------------------------------
    ret = init_components();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Component init failed, halting");
        sys_mgr_report_error(SYS_ERR_NVS_FAIL, "Component init");
        return;
    }

    // ------------------------------------------------------------------------
    // Stage 5: Create application tasks
    // ------------------------------------------------------------------------
    ret = create_tasks();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "Task creation failed, halting");
        return;
    }

    // ------------------------------------------------------------------------
    // Stage 6: Start system manager (transitions to CONNECTING_CAN)
    // ------------------------------------------------------------------------
    ret = sys_mgr_start();
    if (ret != ESP_OK) {
        SYS_LOGE(TAG, "sys_mgr start failed: %s", esp_err_to_name(ret));
        return;
    }

    SYS_LOGI(TAG, "=== CAN Interface Node Started ===");
    SYS_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)sys_get_free_heap());

    // app_main returns, FreeRTOS scheduler continues
    // sys_mgr monitor task handles ongoing supervision
}

