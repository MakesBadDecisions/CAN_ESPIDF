/**
 * @file diagnostics.c
 * @brief Vehicle Diagnostics - VIN, DTCs, ECU Discovery
 * 
 * Wraps OBD2 protocol functions with caching and higher-level operations.
 */

#include "diagnostics.h"
#include "obd2.h"
#include "system.h"
#include "comm_link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "diagnostics";

// ============================================================================
// Module State
// ============================================================================

static struct {
    bool                    initialized;
    diag_state_t            state;
    diag_vehicle_info_t     vehicle;
} s_ctx;

// ============================================================================
// Helper: Parse supported PIDs bitmask
// ============================================================================

static void parse_supported_pids(uint8_t base_pid, const uint8_t *data)
{
    // Each response is 4 bytes = 32 PIDs as a bitmask
    // base_pid: 0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0
    uint8_t idx = base_pid / 0x20;  // 0-7
    if (idx < 8) {
        s_ctx.vehicle.supported_pids[idx] = 
            ((uint32_t)data[0] << 24) |
            ((uint32_t)data[1] << 16) |
            ((uint32_t)data[2] << 8)  |
            ((uint32_t)data[3]);
    }
}

// ============================================================================
// API Implementation
// ============================================================================

esp_err_t diagnostics_init(void)
{
    if (s_ctx.initialized) {
        return ESP_OK;
    }
    
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = DIAG_STATE_IDLE;
    s_ctx.initialized = true;
    
    SYS_LOGI(TAG, "Diagnostics initialized");
    return ESP_OK;
}

diag_state_t diagnostics_get_state(void)
{
    return s_ctx.state;
}

esp_err_t diagnostics_get_vin(char *vin, bool force_refresh)
{
    if (!vin) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Return cached if valid and not forcing refresh
    if (s_ctx.vehicle.vin_valid && !force_refresh) {
        memcpy(vin, s_ctx.vehicle.vin, DIAG_VIN_LEN + 1);
        return ESP_OK;
    }
    
    s_ctx.state = DIAG_STATE_READING_VIN;
    
    esp_err_t err = obd2_read_vin(s_ctx.vehicle.vin, sizeof(s_ctx.vehicle.vin));
    
    s_ctx.state = DIAG_STATE_IDLE;
    
    if (err == ESP_OK) {
        s_ctx.vehicle.vin_valid = true;
        memcpy(vin, s_ctx.vehicle.vin, DIAG_VIN_LEN + 1);
        
        // Send to display node
        comm_link_send_vin(s_ctx.vehicle.vin);
        
        SYS_LOGI(TAG, "VIN: %s", s_ctx.vehicle.vin);
        return ESP_OK;
    }
    
    SYS_LOGW(TAG, "Failed to read VIN: %s", esp_err_to_name(err));
    return ESP_FAIL;
}

esp_err_t diagnostics_read_dtcs(diag_dtc_t *dtcs, uint8_t max_count, uint8_t *out_count)
{
    if (!out_count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *out_count = 0;
    
    s_ctx.state = DIAG_STATE_READING_DTCS;
    
    uint16_t raw_dtcs[DIAG_MAX_DTCS];
    size_t count = 0;
    
    esp_err_t err = obd2_read_dtcs(raw_dtcs, DIAG_MAX_DTCS, &count);
    
    s_ctx.state = DIAG_STATE_IDLE;
    
    if (err != ESP_OK) {
        SYS_LOGW(TAG, "Failed to read DTCs: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    
    // Cache and convert to formatted codes
    s_ctx.vehicle.dtc_count = (uint8_t)count;
    
    uint8_t copy_count = (count < max_count) ? (uint8_t)count : max_count;
    
    for (size_t i = 0; i < count; i++) {
        s_ctx.vehicle.dtcs[i].raw = raw_dtcs[i];
        obd2_format_dtc(raw_dtcs[i], s_ctx.vehicle.dtcs[i].code);
        
        if (dtcs && i < copy_count) {
            dtcs[i] = s_ctx.vehicle.dtcs[i];
        }
    }
    
    s_ctx.vehicle.dtcs_valid = true;
    *out_count = copy_count;
    
    // Send DTCs to display node
    comm_link_send_dtcs(raw_dtcs, (uint8_t)count);
    
    SYS_LOGI(TAG, "Read %zu DTCs", count);
    return ESP_OK;
}

esp_err_t diagnostics_clear_dtcs(void)
{
    s_ctx.state = DIAG_STATE_CLEARING_DTCS;
    
    esp_err_t err = obd2_clear_dtcs();
    
    s_ctx.state = DIAG_STATE_IDLE;
    
    if (err == ESP_OK) {
        s_ctx.vehicle.dtc_count = 0;
        s_ctx.vehicle.dtcs_valid = false;
        SYS_LOGI(TAG, "DTCs cleared");
        return ESP_OK;
    }
    
    SYS_LOGW(TAG, "Failed to clear DTCs: %s", esp_err_to_name(err));
    return ESP_FAIL;
}

esp_err_t diagnostics_scan_ecus(diag_ecu_t *ecus, uint8_t max_count, uint8_t *out_count)
{
    if (!out_count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *out_count = 0;
    s_ctx.state = DIAG_STATE_SCANNING;
    s_ctx.vehicle.ecu_count = 0;
    
    // Physical addressing: query each ECU (0x7E0-0x7E7) individually
    // This avoids the broadcast first-responder problem
    obd2_response_t resp;
    
    for (uint8_t i = 0; i < 8; i++) {
        uint32_t tx_id = OBD2_CAN_ECM_TX_ID + i;  // 0x7E0 + i
        uint32_t rx_id = OBD2_CAN_ECM_RX_ID + i;  // 0x7E8 + i
        
        obd2_request_t req = {
            .mode = OBD2_MODE_CURRENT_DATA,
            .pid = 0x00,
            .tx_id = tx_id,      // Physical address, not broadcast
            .timeout_ms = 150    // Short timeout for discovery
        };
        
        esp_err_t err = obd2_request(&req, &resp);
        
        if (err == ESP_OK && resp.status == OBD2_STATUS_OK && resp.ecu_id == rx_id) {
            diag_ecu_t *ecu = &s_ctx.vehicle.ecus[s_ctx.vehicle.ecu_count];
            ecu->ecu_id = rx_id;
            ecu->responded = true;
            snprintf(ecu->name, sizeof(ecu->name), "ECU_%lX", (unsigned long)rx_id);
            
            if (ecus && s_ctx.vehicle.ecu_count < max_count) {
                ecus[s_ctx.vehicle.ecu_count] = *ecu;
            }
            
            s_ctx.vehicle.ecu_count++;
            SYS_LOGI(TAG, "Found ECU: 0x%03lX", (unsigned long)rx_id);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Delay between queries
    }
    
    *out_count = s_ctx.vehicle.ecu_count;
    s_ctx.state = DIAG_STATE_IDLE;
    
    SYS_LOGI(TAG, "ECU scan complete: %u found", s_ctx.vehicle.ecu_count);
    return ESP_OK;
}

bool diagnostics_is_pid_supported(uint8_t pid)
{
    if (pid == 0) return true;  // PID 0x00 is always supported
    
    uint8_t group = (pid - 1) / 32;     // Which 32-PID group
    uint8_t bit_pos = 31 - ((pid - 1) % 32);  // Bit position (MSB first)
    
    if (group >= 8) return false;
    
    return (s_ctx.vehicle.supported_pids[group] & (1UL << bit_pos)) != 0;
}

const diag_vehicle_info_t *diagnostics_get_vehicle_info(void)
{
    return &s_ctx.vehicle;
}

esp_err_t diagnostics_auto_scan(void)
{
    SYS_LOGI(TAG, "Starting auto-scan...");
    s_ctx.state = DIAG_STATE_SCANNING;
    
    // Step 1: Query supported PIDs (broadcasts on 0x7DF)
    obd2_response_t resp;
    uint8_t pid_groups[] = {0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0};
    
    for (uint8_t i = 0; i < sizeof(pid_groups); i++) {
        esp_err_t err = obd2_request_pid(0x01, pid_groups[i], &resp);
        if (err == ESP_OK && resp.data_len >= 4) {
            parse_supported_pids(pid_groups[i], resp.data);
            SYS_LOGI(TAG, "Supported PIDs 0x%02X: %08lX", 
                     pid_groups[i], s_ctx.vehicle.supported_pids[i]);
            
            // If bit 0 (PID+0x20) is not set, no more PIDs supported
            if (i < 7 && !(resp.data[3] & 0x01)) {
                break;
            }
        } else {
            break;  // No response, stop scanning
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Step 2: Read VIN
    char vin[18];
    diagnostics_get_vin(vin, true);
    
    // Step 3: Read DTCs
    uint8_t dtc_count;
    diagnostics_read_dtcs(NULL, 0, &dtc_count);
    
    s_ctx.state = DIAG_STATE_IDLE;
    SYS_LOGI(TAG, "Auto-scan complete");
    
    return ESP_OK;
}

