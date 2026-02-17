/**
 * @file system.c
 * @brief Core System Infrastructure Implementation
 */

#include "system.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "system";

// NVS namespace for config storage
#define NVS_NAMESPACE "can_espidf"

// ============================================================================
// System Initialization
// ============================================================================

esp_err_t system_init(void)
{
    SYS_LOGI(TAG, "System initializing...");
    
    // NVS is already initialized in main.c before this is called
    // Just verify it's working
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_close(nvs);
        SYS_LOGI(TAG, "NVS namespace '%s' ready", NVS_NAMESPACE);
    } else {
        SYS_LOGW(TAG, "NVS namespace open failed: %s", esp_err_to_name(err));
    }
    
    return ESP_OK;
}

// ============================================================================
// Timing Utilities
// ============================================================================

int64_t sys_time_us(void)
{
    return esp_timer_get_time();
}

uint32_t sys_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

bool sys_timeout_check(int64_t start_us, int64_t timeout_us)
{
    return (esp_timer_get_time() - start_us) >= timeout_us;
}

void sys_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ============================================================================
// NVS Configuration
// ============================================================================

uint32_t sys_config_get_u32(const char *key, uint32_t default_val)
{
    nvs_handle_t nvs;
    uint32_t value = default_val;
    
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, key, &value);
        nvs_close(nvs);
    }
    return value;
}

esp_err_t sys_config_set_u32(const char *key, uint32_t value)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    
    err = nvs_set_u32(nvs, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

esp_err_t sys_config_get_str(const char *key, char *out_buf, size_t buf_len, const char *default_val)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    
    if (err == ESP_OK) {
        size_t len = buf_len;
        err = nvs_get_str(nvs, key, out_buf, &len);
        nvs_close(nvs);
    }
    
    if (err != ESP_OK && default_val != NULL) {
        strncpy(out_buf, default_val, buf_len - 1);
        out_buf[buf_len - 1] = '\0';
    }
    
    return err;
}

esp_err_t sys_config_set_str(const char *key, const char *value)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    
    err = nvs_set_str(nvs, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

// ============================================================================
// Device Information
// ============================================================================

static const char* get_chip_model_name(esp_chip_model_t model)
{
    switch (model) {
        case CHIP_ESP32:   return "ESP32";
        case CHIP_ESP32S2: return "ESP32-S2";
        case CHIP_ESP32S3: return "ESP32-S3";
        case CHIP_ESP32C3: return "ESP32-C3";
        case CHIP_ESP32H2: return "ESP32-H2";
        case CHIP_ESP32C2: return "ESP32-C2";
        default:           return "Unknown";
    }
}

void sys_print_device_info(void)
{
    SYS_LOGI(TAG, "========================================");
    SYS_LOGI(TAG, "       DEVICE INFORMATION");
    SYS_LOGI(TAG, "========================================");
    
    // Chip info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    SYS_LOGI(TAG, "Chip:     %s rev%d", get_chip_model_name(chip_info.model), chip_info.revision);
    SYS_LOGI(TAG, "Cores:    %d", chip_info.cores);
    SYS_LOGI(TAG, "Features: %s%s%s%s",
        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
        (chip_info.features & CHIP_FEATURE_BT) ? "BT " : "",
        (chip_info.features & CHIP_FEATURE_IEEE802154) ? "802.15.4 " : "");
    
    // Flash size
    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        SYS_LOGI(TAG, "Flash:    %lu MB (%lu bytes)", flash_size / (1024 * 1024), flash_size);
    } else {
        SYS_LOGW(TAG, "Flash:    Unknown");
    }
    
    // PSRAM
    size_t psram_size = esp_psram_get_size();
    if (psram_size > 0) {
        SYS_LOGI(TAG, "PSRAM:    %u MB (%u bytes)", (unsigned)(psram_size / (1024 * 1024)), (unsigned)psram_size);
    } else {
        SYS_LOGW(TAG, "PSRAM:    Not detected");
    }
    
    // MAC address
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        SYS_LOGI(TAG, "MAC:      %02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    
    // Heap info
    SYS_LOGI(TAG, "Heap:     %lu free / %lu min", 
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)esp_get_minimum_free_heap_size());
    
    // IDF version
    SYS_LOGI(TAG, "IDF:      %s", esp_get_idf_version());
    
    SYS_LOGI(TAG, "========================================");
}

uint32_t sys_get_free_heap(void)
{
    return esp_get_free_heap_size();
}

uint32_t sys_get_min_free_heap(void)
{
    return esp_get_minimum_free_heap_size();
}

