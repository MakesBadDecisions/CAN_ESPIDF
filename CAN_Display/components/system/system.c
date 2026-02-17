/**
 * @file system.c
 * @brief Display Node System Infrastructure Implementation
 */

#include "system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include <stdio.h>

static const char *TAG = "sys";

// ============================================================================
// System Init
// ============================================================================

esp_err_t system_init(void)
{
    ESP_LOGI(TAG, "System init - Display Node");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "System init complete");
    return ESP_OK;
}

// ============================================================================
// Device Info
// ============================================================================

void sys_print_device_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    printf("\n");
    printf("==========================================\n");
    printf("  CAN_ESPIDF Display Node\n");
    printf("==========================================\n");
    printf("Chip:   %s (Rev %d.%d)\n",
           CONFIG_IDF_TARGET,
           chip_info.revision / 100, chip_info.revision % 100);
    printf("Cores:  %d\n", chip_info.cores);
    printf("Features:");
    if (chip_info.features & CHIP_FEATURE_WIFI_BGN) printf(" WiFi");
    if (chip_info.features & CHIP_FEATURE_BT) printf(" BT");
    if (chip_info.features & CHIP_FEATURE_BLE) printf(" BLE");
    printf("\n");
    
    // Flash size
    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        printf("Flash:  %lu MB\n", (unsigned long)(flash_size / (1024 * 1024)));
    }
    
    // PSRAM
    size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram > 0) {
        printf("PSRAM:  %zu MB\n", psram / (1024 * 1024));
    } else {
        printf("PSRAM:  Not detected\n");
    }
    
    // MAC address
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        printf("MAC:    %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    
    // Heap
    printf("Heap:   %lu bytes free\n", (unsigned long)esp_get_free_heap_size());
    printf("==========================================\n\n");
}

// ============================================================================
// Timing
// ============================================================================

int64_t sys_time_us(void)
{
    return esp_timer_get_time();
}

uint32_t sys_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint32_t sys_get_free_heap(void)
{
    return esp_get_free_heap_size();
}

