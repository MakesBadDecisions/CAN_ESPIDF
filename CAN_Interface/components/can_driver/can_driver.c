/**
 * @file can_driver.c
 * @brief CAN Hardware Abstraction Layer Implementation
 * 
 * Unified interface that dispatches to the appropriate backend driver
 * (MCP2515, MCP2518FD, or TWAI) based on configuration.
 */

#include "can_driver.h"
#include "mcp2515.h"
#include "system.h"
#include "ESP32-S3-N16R8-DevKitC.h"
#include <string.h>

static const char *TAG = "can_driver";

// ============================================================================
// Driver State
// ============================================================================

#define MAX_RX_CALLBACKS 4

static struct {
    can_backend_t       backend;
    bool                initialized;
    can_rx_callback_t   rx_callbacks[MAX_RX_CALLBACKS];
    uint8_t             rx_callback_count;
} s_can_driver = {0};

// ============================================================================
// Default Initialization
// ============================================================================

esp_err_t can_driver_init(void)
{
    can_config_t config = CAN_CONFIG_DEFAULT();
    return can_driver_init_with_config(&config);
}

// ============================================================================
// Configurable Initialization
// ============================================================================

esp_err_t can_driver_init_with_config(const can_config_t *config)
{
    if (s_can_driver.initialized) {
        SYS_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    s_can_driver.backend = config->backend;
    
    switch (config->backend) {
        case CAN_BACKEND_MCP2515: {
            SYS_LOGI(TAG, "Initializing MCP2515 backend");
            
            mcp2515_config_t mcp_config = {
                .spi_host = SPI2_HOST,
                .pin_mosi = PIN_SPI2_MOSI,
                .pin_miso = PIN_SPI2_MISO,
                .pin_sck = PIN_SPI2_SCK,
                .pin_cs = PIN_MCP2515_CS,
                .pin_int = PIN_MCP2515_INT,
                .spi_clock_hz = 8000000,  // 8MHz SPI clock (MCP2515 max is 10MHz)
                .crystal_freq = config->crystal,
                .can_bitrate = config->bitrate,
                .rx_queue_len = config->rx_queue_len,
                .loopback = config->loopback,
                .listen_only = config->listen_only,
            };
            
            ret = mcp2515_init(&mcp_config);
            if (ret != ESP_OK) {
                SYS_LOGE(TAG, "MCP2515 init failed: %s", esp_err_to_name(ret));
                return ret;
            }
            break;
        }
        
        case CAN_BACKEND_MCP2518FD:
            SYS_LOGE(TAG, "MCP2518FD backend not implemented (Phase 7)");
            return ESP_ERR_NOT_SUPPORTED;
        
        case CAN_BACKEND_TWAI:
            SYS_LOGE(TAG, "TWAI backend not implemented");
            return ESP_ERR_NOT_SUPPORTED;
        
        default:
            SYS_LOGE(TAG, "Unknown backend: %d", config->backend);
            return ESP_ERR_INVALID_ARG;
    }
    
    s_can_driver.initialized = true;
    SYS_LOGI(TAG, "Initialized with %s backend @ %d bps",
             config->backend == CAN_BACKEND_MCP2515 ? "MCP2515" :
             config->backend == CAN_BACKEND_MCP2518FD ? "MCP2518FD" : "TWAI",
             config->bitrate);
    
    return ESP_OK;
}

// ============================================================================
// Deinitialization
// ============================================================================

esp_err_t can_driver_deinit(void)
{
    if (!s_can_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_OK;
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            ret = mcp2515_deinit();
            break;
        default:
            break;
    }
    
    s_can_driver.initialized = false;
    return ret;
}

// ============================================================================
// Start/Stop
// ============================================================================

esp_err_t can_driver_start(void)
{
    if (!s_can_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_start();
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t can_driver_stop(void)
{
    if (!s_can_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_stop();
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

// ============================================================================
// Send/Receive
// ============================================================================

esp_err_t can_driver_send(const can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_can_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    SYS_LOGD(TAG, "CAN TX [0x%03lX] DLC=%d: %02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)frame->id, frame->dlc,
             frame->data[0], frame->data[1], frame->data[2], frame->data[3],
             frame->data[4], frame->data[5], frame->data[6], frame->data[7]);

    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_send(frame, timeout_ms);
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t can_driver_receive(can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_can_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_receive(frame, timeout_ms);
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

uint32_t can_driver_get_rx_queue_count(void)
{
    if (!s_can_driver.initialized) {
        return 0;
    }
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_get_rx_queue_count();
        default:
            return 0;
    }
}

// ============================================================================
// Status and Errors
// ============================================================================

esp_err_t can_driver_get_status(can_status_t *status)
{
    if (!s_can_driver.initialized || !status) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(status, 0, sizeof(can_status_t));
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            status->state = mcp2515_get_state();
            mcp2515_get_stats(&status->stats);
            status->bus_active = (status->stats.rx_frames > 0 || status->stats.tx_frames > 0);
            return ESP_OK;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t can_driver_abort_tx(void)
{
    if (!s_can_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_abort_all_tx();
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t can_driver_clear_errors(void)
{
    if (!s_can_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_clear_errors();
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

QueueHandle_t can_driver_get_rx_queue(void)
{
    if (!s_can_driver.initialized) {
        return NULL;
    }
    
    switch (s_can_driver.backend) {
        case CAN_BACKEND_MCP2515:
            return mcp2515_get_rx_queue();
        default:
            return NULL;
    }
}

// ============================================================================
// RX Callback Management
// ============================================================================

esp_err_t can_driver_register_rx_callback(can_rx_callback_t callback)
{
    if (!callback) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_can_driver.rx_callback_count >= MAX_RX_CALLBACKS) {
        SYS_LOGE(TAG, "Max RX callbacks reached (%d)", MAX_RX_CALLBACKS);
        return ESP_ERR_NO_MEM;
    }
    
    // Check for duplicate
    for (int i = 0; i < s_can_driver.rx_callback_count; i++) {
        if (s_can_driver.rx_callbacks[i] == callback) {
            SYS_LOGW(TAG, "Callback already registered");
            return ESP_OK;
        }
    }
    
    s_can_driver.rx_callbacks[s_can_driver.rx_callback_count++] = callback;
    SYS_LOGD(TAG, "Registered RX callback (%d total)", s_can_driver.rx_callback_count);
    
    return ESP_OK;
}

esp_err_t can_driver_unregister_rx_callback(can_rx_callback_t callback)
{
    if (!callback) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (int i = 0; i < s_can_driver.rx_callback_count; i++) {
        if (s_can_driver.rx_callbacks[i] == callback) {
            // Shift remaining callbacks down
            for (int j = i; j < s_can_driver.rx_callback_count - 1; j++) {
                s_can_driver.rx_callbacks[j] = s_can_driver.rx_callbacks[j + 1];
            }
            s_can_driver.rx_callback_count--;
            SYS_LOGD(TAG, "Unregistered RX callback (%d remaining)", s_can_driver.rx_callback_count);
            return ESP_OK;
        }
    }
    
    return ESP_ERR_NOT_FOUND;
}

void can_driver_dispatch_rx_callbacks(const can_frame_t *frame)
{
    SYS_LOGD(TAG, "CAN RX [0x%03lX] DLC=%d: %02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)frame->id, frame->dlc,
             frame->data[0], frame->data[1], frame->data[2], frame->data[3],
             frame->data[4], frame->data[5], frame->data[6], frame->data[7]);

    for (int i = 0; i < s_can_driver.rx_callback_count; i++) {
        if (s_can_driver.rx_callbacks[i]) {
            s_can_driver.rx_callbacks[i](frame);
        }
    }
}

