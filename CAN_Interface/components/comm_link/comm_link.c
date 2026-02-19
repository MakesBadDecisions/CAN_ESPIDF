/**
 * @file comm_link.c
 * @brief UART Communication Link to Display Node
 * 
 * Implements bidirectional UART communication using the comm_protocol format.
 */

#include "comm_link.h"
#include "system.h"
#include "ESP32-S3-N16R8-DevKitC.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "comm_link";

// ============================================================================
// Configuration
// ============================================================================

#define UART_NUM            UART_NUM_1
#define UART_TX_PIN         PIN_UART1_TX
#define UART_RX_PIN         PIN_UART1_RX
#define TX_BUF_SIZE         2048
#define RX_BUF_SIZE         COMM_LINK_RX_BUFFER
#define HEARTBEAT_PERIOD_MS 500
#define LINK_TIMEOUT_MS     2000

// ============================================================================
// TX Queue Message
// ============================================================================

typedef struct {
    uint8_t     data[COMM_MAX_PAYLOAD + COMM_HEADER_SIZE + COMM_CRC_SIZE + 1];
    uint16_t    len;
} tx_frame_t;

// ============================================================================
// Module State
// ============================================================================

static struct {
    bool                initialized;
    bool                running;
    comm_link_state_t   state;
    uint8_t             tx_sequence;
    
    // Tasks
    TaskHandle_t        rx_task;
    TaskHandle_t        hb_task;
    
    // TX queue
    QueueHandle_t       tx_queue;
    SemaphoreHandle_t   tx_mutex;
    
    // RX state machine
    uint8_t             rx_buf[RX_BUF_SIZE];
    uint16_t            rx_idx;
    
    // Callback
    comm_cmd_callback_t cmd_callback;
    
    // Statistics
    comm_link_stats_t   stats;
} s_ctx;

// ============================================================================
// CRC-16 (CCITT)
// ============================================================================

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

// ============================================================================
// Build and Send Frame
// ============================================================================

static esp_err_t send_frame(comm_msg_type_t type, const void *payload, uint16_t payload_len)
{
    if (!s_ctx.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (payload_len > COMM_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    tx_frame_t frame;
    uint16_t idx = 0;
    
    // Start byte
    frame.data[idx++] = COMM_START_BYTE;
    
    // Header
    comm_header_t header = {
        .msg_type    = (uint8_t)type,
        .sequence    = s_ctx.tx_sequence++,
        .timestamp   = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS,
        .payload_len = payload_len,
    };
    memcpy(&frame.data[idx], &header, COMM_HEADER_SIZE);
    idx += COMM_HEADER_SIZE;
    
    // Payload
    if (payload && payload_len > 0) {
        memcpy(&frame.data[idx], payload, payload_len);
        idx += payload_len;
    }
    
    // CRC over header + payload
    uint16_t crc = crc16_ccitt(&frame.data[1], COMM_HEADER_SIZE + payload_len);
    frame.data[idx++] = (crc >> 8) & 0xFF;    // CRC high
    frame.data[idx++] = crc & 0xFF;           // CRC low
    
    frame.len = idx;
    
    // Queue for TX
    if (xQueueSend(s_ctx.tx_queue, &frame, pdMS_TO_TICKS(10)) != pdTRUE) {
        s_ctx.stats.tx_errors++;
        SYS_LOGW(TAG, "TX queue full, dropped type=0x%02X", (uint8_t)type);
        return ESP_ERR_TIMEOUT;
    }

    // Heartbeat TX is high-frequency — log at VERBOSE to avoid flooding
    if ((comm_msg_type_t)type == MSG_HEARTBEAT) {
        SYS_LOGV(TAG, "UART TX -> type=0x%02X seq=%u payload=%u bytes",
                 header.msg_type, header.sequence, payload_len);
    } else {
        SYS_LOGD(TAG, "UART TX -> type=0x%02X seq=%u payload=%u bytes",
                 header.msg_type, header.sequence, payload_len);
    }

    return ESP_OK;
}

// ============================================================================
// Process Received Frame
// ============================================================================

static void process_rx_frame(const uint8_t *data, uint16_t len)
{
    // Minimum length: header + CRC
    if (len < COMM_HEADER_SIZE + COMM_CRC_SIZE) {
        s_ctx.stats.rx_errors++;
        return;
    }
    
    // Parse header
    comm_header_t header;
    memcpy(&header, data, COMM_HEADER_SIZE);
    
    // Validate payload length
    if (header.payload_len > COMM_MAX_PAYLOAD ||
        len != COMM_HEADER_SIZE + header.payload_len + COMM_CRC_SIZE) {
        s_ctx.stats.rx_errors++;
        return;
    }
    
    // Verify CRC
    uint16_t received_crc = (data[len - 2] << 8) | data[len - 1];
    uint16_t calc_crc = crc16_ccitt(data, len - COMM_CRC_SIZE);
    if (received_crc != calc_crc) {
        s_ctx.stats.rx_errors++;
        SYS_LOGW(TAG, "CRC mismatch: recv=0x%04X calc=0x%04X", received_crc, calc_crc);
        return;
    }
    
    s_ctx.stats.rx_frames++;
    s_ctx.stats.last_rx_tick = xTaskGetTickCount();

    // Update link state
    if (s_ctx.state != COMM_LINK_CONNECTED) {
        SYS_LOGI(TAG, "Link UP - first frame from Display");
    }
    s_ctx.state = COMM_LINK_CONNECTED;

    // Heartbeat RX is high-frequency — log at VERBOSE to avoid flooding
    if ((comm_msg_type_t)header.msg_type == MSG_HEARTBEAT) {
        SYS_LOGV(TAG, "UART RX <- type=0x%02X seq=%u payload=%u bytes",
                 header.msg_type, header.sequence, header.payload_len);
    } else {
        SYS_LOGD(TAG, "UART RX <- type=0x%02X seq=%u payload=%u bytes",
                 header.msg_type, header.sequence, header.payload_len);
    }

    // Handle message types
    const uint8_t *payload = (header.payload_len > 0) ? &data[COMM_HEADER_SIZE] : NULL;

    switch ((comm_msg_type_t)header.msg_type) {
        case MSG_HEARTBEAT:
            SYS_LOGV(TAG, "Heartbeat from Display Node");
            break;

        case MSG_CONFIG_CMD:
        case MSG_LOG_CONTROL:
            SYS_LOGI(TAG, "Config cmd 0x%02X from Display, payload=%u bytes",
                     header.msg_type, header.payload_len);
            if (s_ctx.cmd_callback) {
                s_ctx.cmd_callback((comm_msg_type_t)header.msg_type, payload, header.payload_len);
            }
            break;

        default:
            SYS_LOGW(TAG, "Unhandled msg type: 0x%02X seq=%u", header.msg_type, header.sequence);
            break;
    }
}

// ============================================================================
// RX Task - UART read and frame parsing
// ============================================================================

static void rx_task(void *arg)
{
    uint8_t byte;
    enum { STATE_IDLE, STATE_HEADER, STATE_PAYLOAD_CRC } rx_state = STATE_IDLE;
    uint16_t expected_len = 0;
    
    while (s_ctx.running) {
        // Read one byte with timeout
        int len = uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            // Check for link timeout
            if (s_ctx.state == COMM_LINK_CONNECTED) {
                uint32_t elapsed = (xTaskGetTickCount() - s_ctx.stats.last_rx_tick) * portTICK_PERIOD_MS;
                if (elapsed > LINK_TIMEOUT_MS) {
                    s_ctx.state = COMM_LINK_DISCONNECTED;
                    SYS_LOGW(TAG, "Link timeout - no heartbeat");
                }
            }
            continue;
        }
        
        switch (rx_state) {
            case STATE_IDLE:
                if (byte == COMM_START_BYTE) {
                    s_ctx.rx_idx = 0;
                    rx_state = STATE_HEADER;
                }
                break;
                
            case STATE_HEADER:
                s_ctx.rx_buf[s_ctx.rx_idx++] = byte;
                if (s_ctx.rx_idx >= COMM_HEADER_SIZE) {
                    // Parse header to get payload length
                    comm_header_t *hdr = (comm_header_t *)s_ctx.rx_buf;
                    expected_len = COMM_HEADER_SIZE + hdr->payload_len + COMM_CRC_SIZE;
                    
                    if (expected_len > RX_BUF_SIZE) {
                        SYS_LOGW(TAG, "Frame too large: %u", expected_len);
                        s_ctx.stats.rx_overflows++;
                        rx_state = STATE_IDLE;
                    } else if (hdr->payload_len == 0) {
                        // No payload, just need CRC
                        rx_state = STATE_PAYLOAD_CRC;
                    } else {
                        rx_state = STATE_PAYLOAD_CRC;
                    }
                }
                break;
                
            case STATE_PAYLOAD_CRC:
                s_ctx.rx_buf[s_ctx.rx_idx++] = byte;
                if (s_ctx.rx_idx >= expected_len) {
                    // Complete frame received
                    process_rx_frame(s_ctx.rx_buf, s_ctx.rx_idx);
                    rx_state = STATE_IDLE;
                }
                break;
        }
    }
    
    vTaskDelete(NULL);
}

// ============================================================================
// TX Task - Drain TX queue
// ============================================================================

static void tx_task(void *arg)
{
    tx_frame_t frame;
    
    while (s_ctx.running) {
        if (xQueueReceive(s_ctx.tx_queue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            int written = uart_write_bytes(UART_NUM, frame.data, frame.len);
            if (written == frame.len) {
                s_ctx.stats.tx_frames++;
            } else {
                s_ctx.stats.tx_errors++;
            }
        }
    }
    
    vTaskDelete(NULL);
}

// ============================================================================
// Heartbeat Task
// ============================================================================

static void heartbeat_task(void *arg)
{
    while (s_ctx.running) {
        comm_link_send_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
    
    vTaskDelete(NULL);
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t comm_link_init(void)
{
    if (s_ctx.initialized) {
        return ESP_OK;
    }
    
    memset(&s_ctx, 0, sizeof(s_ctx));
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate  = COMM_LINK_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t err = uart_param_config(UART_NUM, &uart_config);
    if (err != ESP_OK) {
        SYS_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        SYS_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_driver_install(UART_NUM, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        SYS_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Create TX queue and mutex
    s_ctx.tx_queue = xQueueCreate(COMM_LINK_TX_QUEUE_LEN, sizeof(tx_frame_t));
    if (!s_ctx.tx_queue) {
        SYS_LOGE(TAG, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }
    
    s_ctx.tx_mutex = xSemaphoreCreateMutex();
    if (!s_ctx.tx_mutex) {
        SYS_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_ctx.state = COMM_LINK_DISCONNECTED;
    s_ctx.initialized = true;
    
    SYS_LOGI(TAG, "Comm link initialized - UART%d @ %d bps", UART_NUM, COMM_LINK_BAUD_RATE);
    return ESP_OK;
}

esp_err_t comm_link_start(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_ctx.running) {
        return ESP_OK;
    }
    
    s_ctx.running = true;
    
    // Create RX task
    BaseType_t ret = xTaskCreate(rx_task, "comm_rx", 4096, NULL, 5, &s_ctx.rx_task);
    if (ret != pdPASS) {
        s_ctx.running = false;
        return ESP_ERR_NO_MEM;
    }
    
    // Create TX task
    // Stack must accommodate tx_frame_t (~1037 bytes) local on stack in send_frame()
    ret = xTaskCreate(tx_task, "comm_tx", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        s_ctx.running = false;
        return ESP_ERR_NO_MEM;
    }

    // Create heartbeat task
    // Stack must accommodate tx_frame_t (~1037 bytes) via send_frame() call chain
    ret = xTaskCreate(heartbeat_task, "comm_hb", 4096, NULL, 4, &s_ctx.hb_task);
    if (ret != pdPASS) {
        s_ctx.running = false;
        return ESP_ERR_NO_MEM;
    }
    
    SYS_LOGI(TAG, "Comm link started");
    return ESP_OK;
}

esp_err_t comm_link_stop(void)
{
    s_ctx.running = false;
    vTaskDelay(pdMS_TO_TICKS(200));  // Let tasks exit
    return ESP_OK;
}

comm_link_state_t comm_link_get_state(void)
{
    return s_ctx.state;
}

esp_err_t comm_link_send_pid(uint16_t pid, float value, uint8_t unit)
{
    comm_pid_value_t pv = {
        .pid_id    = pid,
        .value     = value,
        .unit      = unit,
        .timestamp = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS,
    };
    return send_frame(MSG_PID_DATA_SINGLE, &pv, sizeof(pv));
}

esp_err_t comm_link_send_pid_batch(const comm_pid_value_t *values, uint8_t count)
{
    if (!values || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Payload: 1 byte count + N * sizeof(comm_pid_value_t)
    uint16_t payload_len = 1 + count * sizeof(comm_pid_value_t);
    if (payload_len > COMM_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_SIZE;
    }
    
    uint8_t payload[COMM_MAX_PAYLOAD];
    payload[0] = count;
    memcpy(&payload[1], values, count * sizeof(comm_pid_value_t));
    
    return send_frame(MSG_PID_DATA_BATCH, payload, payload_len);
}

esp_err_t comm_link_send_heartbeat(void)
{
    comm_heartbeat_t hb = {
        .node_state   = 1,  // scanning
        .can_status   = 1,  // bus on (TODO: get from can_driver)
        .free_heap_kb = (uint16_t)(esp_get_free_heap_size() / 1024),
        .uptime_ms    = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS,
    };
    return send_frame(MSG_HEARTBEAT, &hb, sizeof(hb));
}

esp_err_t comm_link_send_dtcs(const uint16_t *dtcs, uint8_t count)
{
    if (count == 0) {
        // Send empty DTC list
        uint8_t zero = 0;
        return send_frame(MSG_DTC_LIST, &zero, 1);
    }
    
    // Payload: 1 byte count + N * 2 bytes
    uint16_t payload_len = 1 + count * sizeof(uint16_t);
    uint8_t payload[COMM_MAX_PAYLOAD];
    payload[0] = count;
    memcpy(&payload[1], dtcs, count * sizeof(uint16_t));
    
    return send_frame(MSG_DTC_LIST, payload, payload_len);
}

esp_err_t comm_link_send_vin(const char *vin)
{
    if (!vin) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // VIN is always 17 characters
    char vin_buf[18];
    strncpy(vin_buf, vin, 17);
    vin_buf[17] = '\0';
    
    return send_frame(MSG_VEHICLE_INFO, vin_buf, 17);
}

esp_err_t comm_link_send_vehicle_info(const comm_vehicle_info_t *info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    SYS_LOGI(TAG, "Sending vehicle info: VIN=%.17s, %u PIDs", 
             info->vin, 
             __builtin_popcount(*(uint32_t*)&info->supported_pids[0]) +
             __builtin_popcount(*(uint32_t*)&info->supported_pids[4]) +
             __builtin_popcount(*(uint32_t*)&info->supported_pids[8]));
    
    return send_frame(MSG_VEHICLE_INFO, info, sizeof(comm_vehicle_info_t));
}

esp_err_t comm_link_send_scan_status(scan_status_t status, uint16_t ecu_count, uint16_t pid_count)
{
    comm_scan_status_t scan = {
        .status = (uint8_t)status,
        .protocol = 0,  // TODO: detect protocol
        .ecu_count = ecu_count,
        .pid_count = pid_count,
        .reserved = 0,
    };
    
    return send_frame(MSG_SCAN_STATUS, &scan, sizeof(comm_scan_status_t));
}

esp_err_t comm_link_register_cmd_callback(comm_cmd_callback_t callback)
{
    s_ctx.cmd_callback = callback;
    return ESP_OK;
}

esp_err_t comm_link_get_stats(comm_link_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *stats = s_ctx.stats;
    return ESP_OK;
}

