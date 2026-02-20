/**
 * @file comm_link.c
 * @brief Display Node UART Communication Implementation
 * 
 * Implements bidirectional UART communication using the comm_protocol format.
 * Receives PID data from CAN Interface and maintains local value cache.
 */

#include "comm_link.h"
#include "system.h"
#include "device.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "comm_link";

// ============================================================================
// Configuration
// ============================================================================

#define UART_NUM            UART_NUM_1
#define UART_TX_PIN         UART_COMM_TX_PIN
#define UART_RX_PIN         UART_COMM_RX_PIN
#define TX_BUF_SIZE         1024
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
    TaskHandle_t        tx_task;
    TaskHandle_t        hb_task;
    
    // TX queue
    QueueHandle_t       tx_queue;
    SemaphoreHandle_t   tx_mutex;
    
    // RX state machine
    uint8_t             rx_buf[RX_BUF_SIZE];
    uint16_t            rx_idx;
    
    // PID value cache
    pid_cache_entry_t   pid_store[COMM_LINK_PID_STORE_MAX];
    SemaphoreHandle_t   pid_mutex;
    
    // Vehicle info
    comm_vehicle_info_t vehicle_info;
    bool                vehicle_info_valid;
    scan_status_t       scan_status;
    SemaphoreHandle_t   vehicle_mutex;
    
    // PID metadata store (RAM only, repopulated each scan)
    pid_meta_entry_t    pid_meta[PID_META_STORE_MAX];
    int                 pid_meta_count;
    
    // Callbacks
    comm_pid_callback_t pid_callback;
    comm_scan_callback_t scan_callback;
    
    // Statistics
    comm_link_stats_t   stats;
} s_ctx;

// ============================================================================
// CRC-16 (CCITT) - must match Interface
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
// PID Metadata Store (RAM only)
// ============================================================================

static void clear_pid_meta(void)
{
    memset(s_ctx.pid_meta, 0, sizeof(s_ctx.pid_meta));
    s_ctx.pid_meta_count = 0;
}

static void store_pid_meta(const comm_pid_meta_t *meta)
{
    // Check for existing entry (update)
    for (int i = 0; i < PID_META_STORE_MAX; i++) {
        if (s_ctx.pid_meta[i].valid && s_ctx.pid_meta[i].pid_id == meta->pid_id) {
            s_ctx.pid_meta[i].unit = meta->unit;
            strncpy(s_ctx.pid_meta[i].name, meta->name, PID_META_NAME_LEN - 1);
            s_ctx.pid_meta[i].name[PID_META_NAME_LEN - 1] = '\0';
            strncpy(s_ctx.pid_meta[i].unit_str, meta->unit_str, PID_META_UNIT_LEN - 1);
            s_ctx.pid_meta[i].unit_str[PID_META_UNIT_LEN - 1] = '\0';
            return;
        }
    }
    
    // Find empty slot
    for (int i = 0; i < PID_META_STORE_MAX; i++) {
        if (!s_ctx.pid_meta[i].valid) {
            s_ctx.pid_meta[i].pid_id = meta->pid_id;
            s_ctx.pid_meta[i].unit = meta->unit;
            strncpy(s_ctx.pid_meta[i].name, meta->name, PID_META_NAME_LEN - 1);
            s_ctx.pid_meta[i].name[PID_META_NAME_LEN - 1] = '\0';
            strncpy(s_ctx.pid_meta[i].unit_str, meta->unit_str, PID_META_UNIT_LEN - 1);
            s_ctx.pid_meta[i].unit_str[PID_META_UNIT_LEN - 1] = '\0';
            s_ctx.pid_meta[i].valid = true;
            s_ctx.pid_meta_count++;
            return;
        }
    }
    
    ESP_LOGW(TAG, "PID metadata store full, dropping PID 0x%04X", meta->pid_id);
}

// ============================================================================
// PID Store Management
// ============================================================================

static void store_pid_value(const comm_pid_value_t *pid)
{
    if (xSemaphoreTake(s_ctx.pid_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    // Look for existing entry or empty slot
    int slot = -1;
    for (int i = 0; i < COMM_LINK_PID_STORE_MAX; i++) {
        if (s_ctx.pid_store[i].valid && s_ctx.pid_store[i].pid_id == pid->pid_id) {
            slot = i;
            break;
        }
        if (!s_ctx.pid_store[i].valid && slot < 0) {
            slot = i;
        }
    }
    
    if (slot >= 0) {
        s_ctx.pid_store[slot].pid_id = pid->pid_id;
        s_ctx.pid_store[slot].value = pid->value;
        s_ctx.pid_store[slot].unit = pid->unit;
        s_ctx.pid_store[slot].timestamp = xTaskGetTickCount();
        s_ctx.pid_store[slot].valid = true;
        s_ctx.stats.pid_updates++;
    }
    
    xSemaphoreGive(s_ctx.pid_mutex);
    
    // Notify callback
    if (s_ctx.pid_callback) {
        s_ctx.pid_callback(pid);
    }
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
        .timestamp   = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
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
        return ESP_ERR_TIMEOUT;
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
        return;
    }
    
    s_ctx.stats.rx_frames++;
    s_ctx.stats.last_rx_tick = xTaskGetTickCount();
    
    // Update link state
    if (s_ctx.state != COMM_LINK_CONNECTED) {
        s_ctx.state = COMM_LINK_CONNECTED;
        ESP_LOGI(TAG, "Link connected");
    }
    
    // Handle message types
    const uint8_t *payload = (header.payload_len > 0) ? &data[COMM_HEADER_SIZE] : NULL;
    
    switch ((comm_msg_type_t)header.msg_type) {
        case MSG_HEARTBEAT:
            // Interface heartbeat - link alive
            break;
            
        case MSG_PID_DATA_SINGLE:
            if (payload && header.payload_len >= sizeof(comm_pid_value_t)) {
                const comm_pid_value_t *pid = (const comm_pid_value_t *)payload;
                store_pid_value(pid);
            }
            break;
            
        case MSG_PID_DATA_BATCH:
            if (payload) {
                // Batch contains array of comm_pid_value_t
                size_t count = header.payload_len / sizeof(comm_pid_value_t);
                const comm_pid_value_t *pids = (const comm_pid_value_t *)payload;
                for (size_t i = 0; i < count; i++) {
                    store_pid_value(&pids[i]);
                }
                ESP_LOGD(TAG, "Batch: %u PIDs", (unsigned)count);
            }
            break;
            
        case MSG_VEHICLE_INFO:
            if (payload && header.payload_len >= sizeof(comm_vehicle_info_t)) {
                if (xSemaphoreTake(s_ctx.vehicle_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    memcpy(&s_ctx.vehicle_info, payload, sizeof(comm_vehicle_info_t));
                    s_ctx.vehicle_info_valid = true;
                    xSemaphoreGive(s_ctx.vehicle_mutex);
                    
                    ESP_LOGI(TAG, "Vehicle info: VIN=%.17s, %u ECUs",
                             s_ctx.vehicle_info.vin,
                             s_ctx.vehicle_info.ecu_count);
                }
            }
            break;
            
        case MSG_DTC_LIST:
            // TODO: Store DTC list
            ESP_LOGI(TAG, "DTC list received");
            break;
            
        case MSG_PID_METADATA:
            if (payload && header.payload_len >= sizeof(comm_pid_meta_t)) {
                size_t count = header.payload_len / sizeof(comm_pid_meta_t);
                const comm_pid_meta_t *metas = (const comm_pid_meta_t *)payload;
                for (size_t i = 0; i < count; i++) {
                    store_pid_meta(&metas[i]);
                }
                ESP_LOGI(TAG, "PID metadata: %u entries received (total: %d)",
                         (unsigned)count, s_ctx.pid_meta_count);
            }
            break;
            
        case MSG_SCAN_STATUS:
            if (payload && header.payload_len >= sizeof(comm_scan_status_t)) {
                const comm_scan_status_t *status = (const comm_scan_status_t *)payload;
                s_ctx.scan_status = (scan_status_t)status->status;
                
                ESP_LOGI(TAG, "Scan status: %d, %u ECUs, %u PIDs",
                         status->status, status->ecu_count, status->pid_count);
                
                // Clear metadata store when scan starts
                if (status->status == SCAN_STATUS_IN_PROGRESS) {
                    clear_pid_meta();
                }
                
                // Scan complete: all metadata received, fire callback
                if (status->status == SCAN_STATUS_COMPLETE) {
                    s_ctx.scan_status = SCAN_STATUS_COMPLETE;
                    if (s_ctx.scan_callback) {
                        s_ctx.scan_callback(SCAN_STATUS_COMPLETE,
                                            s_ctx.vehicle_info_valid ? &s_ctx.vehicle_info : NULL);
                        s_ctx.scan_callback = NULL;
                    }
                }
                
                // Scan failed
                if (status->status == SCAN_STATUS_FAILED || 
                    status->status == SCAN_STATUS_NO_RESPONSE) {
                    if (s_ctx.scan_callback) {
                        s_ctx.scan_callback((scan_status_t)status->status, NULL);
                        s_ctx.scan_callback = NULL;
                    }
                }
            }
            break;
            
        case MSG_ERROR_ALERT:
            // TODO: Display error alert
            ESP_LOGW(TAG, "Error alert from Interface");
            break;
            
        default:
            ESP_LOGD(TAG, "Unhandled msg type: 0x%02X", header.msg_type);
            break;
    }
}

// ============================================================================
// RX Task - UART read and frame parsing
// ============================================================================

static void rx_task_func(void *arg)
{
    (void)arg;
    uint8_t byte;
    enum { STATE_IDLE, STATE_HEADER, STATE_PAYLOAD_CRC } rx_state = STATE_IDLE;
    uint16_t expected_len = 0;
    
    ESP_LOGI(TAG, "RX task started");
    
    while (s_ctx.running) {
        // Read one byte with timeout
        int len = uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            // Check for link timeout
            if (s_ctx.state == COMM_LINK_CONNECTED) {
                uint32_t elapsed = (xTaskGetTickCount() - s_ctx.stats.last_rx_tick) * portTICK_PERIOD_MS;
                if (elapsed > LINK_TIMEOUT_MS) {
                    s_ctx.state = COMM_LINK_DISCONNECTED;
                    ESP_LOGW(TAG, "Link timeout - no heartbeat from Interface");
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
                        ESP_LOGW(TAG, "Framing error: payload_len=%u (type=0x%02X seq=%u) - "
                                 "flushing RX buffer",
                                 hdr->payload_len, hdr->msg_type, hdr->sequence);
                        // Flush the UART RX buffer so we re-sync on the next real start byte
                        // rather than processing the remaining bytes of the corrupted frame.
                        uart_flush_input(UART_NUM);
                        s_ctx.stats.rx_errors++;
                        rx_state = STATE_IDLE;
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
    
    ESP_LOGI(TAG, "RX task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// TX Task - Drain TX queue
// ============================================================================

static void tx_task_func(void *arg)
{
    (void)arg;
    tx_frame_t frame;
    
    ESP_LOGI(TAG, "TX task started");
    
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
    
    ESP_LOGI(TAG, "TX task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// Heartbeat Task
// ============================================================================

static void heartbeat_task_func(void *arg)
{
    (void)arg;
    
    ESP_LOGI(TAG, "Heartbeat task started");
    
    while (s_ctx.running) {
        comm_link_send_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
    
    ESP_LOGI(TAG, "Heartbeat task stopped");
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
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = uart_driver_install(UART_NUM, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Create TX queue and mutexes
    s_ctx.tx_queue = xQueueCreate(COMM_LINK_TX_QUEUE_LEN, sizeof(tx_frame_t));
    if (!s_ctx.tx_queue) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }
    
    s_ctx.tx_mutex = xSemaphoreCreateMutex();
    if (!s_ctx.tx_mutex) {
        ESP_LOGE(TAG, "Failed to create TX mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_ctx.pid_mutex = xSemaphoreCreateMutex();
    if (!s_ctx.pid_mutex) {
        ESP_LOGE(TAG, "Failed to create PID mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_ctx.vehicle_mutex = xSemaphoreCreateMutex();
    if (!s_ctx.vehicle_mutex) {
        ESP_LOGE(TAG, "Failed to create vehicle mutex");
        return ESP_ERR_NO_MEM;
    }
    
    s_ctx.initialized = true;
    ESP_LOGI(TAG, "Comm link initialized - UART%d TX=%d RX=%d @ %d bps", 
             UART_NUM, UART_TX_PIN, UART_RX_PIN, COMM_LINK_BAUD_RATE);
    
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
    s_ctx.stats.last_rx_tick = xTaskGetTickCount();
    
    // Create RX task
    BaseType_t ret = xTaskCreatePinnedToCore(
        rx_task_func,
        "comm_rx",
        4096,
        NULL,
        5,      // Medium priority
        &s_ctx.rx_task,
        1       // Core 1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        s_ctx.running = false;
        return ESP_ERR_NO_MEM;
    }
    
    // Create TX task
    ret = xTaskCreatePinnedToCore(
        tx_task_func,
        "comm_tx",
        3072,
        NULL,
        5,
        &s_ctx.tx_task,
        1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task");
        s_ctx.running = false;
        return ESP_ERR_NO_MEM;
    }
    
    // Create heartbeat task
    ret = xTaskCreatePinnedToCore(
        heartbeat_task_func,
        "comm_hb",
        2048,
        NULL,
        3,      // Lower priority
        &s_ctx.hb_task,
        1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heartbeat task");
        s_ctx.running = false;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Comm link started");
    return ESP_OK;
}

esp_err_t comm_link_stop(void)
{
    if (!s_ctx.running) {
        return ESP_OK;
    }
    
    s_ctx.running = false;
    
    // Tasks will exit on their own
    vTaskDelay(pdMS_TO_TICKS(200));
    
    return ESP_OK;
}

comm_link_state_t comm_link_get_state(void)
{
    return s_ctx.state;
}

const comm_link_stats_t* comm_link_get_stats(void)
{
    return &s_ctx.stats;
}

bool comm_link_get_pid(uint16_t pid_id, pid_cache_entry_t *out_value)
{
    if (!out_value) {
        return false;
    }
    
    bool found = false;
    if (xSemaphoreTake(s_ctx.pid_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < COMM_LINK_PID_STORE_MAX; i++) {
            if (s_ctx.pid_store[i].valid && s_ctx.pid_store[i].pid_id == pid_id) {
                *out_value = s_ctx.pid_store[i];
                found = true;
                break;
            }
        }
        xSemaphoreGive(s_ctx.pid_mutex);
    }
    
    return found;
}

int comm_link_get_all_pids(pid_cache_entry_t *out_values, int max_count)
{
    if (!out_values || max_count <= 0) {
        return 0;
    }
    
    int count = 0;
    if (xSemaphoreTake(s_ctx.pid_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < COMM_LINK_PID_STORE_MAX && count < max_count; i++) {
            if (s_ctx.pid_store[i].valid) {
                out_values[count++] = s_ctx.pid_store[i];
            }
        }
        xSemaphoreGive(s_ctx.pid_mutex);
    }
    
    return count;
}

void comm_link_register_pid_callback(comm_pid_callback_t cb)
{
    s_ctx.pid_callback = cb;
}

esp_err_t comm_link_send_heartbeat(void)
{
    comm_heartbeat_t hb = {
        .node_state   = 0,  // Display is passive
        .can_status   = 0,  // N/A for display
        .free_heap_kb = (uint16_t)(heap_caps_get_free_size(MALLOC_CAP_DEFAULT) / 1024),
        .uptime_ms    = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
    };
    
    return send_frame(MSG_HEARTBEAT, &hb, sizeof(hb));
}

// ============================================================================
// Vehicle Scan API
// ============================================================================

esp_err_t comm_link_request_scan(comm_scan_callback_t callback)
{
    if (!s_ctx.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Build config command
    uint8_t cmd_buf[sizeof(comm_config_cmd_t)];
    comm_config_cmd_t *cmd = (comm_config_cmd_t *)cmd_buf;
    cmd->cmd_type = CMD_SCAN_VEHICLE;
    cmd->reserved = 0;
    cmd->data_len = 0;
    
    s_ctx.scan_status = SCAN_STATUS_IN_PROGRESS;
    s_ctx.scan_callback = callback;
    
    ESP_LOGI(TAG, "Requesting vehicle scan...");
    return send_frame(MSG_CONFIG_CMD, cmd_buf, sizeof(comm_config_cmd_t));
}

scan_status_t comm_link_get_scan_status(void)
{
    return s_ctx.scan_status;
}

bool comm_link_get_vehicle_info(comm_vehicle_info_t *out_info)
{
    if (!out_info) {
        return false;
    }
    
    bool valid = false;
    if (xSemaphoreTake(s_ctx.vehicle_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_ctx.vehicle_info_valid) {
            memcpy(out_info, &s_ctx.vehicle_info, sizeof(comm_vehicle_info_t));
            valid = true;
        }
        xSemaphoreGive(s_ctx.vehicle_mutex);
    }
    
    return valid;
}

bool comm_link_is_pid_supported(uint16_t pid_id)
{
    if (pid_id == 0 || pid_id > MAX_SUPPORTED_PIDS) {
        return false;
    }
    
    bool supported = false;
    if (xSemaphoreTake(s_ctx.vehicle_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_ctx.vehicle_info_valid) {
            uint8_t byte_idx = (pid_id - 1) / 8;
            uint8_t bit_idx = 7 - ((pid_id - 1) % 8);
            supported = (s_ctx.vehicle_info.supported_pids[byte_idx] & (1 << bit_idx)) != 0;
        }
        xSemaphoreGive(s_ctx.vehicle_mutex);
    }
    
    return supported;
}

int comm_link_get_supported_pids(uint16_t *out_pids, int max_count)
{
    if (!out_pids || max_count <= 0) {
        return 0;
    }
    
    int count = 0;
    if (xSemaphoreTake(s_ctx.vehicle_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_ctx.vehicle_info_valid) {
            for (int pos = 0; pos < MAX_SUPPORTED_PIDS && count < max_count; pos++) {
                uint8_t byte_idx = pos / 8;
                uint8_t bit_idx = 7 - (pos % 8);
                if (s_ctx.vehicle_info.supported_pids[byte_idx] & (1 << bit_idx)) {
                    out_pids[count++] = pos + 1;  // Bitmap pos 0 = PID 1
                }
            }
        }
        xSemaphoreGive(s_ctx.vehicle_mutex);
    }
    
    return count;
}

// ============================================================================
// Poll List API
// ============================================================================

esp_err_t comm_link_set_poll_list(const uint16_t *pids, uint8_t count, uint8_t rate_hz)
{
    if (!s_ctx.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!pids || count == 0 || count > MAX_POLL_PIDS) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Build payload: config_cmd header + poll_list
    uint8_t buf[sizeof(comm_config_cmd_t) + sizeof(comm_poll_list_t)];
    comm_config_cmd_t *cmd = (comm_config_cmd_t *)buf;
    comm_poll_list_t *poll = (comm_poll_list_t *)(buf + sizeof(comm_config_cmd_t));
    
    cmd->cmd_type = CMD_SET_POLL_LIST;
    cmd->reserved = 0;
    cmd->data_len = sizeof(comm_poll_list_t);
    
    poll->pid_count = count;
    poll->poll_rate_hz = rate_hz;
    memcpy(poll->pids, pids, count * sizeof(uint16_t));
    
    ESP_LOGI(TAG, "Setting poll list: %u PIDs @ %u Hz", count, rate_hz);
    return send_frame(MSG_CONFIG_CMD, buf, sizeof(comm_config_cmd_t) + sizeof(comm_poll_list_t));
}

esp_err_t comm_link_clear_poll_list(void)
{
    if (!s_ctx.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t cmd_buf[sizeof(comm_config_cmd_t)];
    comm_config_cmd_t *cmd = (comm_config_cmd_t *)cmd_buf;
    cmd->cmd_type = CMD_CLEAR_POLL_LIST;
    cmd->reserved = 0;
    cmd->data_len = 0;
    
    ESP_LOGI(TAG, "Clearing poll list");
    return send_frame(MSG_CONFIG_CMD, cmd_buf, sizeof(comm_config_cmd_t));
}

// ============================================================================
// PID Metadata API (RAM store populated from scan)
// ============================================================================

const char *comm_link_get_pid_name(uint16_t pid_id)
{
    for (int i = 0; i < PID_META_STORE_MAX; i++) {
        if (s_ctx.pid_meta[i].valid && s_ctx.pid_meta[i].pid_id == pid_id) {
            return s_ctx.pid_meta[i].name;
        }
    }
    return NULL;
}

const char *comm_link_get_pid_unit_str(uint16_t pid_id)
{
    for (int i = 0; i < PID_META_STORE_MAX; i++) {
        if (s_ctx.pid_meta[i].valid && s_ctx.pid_meta[i].pid_id == pid_id) {
            return s_ctx.pid_meta[i].unit_str;
        }
    }
    return NULL;
}

pid_unit_t comm_link_get_pid_unit(uint16_t pid_id)
{
    for (int i = 0; i < PID_META_STORE_MAX; i++) {
        if (s_ctx.pid_meta[i].valid && s_ctx.pid_meta[i].pid_id == pid_id) {
            return (pid_unit_t)s_ctx.pid_meta[i].unit;
        }
    }
    return PID_UNIT_NONE;
}

uint16_t comm_link_get_meta_pid_id(int index)
{
    int count = 0;
    for (int i = 0; i < PID_META_STORE_MAX; i++) {
        if (s_ctx.pid_meta[i].valid) {
            if (count == index) {
                return s_ctx.pid_meta[i].pid_id;
            }
            count++;
        }
    }
    return 0xFFFF;
}

int comm_link_get_pid_meta_count(void)
{
    return s_ctx.pid_meta_count;
}
