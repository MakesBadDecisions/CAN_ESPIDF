/**
 * @file obd2.c
 * @brief OBD-II Protocol Stack Implementation
 * 
 * Handles OBD-II request/response over CAN with ISO-TP for multi-frame.
 */

#include "obd2.h"
#include "can_driver.h"
#include "system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "obd2";

// ============================================================================
// Internal State
// ============================================================================

static struct {
    obd2_config_t   config;
    isotp_session_t isotp;
    SemaphoreHandle_t response_sem;     // Signals response received
    obd2_response_t *pending_response;  // Points to caller's response struct
    uint32_t        expected_rx_id;     // Expected response CAN ID
    bool            waiting;            // Waiting for response
} s_obd2;

// ============================================================================
// Helper Functions
// ============================================================================

static uint32_t get_response_id(uint32_t tx_id)
{
    // Map TX ID to expected RX ID
    if (tx_id == OBD2_CAN_BROADCAST_ID) {
        return 0;  // Accept any 0x7E8-0x7EF
    }
    // Physical addressing: response = request + 8
    return tx_id + 8;
}

static bool is_obd2_response_id(uint32_t id)
{
    return (id >= 0x7E8 && id <= 0x7EF);
}

static void clear_isotp_session(void)
{
    memset(&s_obd2.isotp, 0, sizeof(s_obd2.isotp));
}

// ============================================================================
// Request Building
// ============================================================================

static esp_err_t build_request_frame(const obd2_request_t *request, 
                                      can_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->id = request->tx_id;
    frame->extended = false;
    frame->rtr = false;
    
    if (request->mode == OBD2_MODE_EXTENDED && request->pid > 0xFF) {
        // Mode 0x22 with 16-bit PID: 3 bytes payload
        frame->data[0] = 0x03;  // ISO-TP single frame, 3 bytes
        frame->data[1] = request->mode;
        frame->data[2] = (request->pid >> 8) & 0xFF;
        frame->data[3] = request->pid & 0xFF;
        frame->dlc = 8;
    } else if (request->mode == OBD2_MODE_READ_DTC || 
               request->mode == OBD2_MODE_CLEAR_DTC ||
               request->mode == OBD2_MODE_PENDING_DTC ||
               request->mode == OBD2_MODE_PERMANENT_DTC) {
        // No PID for DTC modes
        frame->data[0] = 0x01;  // 1 byte payload
        frame->data[1] = request->mode;
        frame->dlc = 8;
    } else {
        // Standard Mode 01/02/09 with 8-bit PID: 2 bytes payload
        frame->data[0] = 0x02;  // ISO-TP single frame, 2 bytes
        frame->data[1] = request->mode;
        frame->data[2] = request->pid & 0xFF;
        frame->dlc = 8;
    }
    
    // Pad with 0x55 (common OBD-II padding, some use 0x00 or 0xAA)
    for (int i = frame->data[0] + 1; i < 8; i++) {
        frame->data[i] = 0x55;
    }
    
    return ESP_OK;
}

// ============================================================================
// ISO-TP Flow Control
// ============================================================================

static esp_err_t send_flow_control(uint32_t tx_id)
{
    can_frame_t fc_frame;
    memset(&fc_frame, 0, sizeof(fc_frame));
    
    fc_frame.id = tx_id;
    fc_frame.extended = false;
    fc_frame.rtr = false;
    fc_frame.dlc = 8;
    fc_frame.data[0] = ISOTP_FRAME_FLOW_CONTROL | ISOTP_FC_CONTINUE;
    fc_frame.data[1] = s_obd2.config.isotp_block_size;
    fc_frame.data[2] = s_obd2.config.isotp_sep_time_ms;
    fc_frame.data[3] = 0x55;
    fc_frame.data[4] = 0x55;
    fc_frame.data[5] = 0x55;
    fc_frame.data[6] = 0x55;
    fc_frame.data[7] = 0x55;
    
    return can_driver_send(&fc_frame, 50);
}

// ============================================================================
// Response Parsing
// ============================================================================

static void parse_single_frame(const uint8_t *data, uint8_t len,
                               obd2_response_t *resp)
{
    uint8_t payload_len = data[0] & 0x0F;
    
    if (payload_len < 1 || payload_len > 7) {
        resp->status = OBD2_STATUS_INVALID_FRAME;
        return;
    }
    
    uint8_t mode = data[1];
    
    // Check for negative response
    if (mode == OBD2_NEGATIVE_RESPONSE) {
        resp->status = OBD2_STATUS_NEGATIVE;
        resp->mode = data[2];  // Rejected mode
        resp->negative_code = (payload_len >= 3) ? data[3] : 0;
        
        // 0x78 = Response Pending, should retry
        if (resp->negative_code == 0x78) {
            resp->status = OBD2_STATUS_PENDING;
        }
        return;
    }
    
    // Validate positive response
    resp->mode = mode;
    resp->status = OBD2_STATUS_OK;
    
    // Extract PID based on mode
    if (mode == (OBD2_MODE_EXTENDED + OBD2_RESPONSE_OFFSET)) {
        // Mode 0x62: 16-bit PID
        if (payload_len >= 3) {
            resp->pid = (data[2] << 8) | data[3];
            resp->data_len = payload_len - 3;
            memcpy(resp->data, &data[4], resp->data_len);
        }
    } else if (mode == (OBD2_MODE_READ_DTC + OBD2_RESPONSE_OFFSET) ||
               mode == (OBD2_MODE_PENDING_DTC + OBD2_RESPONSE_OFFSET) ||
               mode == (OBD2_MODE_PERMANENT_DTC + OBD2_RESPONSE_OFFSET)) {
        // Mode 0x43/0x47/0x4A: DTCs, no PID
        resp->pid = 0;
        resp->data_len = payload_len - 1;
        memcpy(resp->data, &data[2], resp->data_len);
    } else {
        // Mode 0x41, 0x42, 0x49, etc.: 8-bit PID
        if (payload_len >= 2) {
            resp->pid = data[2];
            resp->data_len = payload_len - 2;
            memcpy(resp->data, &data[3], resp->data_len);
        }
    }
}

static void parse_first_frame(const uint8_t *data, uint8_t len,
                              uint32_t rx_id, uint32_t tx_id)
{
    uint16_t total_len = ((data[0] & 0x0F) << 8) | data[1];
    
    if (total_len > OBD2_MAX_PAYLOAD_LEN) {
        SYS_LOGW(TAG, "ISO-TP message too large: %u bytes", total_len);
        return;
    }
    
    // Start multi-frame session
    s_obd2.isotp.active = true;
    s_obd2.isotp.rx_id = rx_id;
    s_obd2.isotp.total_len = total_len;
    s_obd2.isotp.received_len = 6;  // First frame has 6 data bytes
    s_obd2.isotp.next_seq = 1;
    s_obd2.isotp.last_frame_tick = xTaskGetTickCount();
    
    memcpy(s_obd2.isotp.buffer, &data[2], 6);
    
    // Send flow control to continue
    send_flow_control(tx_id);
}

static void parse_consecutive_frame(const uint8_t *data, uint8_t len)
{
    if (!s_obd2.isotp.active) {
        SYS_LOGW(TAG, "Unexpected consecutive frame");
        return;
    }
    
    uint8_t seq = data[0] & 0x0F;
    
    if (seq != s_obd2.isotp.next_seq) {
        SYS_LOGE(TAG, "ISO-TP sequence error: expected %d, got %d",
                 s_obd2.isotp.next_seq, seq);
        clear_isotp_session();
        if (s_obd2.pending_response) {
            s_obd2.pending_response->status = OBD2_STATUS_ISOTP_ERROR;
        }
        return;
    }
    
    // Copy data (up to 7 bytes per consecutive frame)
    uint16_t remaining = s_obd2.isotp.total_len - s_obd2.isotp.received_len;
    uint8_t copy_len = (remaining > 7) ? 7 : remaining;
    
    memcpy(&s_obd2.isotp.buffer[s_obd2.isotp.received_len], &data[1], copy_len);
    s_obd2.isotp.received_len += copy_len;
    s_obd2.isotp.next_seq = (s_obd2.isotp.next_seq + 1) & 0x0F;
    s_obd2.isotp.last_frame_tick = xTaskGetTickCount();
    
    // Check if complete
    if (s_obd2.isotp.received_len >= s_obd2.isotp.total_len) {
        // Multi-frame complete - parse full message
        if (s_obd2.pending_response) {
            obd2_response_t *resp = s_obd2.pending_response;
            uint8_t *buf = s_obd2.isotp.buffer;
            
            resp->mode = buf[0];
            resp->status = OBD2_STATUS_OK;
            
            // Check for negative response
            if (resp->mode == OBD2_NEGATIVE_RESPONSE) {
                resp->status = OBD2_STATUS_NEGATIVE;
                resp->negative_code = (s_obd2.isotp.total_len >= 3) ? buf[2] : 0;
            } else if (resp->mode == (OBD2_MODE_EXTENDED + OBD2_RESPONSE_OFFSET)) {
                // Mode 0x62: 16-bit PID
                resp->pid = (buf[1] << 8) | buf[2];
                resp->data_len = s_obd2.isotp.total_len - 3;
                memcpy(resp->data, &buf[3], resp->data_len);
            } else if (resp->mode == (OBD2_MODE_READ_DTC + OBD2_RESPONSE_OFFSET) ||
                       resp->mode == (OBD2_MODE_PENDING_DTC + OBD2_RESPONSE_OFFSET) ||
                       resp->mode == (OBD2_MODE_PERMANENT_DTC + OBD2_RESPONSE_OFFSET)) {
                // Mode 0x43/0x47/0x4A: DTCs — data starts right after mode byte
                resp->pid = 0;
                resp->data_len = s_obd2.isotp.total_len - 1;
                memcpy(resp->data, &buf[1], resp->data_len);
            } else if (resp->mode == (OBD2_MODE_VEHICLE_INFO + OBD2_RESPONSE_OFFSET)) {
                // Mode 0x49: VIN, ECU name, etc.
                resp->pid = buf[1];
                resp->data_len = s_obd2.isotp.total_len - 2;
                memcpy(resp->data, &buf[2], resp->data_len);
            } else {
                // Standard response
                resp->pid = buf[1];
                resp->data_len = s_obd2.isotp.total_len - 2;
                memcpy(resp->data, &buf[2], resp->data_len);
            }
            
            resp->ecu_id = s_obd2.isotp.rx_id;
            xSemaphoreGive(s_obd2.response_sem);
        }
        
        clear_isotp_session();
    }
}

// ============================================================================
// Frame Processing (called from CAN RX)
// ============================================================================

/**
 * @brief CAN RX callback wrapper for can_driver callback registration
 */
static void obd2_can_rx_callback(const can_frame_t *frame)
{
    obd2_process_frame(frame->id, frame->data, frame->dlc);
}

void obd2_process_frame(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (!is_obd2_response_id(id)) {
        return;
    }

    SYS_LOGD(TAG, "OBD2 frame [0x%03lX] len=%d: %02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)id, len,
             data[0], data[1], data[2], data[3],
             data[4], data[5], data[6], data[7]);

    if (!s_obd2.waiting) {
        SYS_LOGW(TAG, "Unsolicited OBD2 frame from [0x%03lX] - not waiting", (unsigned long)id);
        return;
    }
    
    // Check if this is from expected ECU (or any if broadcast)
    if (s_obd2.expected_rx_id != 0 && id != s_obd2.expected_rx_id) {
        return;
    }
    
    uint8_t frame_type = data[0] & 0xF0;
    
    switch (frame_type) {
        case ISOTP_FRAME_SINGLE:
            if (s_obd2.pending_response) {
                parse_single_frame(data, len, s_obd2.pending_response);
                s_obd2.pending_response->ecu_id = id;
                xSemaphoreGive(s_obd2.response_sem);
            }
            break;
            
        case ISOTP_FRAME_FIRST:
            parse_first_frame(data, len, id, 
                              s_obd2.expected_rx_id ? s_obd2.expected_rx_id - 8 : OBD2_CAN_ECM_TX_ID);
            break;
            
        case ISOTP_FRAME_CONSECUTIVE:
            parse_consecutive_frame(data, len);
            break;
            
        case ISOTP_FRAME_FLOW_CONTROL:
            // We don't send multi-frame requests (yet), ignore
            break;
            
        default:
            SYS_LOGW(TAG, "Unknown ISO-TP frame type: 0x%02X", frame_type);
            break;
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t obd2_init(const obd2_config_t *config)
{
    // Apply config or defaults
    if (config) {
        s_obd2.config = *config;
    } else {
        s_obd2.config.default_timeout_ms = OBD2_DEFAULT_TIMEOUT_MS;
        s_obd2.config.max_retries = OBD2_MAX_RETRIES;
        s_obd2.config.isotp_block_size = 0;  // No limit
        s_obd2.config.isotp_sep_time_ms = OBD2_ISOTP_SEP_TIME_MS;
    }
    
    // Create response semaphore
    s_obd2.response_sem = xSemaphoreCreateBinary();
    if (!s_obd2.response_sem) {
        SYS_LOGE(TAG, "Failed to create response semaphore");
        return ESP_ERR_NO_MEM;
    }
    
    clear_isotp_session();
    s_obd2.waiting = false;
    s_obd2.pending_response = NULL;
    
    // Register CAN RX callback to receive frames
    esp_err_t err = can_driver_register_rx_callback(obd2_can_rx_callback);
    if (err != ESP_OK) {
        SYS_LOGW(TAG, "Failed to register CAN callback: %s", esp_err_to_name(err));
        // Non-fatal - continue anyway, caller can poll rx queue if needed
    }
    
    SYS_LOGI(TAG, "OBD-II stack initialized (timeout=%lu ms, retries=%d)",
             s_obd2.config.default_timeout_ms, s_obd2.config.max_retries);
    
    return ESP_OK;
}

esp_err_t obd2_request(const obd2_request_t *request, obd2_response_t *response)
{
    if (!request || !response) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize response
    memset(response, 0, sizeof(*response));
    response->status = OBD2_STATUS_TIMEOUT;
    
    // Build CAN frame
    can_frame_t frame;
    esp_err_t err = build_request_frame(request, &frame);
    if (err != ESP_OK) {
        SYS_LOGE(TAG, "Failed to build request frame");
        return err;
    }
    
    uint32_t timeout_ms = request->timeout_ms ? request->timeout_ms 
                                               : s_obd2.config.default_timeout_ms;
    uint8_t retries = s_obd2.config.max_retries;
    
retry:
    // Setup response capture
    s_obd2.expected_rx_id = get_response_id(request->tx_id);
    s_obd2.pending_response = response;
    s_obd2.waiting = true;
    xSemaphoreTake(s_obd2.response_sem, 0);  // Clear any pending signal
    
    // Send request
    err = can_driver_send(&frame, 100);
    if (err != ESP_OK) {
        SYS_LOGE(TAG, "CAN send failed: %s", esp_err_to_name(err));
        s_obd2.waiting = false;
        response->status = OBD2_STATUS_CAN_ERROR;
        return err;
    }

    SYS_LOGI(TAG, "OBD2 TX [0x%03lX] Mode=0x%02X PID=0x%04X | %02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned long)request->tx_id, request->mode, request->pid,
             frame.data[0], frame.data[1], frame.data[2], frame.data[3],
             frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    
    // Wait for response
    BaseType_t got_response = xSemaphoreTake(s_obd2.response_sem, 
                                              pdMS_TO_TICKS(timeout_ms));
    s_obd2.waiting = false;
    s_obd2.pending_response = NULL;
    
    if (got_response != pdTRUE) {
        // Abort any TX buffers stuck waiting for an ACK that never came.
        // Must happen before retry so the next CAN send has a free buffer.
        can_driver_abort_tx();

        if (retries > 0) {
            retries--;
            SYS_LOGW(TAG, "Timeout, retrying (%d left)", retries);
            goto retry;
        }

        // All retries exhausted - clear error state (bus-off recovery if needed)
        // so subsequent requests from other callers are not blocked.
        can_driver_clear_errors();
        SYS_LOGW(TAG, "No response for Mode 0x%02X PID 0x%04X",
                 request->mode, request->pid);
        return ESP_OK;  // Return OK, caller checks response->status
    }
    
    // Handle response pending
    if (response->status == OBD2_STATUS_PENDING && retries > 0) {
        retries--;
        timeout_ms = 5000;  // ECU asked for more time
        SYS_LOGI(TAG, "Response pending, waiting longer...");
        goto retry;
    }
    
    SYS_LOGI(TAG, "OBD2 RX [0x%03lX] Mode=0x%02X PID=0x%04X len=%d | %02X %02X %02X %02X",
             (unsigned long)response->ecu_id, response->mode, response->pid, response->data_len,
             response->data_len > 0 ? response->data[0] : 0xFF,
             response->data_len > 1 ? response->data[1] : 0xFF,
             response->data_len > 2 ? response->data[2] : 0xFF,
             response->data_len > 3 ? response->data[3] : 0xFF);

    return ESP_OK;
}

esp_err_t obd2_request_pid(uint8_t mode, uint16_t pid, obd2_response_t *response)
{
    obd2_request_t req = {
        .mode = mode,
        .pid = pid,
        .tx_id = OBD2_CAN_BROADCAST_ID,
        .timeout_ms = 0
    };
    return obd2_request(&req, response);
}

esp_err_t obd2_get_supported_pids(uint8_t base_pid, uint32_t *bitmap)
{
    if (!bitmap) {
        return ESP_ERR_INVALID_ARG;
    }
    
    obd2_response_t resp;
    esp_err_t err = obd2_request_pid(OBD2_MODE_CURRENT_DATA, base_pid, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status != OBD2_STATUS_OK || resp.data_len < 4) {
        *bitmap = 0;
        return ESP_OK;
    }
    
    *bitmap = (resp.data[0] << 24) | (resp.data[1] << 16) | 
              (resp.data[2] << 8) | resp.data[3];
    
    return ESP_OK;
}

esp_err_t obd2_is_pid_supported(uint16_t pid, bool *supported)
{
    if (!supported || pid > 0xFF) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Determine which bitmap to check
    uint8_t base_pid = (pid / 32) * 32;  // 0x00, 0x20, 0x40, etc.
    uint8_t bit_pos = pid % 32;
    
    uint32_t bitmap;
    esp_err_t err = obd2_get_supported_pids(base_pid, &bitmap);
    if (err != ESP_OK) {
        return err;
    }
    
    // Bit 31 = PID base+1, Bit 30 = PID base+2, etc.
    *supported = (bitmap & (1 << (31 - bit_pos))) != 0;
    
    return ESP_OK;
}

esp_err_t obd2_read_vin(char *vin_out, size_t buf_len)
{
    if (!vin_out || buf_len < 18) {
        return ESP_ERR_INVALID_ARG;
    }
    
    obd2_response_t resp;
    esp_err_t err = obd2_request_pid(OBD2_MODE_VEHICLE_INFO, 0x02, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status != OBD2_STATUS_OK) {
        return ESP_FAIL;
    }
    
    // Mode 09 PID 02 response: byte 0 = message count, bytes 1-17 = VIN
    if (resp.data_len >= 18) {
        memcpy(vin_out, &resp.data[1], 17);
        vin_out[17] = '\0';
    } else if (resp.data_len >= 17) {
        memcpy(vin_out, resp.data, 17);
        vin_out[17] = '\0';
    } else {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t obd2_read_ecu_name(char *name_out, size_t buf_len)
{
    if (!name_out || buf_len < 21) {
        return ESP_ERR_INVALID_ARG;
    }
    
    obd2_response_t resp;
    esp_err_t err = obd2_request_pid(OBD2_MODE_VEHICLE_INFO, 0x0A, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status != OBD2_STATUS_OK) {
        return ESP_FAIL;
    }
    
    // Mode 09 PID 0x0A response: byte 0 = message count, bytes 1-20 = ECU name
    // ASCII-coded, right-padded with null chars
    if (resp.data_len >= 21) {
        memcpy(name_out, &resp.data[1], 20);
        name_out[20] = '\0';
    } else if (resp.data_len >= 20) {
        memcpy(name_out, resp.data, 20);
        name_out[20] = '\0';
    } else {
        return ESP_FAIL;
    }
    
    // Trim trailing nulls/spaces for cleaner display
    for (int i = 19; i >= 0; i--) {
        if (name_out[i] == '\0' || name_out[i] == ' ') {
            name_out[i] = '\0';
        } else {
            break;
        }
    }
    
    return ESP_OK;
}

esp_err_t obd2_read_cal_id(char *cal_id_out, size_t buf_len)
{
    if (!cal_id_out || buf_len < 17) {
        return ESP_ERR_INVALID_ARG;
    }
    
    obd2_response_t resp;
    esp_err_t err = obd2_request_pid(OBD2_MODE_VEHICLE_INFO, 0x04, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status != OBD2_STATUS_OK) {
        return ESP_FAIL;
    }
    
    // Mode 09 PID 0x04 response: byte 0 = CalID count, then N×16 bytes
    // Take just the first CalID (bytes 1-16)
    if (resp.data_len >= 17) {
        memcpy(cal_id_out, &resp.data[1], 16);
        cal_id_out[16] = '\0';
    } else if (resp.data_len >= 16) {
        memcpy(cal_id_out, resp.data, 16);
        cal_id_out[16] = '\0';
    } else {
        return ESP_FAIL;
    }
    
    // Trim trailing nulls/spaces
    for (int i = 15; i >= 0; i--) {
        if (cal_id_out[i] == '\0' || cal_id_out[i] == ' ') {
            cal_id_out[i] = '\0';
        } else {
            break;
        }
    }
    
    return ESP_OK;
}

esp_err_t obd2_read_cvn(char *cvn_out, size_t buf_len)
{
    if (!cvn_out || buf_len < 9) {
        return ESP_ERR_INVALID_ARG;
    }
    
    obd2_response_t resp;
    esp_err_t err = obd2_request_pid(OBD2_MODE_VEHICLE_INFO, 0x06, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status != OBD2_STATUS_OK) {
        return ESP_FAIL;
    }
    
    // Mode 09 PID 0x06 response: byte 0 = CVN count, then N×4 bytes raw
    // Display first CVN as 8-char hex string
    if (resp.data_len >= 5) {
        snprintf(cvn_out, buf_len, "%02X%02X%02X%02X",
                 resp.data[1], resp.data[2], resp.data[3], resp.data[4]);
    } else if (resp.data_len >= 4) {
        snprintf(cvn_out, buf_len, "%02X%02X%02X%02X",
                 resp.data[0], resp.data[1], resp.data[2], resp.data[3]);
    } else {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/**
 * @brief Internal helper: read DTCs using specified OBD2 mode
 */
static esp_err_t read_dtcs_by_mode(uint8_t mode, uint16_t *dtc_array,
                                   size_t max_dtcs, size_t *dtc_count)
{
    if (!dtc_array || !dtc_count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *dtc_count = 0;
    
    obd2_response_t resp;
    obd2_request_t req = {
        .mode = mode,
        .pid = 0,
        .tx_id = OBD2_CAN_BROADCAST_ID,
        .timeout_ms = 0
    };
    
    esp_err_t err = obd2_request(&req, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status != OBD2_STATUS_OK) {
        return ESP_FAIL;
    }
    
    // First byte is DTC count, then 2-byte DTC codes
    if (resp.data_len < 1) {
        return ESP_OK;
    }
    
    size_t num_dtcs = resp.data[0];
    size_t data_idx = 1;
    
    SYS_LOGI(TAG, "Mode 0x%02X: count_byte=%zu, data_len=%u",
             mode, num_dtcs, resp.data_len);
    
    for (size_t i = 0; i < num_dtcs && *dtc_count < max_dtcs && data_idx + 1 < resp.data_len; i++) {
        uint16_t dtc = (resp.data[data_idx] << 8) | resp.data[data_idx + 1];
        data_idx += 2;
        if (dtc != 0x0000) {   // Skip zero-padding
            dtc_array[*dtc_count] = dtc;
            (*dtc_count)++;
        }
    }
    
    return ESP_OK;
}

esp_err_t obd2_read_dtcs(uint16_t *dtc_array, size_t max_dtcs, size_t *dtc_count)
{
    return read_dtcs_by_mode(OBD2_MODE_READ_DTC, dtc_array, max_dtcs, dtc_count);
}

esp_err_t obd2_read_pending_dtcs(uint16_t *dtc_array, size_t max_dtcs, size_t *dtc_count)
{
    return read_dtcs_by_mode(OBD2_MODE_PENDING_DTC, dtc_array, max_dtcs, dtc_count);
}

esp_err_t obd2_read_permanent_dtcs(uint16_t *dtc_array, size_t max_dtcs, size_t *dtc_count)
{
    return read_dtcs_by_mode(OBD2_MODE_PERMANENT_DTC, dtc_array, max_dtcs, dtc_count);
}

esp_err_t obd2_read_monitor_status(bool *mil_on, uint8_t *dtc_count)
{
    if (!mil_on || !dtc_count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *mil_on = false;
    *dtc_count = 0;
    
    obd2_response_t resp;
    esp_err_t err = obd2_request_pid(OBD2_MODE_CURRENT_DATA, 0x01, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status != OBD2_STATUS_OK || resp.data_len < 4) {
        return ESP_FAIL;
    }
    
    // Byte A: bit 7 = MIL, bits 6-0 = DTC count
    *mil_on = (resp.data[0] & 0x80) != 0;
    *dtc_count = resp.data[0] & 0x7F;
    
    SYS_LOGI(TAG, "Monitor status: MIL=%s, DTC count=%u",
             *mil_on ? "ON" : "OFF", *dtc_count);
    
    return ESP_OK;
}

esp_err_t obd2_clear_dtcs(void)
{
    obd2_response_t resp;
    obd2_request_t req = {
        .mode = OBD2_MODE_CLEAR_DTC,
        .pid = 0,
        .tx_id = OBD2_CAN_BROADCAST_ID,
        .timeout_ms = 1000  // Clear can take longer
    };
    
    esp_err_t err = obd2_request(&req, &resp);
    if (err != ESP_OK) {
        return err;
    }
    
    if (resp.status == OBD2_STATUS_OK) {
        SYS_LOGI(TAG, "DTCs cleared successfully");
        return ESP_OK;
    }
    
    SYS_LOGW(TAG, "Clear DTCs failed: %s", obd2_status_str(resp.status));
    return ESP_FAIL;
}

const char *obd2_status_str(obd2_status_t status)
{
    switch (status) {
        case OBD2_STATUS_OK:             return "OK";
        case OBD2_STATUS_TIMEOUT:        return "Timeout";
        case OBD2_STATUS_PENDING:        return "Pending";
        case OBD2_STATUS_NEGATIVE:       return "Negative Response";
        case OBD2_STATUS_INVALID_FRAME:  return "Invalid Frame";
        case OBD2_STATUS_BUFFER_OVERFLOW:return "Buffer Overflow";
        case OBD2_STATUS_ISOTP_ERROR:    return "ISO-TP Error";
        case OBD2_STATUS_CAN_ERROR:      return "CAN Error";
        case OBD2_STATUS_NOT_SUPPORTED:  return "Not Supported";
        default:                         return "Unknown";
    }
}

void obd2_format_dtc(uint16_t dtc_raw, char *out)
{
    // First 2 bits: category (P, C, B, U)
    // Next 2 bits: first digit
    // Remaining 12 bits: 3 hex digits
    
    static const char categories[] = "PCBU";
    uint8_t cat = (dtc_raw >> 14) & 0x03;
    uint8_t d1 = (dtc_raw >> 12) & 0x03;
    uint8_t d2 = (dtc_raw >> 8) & 0x0F;
    uint8_t d3 = (dtc_raw >> 4) & 0x0F;
    uint8_t d4 = dtc_raw & 0x0F;
    
    out[0] = categories[cat];
    out[1] = '0' + d1;
    out[2] = (d2 < 10) ? '0' + d2 : 'A' + d2 - 10;
    out[3] = (d3 < 10) ? '0' + d3 : 'A' + d3 - 10;
    out[4] = (d4 < 10) ? '0' + d4 : 'A' + d4 - 10;
    out[5] = '\0';
}

