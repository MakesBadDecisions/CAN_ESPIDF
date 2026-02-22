/**
 * @file scan_handler.c
 * @brief HTTP handlers for CAN scan + vehicle info + DTC operations
 *
 * Endpoints:
 *   POST /api/can/scan       — trigger vehicle scan
 *   GET  /api/can/vehicle    — cached vehicle info + scan status
 *   GET  /api/can/dtcs       — cached DTC list
 *   POST /api/can/dtcs/read  — trigger DTC read from vehicle
 *   POST /api/can/dtcs/clear — trigger DTC clear on vehicle
 */

#include "wifi_internal.h"
#include "comm_link.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "http_scan";

// ============================================================================
// Scan callback — fires when CAN Interface finishes the scan
// ============================================================================

static void web_scan_complete_cb(scan_status_t status, const comm_vehicle_info_t *info)
{
    if (status == SCAN_STATUS_COMPLETE && info) {
        ESP_LOGI(TAG, "Web-triggered scan complete — VIN: %.17s, %d ECUs",
                 info->vin, info->ecu_count);
    } else {
        ESP_LOGW(TAG, "Web-triggered scan ended with status %d", (int)status);
    }
}

// ============================================================================
// POST /api/can/scan — trigger vehicle scan
// ============================================================================

static esp_err_t scan_post_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    scan_status_t current = comm_link_get_scan_status();

    if (current == SCAN_STATUS_IN_PROGRESS) {
        const char *resp = "{\"ok\":false,\"reason\":\"scan already in progress\"}";
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    esp_err_t err = comm_link_request_scan(web_scan_complete_cb);
    if (err != ESP_OK) {
        char resp[96];
        int len = snprintf(resp, sizeof(resp),
                           "{\"ok\":false,\"reason\":\"%s\"}", esp_err_to_name(err));
        httpd_resp_send(req, resp, len);
        return ESP_OK;
    }

    const char *resp = "{\"ok\":true,\"status\":\"in_progress\"}";
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// ============================================================================
// GET /api/can/vehicle — cached vehicle info + scan status
// ============================================================================

static const char *scan_status_str(scan_status_t s)
{
    switch (s) {
        case SCAN_STATUS_IDLE:        return "idle";
        case SCAN_STATUS_IN_PROGRESS: return "scanning";
        case SCAN_STATUS_COMPLETE:    return "complete";
        case SCAN_STATUS_FAILED:      return "failed";
        case SCAN_STATUS_NO_RESPONSE: return "no_response";
        default:                      return "unknown";
    }
}

static esp_err_t vehicle_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    scan_status_t status = comm_link_get_scan_status();
    comm_vehicle_info_t info;
    bool has_info = comm_link_get_vehicle_info(&info);

    char json[512];
    int len;

    if (has_info) {
        // Count supported PIDs from bitmap
        int pid_count = 0;
        for (int i = 0; i < 12; i++) {
            uint8_t byte = info.supported_pids[i];
            while (byte) { pid_count += byte & 1; byte >>= 1; }
        }

        len = snprintf(json, sizeof(json),
            "{"
            "\"status\":\"%s\","
            "\"vin\":\"%.17s\","
            "\"protocol\":%d,"
            "\"ecu_count\":%d,"
            "\"pid_count\":%d,"
            "\"dtc_count\":%d,"
            "\"mil_status\":%d,"
            "\"emission_dtc_count\":%d,"
            "\"ecu_name\":\"%.20s\","
            "\"cal_id\":\"%.16s\","
            "\"cvn\":\"%.8s\","
            "\"has_info\":true"
            "}",
            scan_status_str(status),
            info.vin,
            info.protocol,
            info.ecu_count,
            pid_count,
            info.dtc_count,
            info.mil_status,
            info.emission_dtc_count,
            info.ecu_name,
            info.cal_id,
            info.cvn);
    } else {
        len = snprintf(json, sizeof(json),
            "{\"status\":\"%s\",\"has_info\":false}",
            scan_status_str(status));
    }

    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// GET /api/can/dtcs — cached DTC list
// ============================================================================

static const char * const dtc_system_str[] = { "Powertrain", "Chassis", "Body", "Network" };
static const char dtc_system_prefix[]      = { 'P',          'C',       'B',    'U' };

static esp_err_t dtcs_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    comm_dtc_entry_t dtcs[COMM_LINK_DTC_STORE_MAX];
    int count = comm_link_get_dtcs(dtcs, COMM_LINK_DTC_STORE_MAX);
    bool has_data = comm_link_has_dtc_data();

    // Build JSON response — worst case ~100 bytes per DTC entry
    char *json = malloc(128 + count * 100);
    if (!json) {
        httpd_resp_send_500(req);
        return ESP_OK;
    }

    int off = sprintf(json, "{\"has_data\":%s,\"count\":%d,\"dtcs\":[",
                      has_data ? "true" : "false", count);

    for (int i = 0; i < count; i++) {
        // Format DTC code from raw u16 — identical to obd2_format_dtc():
        // bits 15-14: system (P/C/B/U), bits 13-12: first digit (0-3)
        // bits 11-8/7-4/3-0: hex digits (0-F)
        uint16_t raw = dtcs[i].code;
        uint8_t sys = (raw >> 14) & 0x03;
        uint8_t d1  = (raw >> 12) & 0x03;
        uint8_t d2  = (raw >> 8)  & 0x0F;
        uint8_t d3  = (raw >> 4)  & 0x0F;
        uint8_t d4  = raw & 0x0F;
        char code[6];
        code[0] = dtc_system_prefix[sys];
        code[1] = '0' + d1;
        code[2] = (d2 < 10) ? '0' + d2 : 'A' + d2 - 10;
        code[3] = (d3 < 10) ? '0' + d3 : 'A' + d3 - 10;
        code[4] = (d4 < 10) ? '0' + d4 : 'A' + d4 - 10;
        code[5] = '\0';

        // Build type string from bitmask
        char type_buf[32];
        int toff = 0;
        if (dtcs[i].type & 0x01) { toff += sprintf(type_buf + toff, "stored"); }
        if (dtcs[i].type & 0x02) { if (toff) toff += sprintf(type_buf + toff, ", "); toff += sprintf(type_buf + toff, "pending"); }
        if (dtcs[i].type & 0x04) { if (toff) toff += sprintf(type_buf + toff, ", "); toff += sprintf(type_buf + toff, "permanent"); }
        if (toff == 0) sprintf(type_buf, "unknown");
        const char *type_str = type_buf;

        off += sprintf(json + off,
            "%s{\"code\":\"%s\",\"raw\":%u,\"type\":\"%s\",\"system\":\"%s\"}",
            i > 0 ? "," : "",
            code,
            raw,
            type_str,
            (sys < 4) ? dtc_system_str[sys] : "Unknown");
    }

    off += sprintf(json + off, "]}");

    httpd_resp_send(req, json, off);
    free(json);
    return ESP_OK;
}

// ============================================================================
// POST /api/can/dtcs/read — trigger DTC read
// ============================================================================

static void web_dtc_read_cb(uint8_t count)
{
    ESP_LOGI(TAG, "Web-triggered DTC read complete — %u DTCs", count);
}

static esp_err_t dtcs_read_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t err = comm_link_request_dtcs(web_dtc_read_cb);
    if (err != ESP_OK) {
        char resp[96];
        int len = snprintf(resp, sizeof(resp),
                           "{\"ok\":false,\"reason\":\"%s\"}", esp_err_to_name(err));
        httpd_resp_send(req, resp, len);
        return ESP_OK;
    }

    const char *resp = "{\"ok\":true,\"status\":\"reading\"}";
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// ============================================================================
// POST /api/can/dtcs/clear — trigger DTC clear
// ============================================================================

static void web_dtc_clear_cb(uint8_t count)
{
    ESP_LOGI(TAG, "Web-triggered DTC clear complete — %u DTCs remaining", count);
}

static esp_err_t dtcs_clear_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t err = comm_link_clear_dtcs(web_dtc_clear_cb);
    if (err != ESP_OK) {
        char resp[96];
        int len = snprintf(resp, sizeof(resp),
                           "{\"ok\":false,\"reason\":\"%s\"}", esp_err_to_name(err));
        httpd_resp_send(req, resp, len);
        return ESP_OK;
    }

    const char *resp = "{\"ok\":true,\"status\":\"clearing\"}";
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

// ============================================================================
// Registration
// ============================================================================

static const httpd_uri_t uri_scan = {
    .uri      = "/api/can/scan",
    .method   = HTTP_POST,
    .handler  = scan_post_handler,
};

static const httpd_uri_t uri_vehicle = {
    .uri      = "/api/can/vehicle",
    .method   = HTTP_GET,
    .handler  = vehicle_get_handler,
};

static const httpd_uri_t uri_dtcs_get = {
    .uri      = "/api/can/dtcs",
    .method   = HTTP_GET,
    .handler  = dtcs_get_handler,
};

static const httpd_uri_t uri_dtcs_read = {
    .uri      = "/api/can/dtcs/read",
    .method   = HTTP_POST,
    .handler  = dtcs_read_handler,
};

static const httpd_uri_t uri_dtcs_clear = {
    .uri      = "/api/can/dtcs/clear",
    .method   = HTTP_POST,
    .handler  = dtcs_clear_handler,
};

esp_err_t scan_handler_register(httpd_handle_t server)
{
    esp_err_t ret;

    ret = httpd_register_uri_handler(server, &uri_scan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed POST /api/can/scan: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_vehicle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed GET /api/can/vehicle: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_dtcs_get);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed GET /api/can/dtcs: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_dtcs_read);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed POST /api/can/dtcs/read: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_dtcs_clear);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed POST /api/can/dtcs/clear: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Registered: /api/can/scan, /api/can/vehicle, /api/can/dtcs, /api/can/dtcs/read, /api/can/dtcs/clear");
    return ESP_OK;
}
