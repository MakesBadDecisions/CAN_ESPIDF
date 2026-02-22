/**
 * @file pid_handler.c
 * @brief HTTP handlers — PID browser and poll list management
 *
 * Endpoints:
 *   GET  /api/pids       → all supported PIDs with selected flag
 *   POST /api/pids/poll  → set selected PID list, save to SD
 *   GET  /api/pids/poll  → current selected list + poll state
 *
 * State is delegated to pid_store (persisted to SD card).
 * When selection changes:
 *   1. pid_store updated + saved
 *   2. Gauge slots using de-selected PIDs are cleared
 *   3. Gauge dropdown options refreshed
 */

#include "wifi_internal.h"
#include "comm_link.h"
#include "gauge_engine.h"
#include "pid_store.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "http_pid";

// ============================================================================
// Helpers
// ============================================================================

/**
 * Clear any gauge slots whose PID is no longer in the selected list.
 */
static void clear_orphan_gauge_slots(void)
{
    for (int i = 0; i < GAUGE_MAX_SLOTS; i++) {
        const gauge_slot_t *slot = gauge_engine_get_slot(i);
        if (!slot || slot->pid_id == 0xFFFF) continue;
        if (GAUGE_IS_VIRTUAL(slot->pid_id)) continue;

        if (!pid_store_is_selected(slot->pid_id)) {
            gauge_engine_clear_slot(i);
            ESP_LOGI(TAG, "Cleared orphan gauge slot %d (PID 0x%02X de-selected)",
                     i, slot->pid_id);
        }
    }
}

// ============================================================================
// GET /api/pids — all supported PIDs with selection state
// ============================================================================

static esp_err_t pids_list_handler(httpd_req_t *req)
{
    int meta_count = comm_link_get_pid_meta_count();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr_chunk(req, "{\"pids\":[");

    char item[256];
    bool first = true;

    for (int i = 0; i < meta_count; i++) {
        uint16_t pid_id = comm_link_get_meta_pid_id(i);
        if (pid_id == 0xFFFF) continue;

        const char *name = comm_link_get_pid_name(pid_id);
        const char *unit = comm_link_get_pid_unit_str(pid_id);
        bool selected = pid_store_is_selected(pid_id);

        float warn = 0, crit = 0, max_v = 0;
        gauge_engine_get_alert(pid_id, &warn, &crit, &max_v);

        int len = snprintf(item, sizeof(item),
            "%s{\"id\":%u,\"hex\":\"0x%02X\",\"name\":\"%s\","
            "\"unit\":\"%s\",\"selected\":%s,"
            "\"warn\":%.4g,\"crit\":%.4g,\"max\":%.4g}",
            first ? "" : ",",
            pid_id,
            pid_id & 0xFF,
            name ? name : "Unknown",
            unit ? unit : "",
            selected ? "true" : "false",
            warn, crit, max_v);
        httpd_resp_send_chunk(req, item, len);
        first = false;
    }

    // Append virtual PIDs (IMU)
    int len = snprintf(item, sizeof(item),
        "%s{\"id\":%u,\"hex\":\"0xFF00\",\"name\":\"IMU G-Load / Tilt\","
        "\"unit\":\"g / deg\",\"selected\":false,\"virtual\":true,"
        "\"warn\":0,\"crit\":0,\"max\":0}",
        first ? "" : ",", VPID_IMU);
    httpd_resp_send_chunk(req, item, len);

    char footer[64];
    len = snprintf(footer, sizeof(footer), "],\"total\":%d}", meta_count);
    httpd_resp_send_chunk(req, footer, len);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ============================================================================
// POST /api/pids/poll — set selected PID list, save, refresh UI
// ============================================================================

static esp_err_t pids_poll_set_handler(httpd_req_t *req)
{
    // Read body
    char body[1024];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return ESP_OK;
    }
    int received = httpd_req_recv(req, body, total);
    if (received != total) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Receive error");
        return ESP_OK;
    }
    body[total] = '\0';

    // Parse rate_hz
    const char *rate_p = strstr(body, "\"rate_hz\":");
    if (rate_p) {
        int r = atoi(rate_p + 10);
        if (r >= 1 && r <= 50) pid_store_set_rate_hz((uint8_t)r);
    }

    // Parse pids array: "pids":[12, 13, 16, ...]
    uint16_t pids[PID_STORE_MAX];
    int count = 0;

    const char *arr = strstr(body, "\"pids\":[");
    if (arr) {
        arr += 8;
        while (*arr && *arr != ']' && count < PID_STORE_MAX) {
            while (*arr == ' ' || *arr == ',') arr++;
            if (*arr == ']') break;
            int pid = atoi(arr);
            if (pid > 0 && pid <= 0xFFFF) {
                pids[count++] = (uint16_t)pid;
            }
            while (*arr && *arr != ',' && *arr != ']') arr++;
        }
    }

    // Update pid_store
    pid_store_set(pids, count);
    pid_store_save();

    // Clear gauge slots that reference de-selected PIDs
    clear_orphan_gauge_slots();

    // Notify gauge engine to refresh LCD dropdowns
    gauge_engine_notify_pid_options_changed();

    ESP_LOGI(TAG, "PID selection updated: %d PIDs @ %d Hz (saved to SD)",
             count, pid_store_get_rate_hz());

    // Response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char json[64];
    int len = snprintf(json, sizeof(json),
        "{\"ok\":true,\"count\":%d,\"rate_hz\":%d}",
        count, pid_store_get_rate_hz());
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// GET /api/pids/poll — current selection + poll state
// ============================================================================

static esp_err_t pids_poll_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    int count = pid_store_get_count();
    char header[80];
    int len = snprintf(header, sizeof(header),
        "{\"active\":%s,\"rate_hz\":%d,\"count\":%d,\"pids\":[",
        gauge_engine_is_polling() ? "true" : "false",
        pid_store_get_rate_hz(), count);
    httpd_resp_send_chunk(req, header, len);

    uint16_t pids[PID_STORE_MAX];
    int n = pid_store_get_selected(pids, PID_STORE_MAX);
    char num[8];
    for (int i = 0; i < n; i++) {
        len = snprintf(num, sizeof(num), "%s%u",
                       i > 0 ? "," : "", pids[i]);
        httpd_resp_send_chunk(req, num, len);
    }

    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ============================================================================
// POST /api/pids/alerts — save alert thresholds
// ============================================================================

static esp_err_t pids_alerts_handler(httpd_req_t *req)
{
    char body[2048];
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(body)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large");
        return ESP_OK;
    }
    int received = httpd_req_recv(req, body, total);
    if (received != total) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Receive error");
        return ESP_OK;
    }
    body[total] = '\0';

    // Parse alerts array: {alerts:[{id:N,warn:V,crit:V,max:V},...]}  
    int count = 0;
    const char *p = strstr(body, "\"alerts\":");
    if (p) {
        p = strchr(p, '[');
        if (p) {
            p++;
            while (*p && *p != ']') {
                const char *obj = strchr(p, '{');
                if (!obj) break;
                obj++;

                int id = 0;
                float warn = 0, crit_v = 0, max_v = 0;
                const char *f;

                f = strstr(obj, "\"id\":");
                if (f) id = atoi(f + 5);
                f = strstr(obj, "\"warn\":");
                if (f) warn = strtof(f + 7, NULL);
                f = strstr(obj, "\"crit\":");
                if (f) crit_v = strtof(f + 7, NULL);
                f = strstr(obj, "\"max\":");
                if (f) max_v = strtof(f + 6, NULL);

                if (id > 0) {
                    gauge_engine_set_alert((uint16_t)id, warn, crit_v, max_v);
                    count++;
                }

                const char *end = strchr(obj, '}');
                if (!end) break;
                p = end + 1;
            }
        }
    }

    gauge_engine_save_alerts();
    ESP_LOGI(TAG, "Alert thresholds updated: %d PIDs", count);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char json[64];
    int len = snprintf(json, sizeof(json), "{\"ok\":true,\"count\":%d}", count);
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// Registration
// ============================================================================

static const httpd_uri_t uri_pids_list = {
    .uri      = "/api/pids",
    .method   = HTTP_GET,
    .handler  = pids_list_handler,
};

static const httpd_uri_t uri_pids_poll_set = {
    .uri      = "/api/pids/poll",
    .method   = HTTP_POST,
    .handler  = pids_poll_set_handler,
};

static const httpd_uri_t uri_pids_poll_get = {
    .uri      = "/api/pids/poll",
    .method   = HTTP_GET,
    .handler  = pids_poll_get_handler,
};

static const httpd_uri_t uri_pids_alerts = {
    .uri      = "/api/pids/alerts",
    .method   = HTTP_POST,
    .handler  = pids_alerts_handler,
};

esp_err_t pid_handler_register(httpd_handle_t server)
{
    esp_err_t ret = ESP_OK;

    if (httpd_register_uri_handler(server, &uri_pids_list) != ESP_OK) ret = ESP_FAIL;
    if (httpd_register_uri_handler(server, &uri_pids_poll_set) != ESP_OK) ret = ESP_FAIL;
    if (httpd_register_uri_handler(server, &uri_pids_poll_get) != ESP_OK) ret = ESP_FAIL;
    if (httpd_register_uri_handler(server, &uri_pids_alerts) != ESP_OK) ret = ESP_FAIL;

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Registered: PID browser & poll endpoints");
    }
    return ret;
}
