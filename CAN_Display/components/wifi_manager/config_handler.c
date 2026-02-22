/**
 * @file config_handler.c
 * @brief HTTP handlers — gauge config, display settings, WiFi settings
 *
 * Endpoints:
 *   GET    /api/gauges           → all 20 gauge slots as JSON array
 *   POST   /api/gauges           → set a slot {slot, pid_index, unit_index}
 *   DELETE /api/gauges           → clear a slot {slot}
 *   GET    /api/gauges/pids      → PID dropdown options (newline-separated)
 *   GET    /api/gauges/units     → unit options for a slot ?slot=N
 *   GET    /api/config/splash    → {duration_ms}
 *   POST   /api/config/splash    → set {duration_ms}
 *   GET    /api/config/backlight → {brightness}
 *   POST   /api/config/backlight → set {brightness}
 *   GET    /api/config/wifi      → {ssid, channel, auto_start}
 *   POST   /api/config/wifi      → set {ssid, pass, channel, auto_start}
 *   GET    /api/config/autopoll  → {enabled}
 *   POST   /api/config/autopoll  → set {enabled}
 *
 * Each endpoint is a focused handler. No LVGL or display operations here —
 * settings changes take effect on next boot or next AP restart.
 */

#include "wifi_internal.h"
#include "wifi_manager.h"
#include "gauge_engine.h"
#include "boot_splash.h"
#include "display_driver.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "http_cfg";

// ============================================================================
// Helpers
// ============================================================================

/** Read POST body into a stack buffer. Returns length or -1 on error. */
static int read_body(httpd_req_t *req, char *buf, size_t buf_size)
{
    int total = req->content_len;
    if (total <= 0 || total >= (int)buf_size) return -1;
    int received = httpd_req_recv(req, buf, total);
    if (received != total) return -1;
    buf[total] = '\0';
    return total;
}

/** Tiny JSON int parser: find "key":123 and return the integer value */
static bool json_get_int(const char *json, const char *key, int *out)
{
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    *out = atoi(p);
    return true;
}

/** Tiny JSON string parser: find "key":"value" and copy to out */
static bool json_get_str(const char *json, const char *key, char *out, size_t out_size)
{
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    const char *end = strchr(p, '"');
    if (!end) return false;
    size_t len = (size_t)(end - p);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

/** Tiny JSON bool parser: find "key":true/false */
static bool json_get_bool(const char *json, const char *key, bool *out)
{
    char search[48];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    *out = (*p == 't');
    return true;
}

// ============================================================================
// GET /api/gauges — all gauge slots
// ============================================================================

static esp_err_t gauges_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr_chunk(req, "{\"slots\":[");

    char item[128];
    for (int i = 0; i < GAUGE_MAX_SLOTS; i++) {
        const gauge_slot_t *slot = gauge_engine_get_slot(i);
        if (!slot) continue;

        int len = snprintf(item, sizeof(item),
            "%s{\"slot\":%d,\"pid_id\":%u,\"base_unit\":%d,"
            "\"display_unit\":%d,\"value\":\"%s\",\"valid\":%s}",
            i > 0 ? "," : "",
            i,
            slot->pid_id,
            slot->base_unit,
            slot->display_unit,
            slot->pid_id != 0xFFFF ? slot->value_str : "",
            slot->value_valid ? "true" : "false");
        httpd_resp_send_chunk(req, item, len);
    }

    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// ============================================================================
// POST /api/gauges — set a gauge slot {slot, pid_index, unit_index}
// ============================================================================

static esp_err_t gauges_set_handler(httpd_req_t *req)
{
    char body[128];
    if (read_body(req, body, sizeof(body)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large or missing");
        return ESP_OK;
    }

    int slot = -1, pid_index = -1, unit_index = 0;
    json_get_int(body, "slot", &slot);
    json_get_int(body, "pid_index", &pid_index);
    json_get_int(body, "unit_index", &unit_index);

    if (slot < 0 || slot >= GAUGE_MAX_SLOTS || pid_index < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid slot or pid_index");
        return ESP_OK;
    }

    esp_err_t ret = gauge_engine_set_pid(slot, pid_index);
    if (ret == ESP_OK && unit_index > 0) {
        gauge_engine_set_unit(slot, unit_index);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (ret == ESP_OK) {
        httpd_resp_sendstr(req, "{\"ok\":true}");
    } else {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"set_pid failed\"}");
    }
    return ESP_OK;
}

// ============================================================================
// DELETE /api/gauges — clear a gauge slot {slot}
// ============================================================================

static esp_err_t gauges_clear_handler(httpd_req_t *req)
{
    char body[64];
    if (read_body(req, body, sizeof(body)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body too large or missing");
        return ESP_OK;
    }

    int slot = -1;
    json_get_int(body, "slot", &slot);
    if (slot < 0 || slot >= GAUGE_MAX_SLOTS) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid slot");
        return ESP_OK;
    }

    gauge_engine_clear_slot(slot);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ============================================================================
// GET /api/gauges/pids — PID dropdown options
// ============================================================================

static esp_err_t gauges_pids_handler(httpd_req_t *req)
{
    char *buf = malloc(4096);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory");
        return ESP_OK;
    }

    int count = gauge_engine_build_pid_options(buf, 4096);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Wrap in JSON: {"count":N,"options":"RPM\nSpeed\n..."}
    httpd_resp_sendstr_chunk(req, "{\"count\":");
    char num[12];
    snprintf(num, sizeof(num), "%d", count);
    httpd_resp_sendstr_chunk(req, num);
    httpd_resp_sendstr_chunk(req, ",\"options\":\"");

    // Escape newlines for JSON
    for (char *p = buf; *p; p++) {
        if (*p == '\n') {
            httpd_resp_sendstr_chunk(req, "\\n");
        } else if (*p == '"') {
            httpd_resp_sendstr_chunk(req, "\\\"");
        } else {
            httpd_resp_send_chunk(req, p, 1);
        }
    }

    httpd_resp_sendstr_chunk(req, "\"}");
    httpd_resp_send_chunk(req, NULL, 0);
    free(buf);
    return ESP_OK;
}

// ============================================================================
// GET /api/gauges/units?slot=N — unit options for a slot
// ============================================================================

static esp_err_t gauges_units_handler(httpd_req_t *req)
{
    // Parse ?slot=N from query string
    char query[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing ?slot=N");
        return ESP_OK;
    }
    char val[8] = {0};
    if (httpd_query_key_value(query, "slot", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing slot parameter");
        return ESP_OK;
    }
    int slot = atoi(val);

    char buf[GAUGE_UNIT_OPTS_LEN];
    int count = gauge_engine_get_unit_options(slot, buf, sizeof(buf));

    char json[256];
    // Escape newlines in options
    char esc[GAUGE_UNIT_OPTS_LEN * 2];
    char *dst = esc;
    for (const char *p = buf; *p && (dst - esc) < (int)sizeof(esc) - 3; p++) {
        if (*p == '\n') { *dst++ = '\\'; *dst++ = 'n'; }
        else { *dst++ = *p; }
    }
    *dst = '\0';

    int len = snprintf(json, sizeof(json),
        "{\"slot\":%d,\"count\":%d,\"options\":\"%s\"}",
        slot, count, esc);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// GET /api/config/splash — splash duration
// ============================================================================

static esp_err_t splash_get_handler(httpd_req_t *req)
{
    uint32_t ms = boot_splash_get_duration();
    char json[48];
    int len = snprintf(json, sizeof(json), "{\"duration_ms\":%lu}", (unsigned long)ms);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// POST /api/config/splash — set splash duration {duration_ms}
// ============================================================================

static esp_err_t splash_set_handler(httpd_req_t *req)
{
    char body[64];
    if (read_body(req, body, sizeof(body)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body error");
        return ESP_OK;
    }

    int ms = -1;
    json_get_int(body, "duration_ms", &ms);
    if (ms < 0 || ms > 30000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "duration_ms 0-30000");
        return ESP_OK;
    }

    boot_splash_set_duration((uint32_t)ms);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    ESP_LOGI(TAG, "Splash duration set to %d ms", ms);
    return ESP_OK;
}

// ============================================================================
// GET /api/config/backlight — brightness
// ============================================================================

static esp_err_t backlight_get_handler(httpd_req_t *req)
{
    uint8_t pct = display_get_brightness();
    char json[32];
    int len = snprintf(json, sizeof(json), "{\"brightness\":%d}", pct);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// POST /api/config/backlight — set brightness {brightness: 0-100}
// ============================================================================

static esp_err_t backlight_set_handler(httpd_req_t *req)
{
    char body[64];
    if (read_body(req, body, sizeof(body)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body error");
        return ESP_OK;
    }

    int pct = -1;
    json_get_int(body, "brightness", &pct);
    if (pct < 0 || pct > 100) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "brightness 0-100");
        return ESP_OK;
    }

    display_set_brightness((uint8_t)pct);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    ESP_LOGI(TAG, "Backlight set to %d%%", pct);
    return ESP_OK;
}

// ============================================================================
// GET /api/config/wifi — WiFi settings
// ============================================================================

static esp_err_t wifi_get_handler(httpd_req_t *req)
{
    char json[128];
    int len = snprintf(json, sizeof(json),
        "{\"ssid\":\"%s\",\"ip\":\"%s\",\"clients\":%d}",
        wifi_manager_get_ssid(),
        wifi_manager_get_ip(),
        wifi_manager_get_client_count());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// POST /api/config/wifi — update WiFi settings (take effect on next AP start)
// ============================================================================

static esp_err_t wifi_set_handler(httpd_req_t *req)
{
    char body[256];
    if (read_body(req, body, sizeof(body)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body error");
        return ESP_OK;
    }

    char ssid[33] = {0};
    char pass[65] = {0};
    int channel = -1;
    bool auto_start = false;

    if (json_get_str(body, "ssid", ssid, sizeof(ssid))) {
        wifi_manager_set_ssid(ssid);
        ESP_LOGI(TAG, "WiFi SSID set: '%s'", ssid);
    }
    if (json_get_str(body, "pass", pass, sizeof(pass))) {
        wifi_manager_set_password(pass);
        ESP_LOGI(TAG, "WiFi password updated");
    }
    if (json_get_int(body, "channel", &channel) && channel >= 1 && channel <= 13) {
        wifi_manager_set_channel((uint8_t)channel);
        ESP_LOGI(TAG, "WiFi channel set: %d", channel);
    }
    if (json_get_bool(body, "auto_start", &auto_start)) {
        wifi_manager_set_auto_start(auto_start);
        ESP_LOGI(TAG, "WiFi auto-start: %s", auto_start ? "on" : "off");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ============================================================================
// GET /api/config/autopoll — auto-poll setting
// ============================================================================

static bool nvs_get_autopoll(void)
{
    nvs_handle_t h;
    uint8_t val = 0;
    if (nvs_open("gauge_cfg", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "autopoll", &val);
        nvs_close(h);
    }
    return val != 0;
}

static void nvs_set_autopoll(bool enabled)
{
    nvs_handle_t h;
    if (nvs_open("gauge_cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "autopoll", enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

static esp_err_t autopoll_get_handler(httpd_req_t *req)
{
    char json[32];
    int len = snprintf(json, sizeof(json), "{\"enabled\":%s}",
                       nvs_get_autopoll() ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

static esp_err_t autopoll_set_handler(httpd_req_t *req)
{
    char body[64];
    if (read_body(req, body, sizeof(body)) < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Body error");
        return ESP_OK;
    }
    bool enabled = false;
    if (json_get_bool(body, "enabled", &enabled)) {
        nvs_set_autopoll(enabled);
        ESP_LOGI(TAG, "Auto-poll set to: %s", enabled ? "on" : "off");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ============================================================================
// Registration
// ============================================================================

static const httpd_uri_t uris[] = {
    { "/api/gauges",            HTTP_GET,    gauges_get_handler,     NULL },
    { "/api/gauges",            HTTP_POST,   gauges_set_handler,     NULL },
    { "/api/gauges",            HTTP_DELETE, gauges_clear_handler,   NULL },
    { "/api/gauges/pids",       HTTP_GET,    gauges_pids_handler,    NULL },
    { "/api/gauges/units",      HTTP_GET,    gauges_units_handler,   NULL },
    { "/api/config/splash",     HTTP_GET,    splash_get_handler,     NULL },
    { "/api/config/splash",     HTTP_POST,   splash_set_handler,     NULL },
    { "/api/config/backlight",  HTTP_GET,    backlight_get_handler,  NULL },
    { "/api/config/backlight",  HTTP_POST,   backlight_set_handler,  NULL },
    { "/api/config/wifi",       HTTP_GET,    wifi_get_handler,       NULL },
    { "/api/config/wifi",       HTTP_POST,   wifi_set_handler,       NULL },
    { "/api/config/autopoll",   HTTP_GET,    autopoll_get_handler,   NULL },
    { "/api/config/autopoll",   HTTP_POST,   autopoll_set_handler,   NULL },
};

esp_err_t config_handler_register(httpd_handle_t server)
{
    esp_err_t ret = ESP_OK;
    int count = sizeof(uris) / sizeof(uris[0]);
    for (int i = 0; i < count; i++) {
        esp_err_t r = httpd_register_uri_handler(server, &uris[i]);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "Failed %s %s: %s", 
                     uris[i].method == HTTP_GET ? "GET" :
                     uris[i].method == HTTP_POST ? "POST" : "DELETE",
                     uris[i].uri, esp_err_to_name(r));
            ret = r;
        }
    }
    ESP_LOGI(TAG, "Registered: %d config endpoints", count);
    return ret;
}
