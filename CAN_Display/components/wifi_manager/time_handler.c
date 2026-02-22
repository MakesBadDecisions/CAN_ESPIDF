/**
 * @file time_handler.c
 * @brief HTTP handlers — POST /api/settime, GET /api/time
 *
 * Time sync endpoint for the captive portal. When a client connects
 * and loads the settings page, JavaScript automatically POSTs the
 * device's epoch + timezone offset. The handler:
 *   1. Sets the ESP32 system clock (settimeofday)
 *   2. Writes the time to the PCF85063 RTC (UTC)
 *   3. Saves the timezone offset to NVS
 *
 * GET /api/time returns the current system time + timezone for display.
 */

#include "wifi_internal.h"
#include "pcf85063.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "http_time";

// Max POST body size (small JSON: {"epoch":1708012345,"tz_offset":-300})
#define MAX_POST_SIZE  128

// ============================================================================
// Simple JSON integer parser (avoids cJSON dependency for 2 fields)
// ============================================================================

/**
 * @brief Extract an integer value for a given key from a JSON string
 * @return true if found and parsed
 */
static bool json_get_int(const char *json, const char *key, long long *out)
{
    // Build search pattern: "key":
    char pattern[48];
    int plen = snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    if (plen <= 0 || plen >= (int)sizeof(pattern)) return false;

    const char *p = strstr(json, pattern);
    if (!p) return false;
    p += plen;

    // Skip whitespace
    while (*p == ' ' || *p == '\t') p++;

    char *end = NULL;
    *out = strtoll(p, &end, 10);
    return (end != p);
}

// ============================================================================
// POST /api/settime — receive epoch + timezone from client browser
// ============================================================================

static esp_err_t settime_post_handler(httpd_req_t *req)
{
    // Read POST body
    int content_len = req->content_len;
    if (content_len <= 0 || content_len > MAX_POST_SIZE) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"invalid content length\"}");
        return ESP_OK;
    }

    char body[MAX_POST_SIZE + 1];
    int received = httpd_req_recv(req, body, content_len);
    if (received <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"recv failed\"}");
        return ESP_OK;
    }
    body[received] = '\0';

    ESP_LOGI(TAG, "POST /api/settime: %s", body);

    // Parse epoch (required)
    long long epoch_val = 0;
    if (!json_get_int(body, "epoch", &epoch_val) || epoch_val < 1577836800) {
        // Must be after Jan 1 2020
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"invalid or missing epoch\"}");
        return ESP_OK;
    }

    // Parse timezone offset (optional, minutes from UTC)
    // JS getTimezoneOffset() returns inverse sign: EST=-300 means UTC-5,
    // but JS returns +300. We negate it here.
    long long tz_offset = 0;
    bool has_tz = json_get_int(body, "tz_offset", &tz_offset);

    time_t epoch_sec = (time_t)epoch_val;

    // 1. Set ESP32 system clock
    struct timeval tv = { .tv_sec = epoch_sec, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    // 2. Write to RTC (if present)
    bool rtc_ok = false;
    if (pcf85063_is_initialized()) {
        struct tm utc;
        gmtime_r(&epoch_sec, &utc);
        esp_err_t ret = pcf85063_set_time(&utc);
        rtc_ok = (ret == ESP_OK);
        if (!rtc_ok) {
            ESP_LOGW(TAG, "RTC write failed: %s", esp_err_to_name(ret));
        }
    }

    // 3. Save timezone offset (negate JS convention)
    int16_t tz_min = 0;
    if (has_tz) {
        tz_min = (int16_t)(-tz_offset);    // JS +300 → UTC-300 = UTC-5h
        pcf85063_save_tz_offset(tz_min);
    }

    // Format current time for the response
    struct tm local_tm;
    time_t local_epoch = epoch_sec + (tz_min * 60);
    gmtime_r(&local_epoch, &local_tm);

    char time_str[32];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &local_tm);

    // Respond
    char resp[200];
    int len = snprintf(resp, sizeof(resp),
        "{\"ok\":true,\"rtc\":%s,\"epoch\":%lld,\"tz_offset\":%d,\"local_time\":\"%s\"}",
        rtc_ok ? "true" : "false",
        (long long)epoch_sec,
        (int)tz_min,
        time_str);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, resp, len);

    ESP_LOGI(TAG, "Time set: epoch=%lld tz=%+dmin local=%s rtc=%s",
             (long long)epoch_sec, tz_min, time_str, rtc_ok ? "OK" : "skip");

    return ESP_OK;
}

// ============================================================================
// GET /api/time — read current system time + RTC status
// ============================================================================

static esp_err_t time_get_handler(httpd_req_t *req)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now = tv.tv_sec;

    int16_t tz_min = pcf85063_get_tz_offset();

    // Format UTC time
    struct tm utc_tm;
    gmtime_r(&now, &utc_tm);
    char utc_str[32];
    strftime(utc_str, sizeof(utc_str), "%Y-%m-%d %H:%M:%S", &utc_tm);

    // Format local time
    time_t local_epoch = now + (tz_min * 60);
    struct tm local_tm;
    gmtime_r(&local_epoch, &local_tm);
    char local_str[32];
    strftime(local_str, sizeof(local_str), "%Y-%m-%d %H:%M:%S", &local_tm);

    // Valid = epoch > Jan 1 2020
    bool valid = (now > 1577836800);

    char resp[300];
    int len = snprintf(resp, sizeof(resp),
        "{\"epoch\":%lld,\"utc\":\"%s\",\"local\":\"%s\","
        "\"tz_offset\":%d,\"rtc\":%s,\"valid\":%s}",
        (long long)now,
        utc_str,
        local_str,
        (int)tz_min,
        pcf85063_is_initialized() ? "true" : "false",
        valid ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, resp, len);
    return ESP_OK;
}

// ============================================================================
// Registration
// ============================================================================

static const httpd_uri_t uri_settime = {
    .uri      = "/api/settime",
    .method   = HTTP_POST,
    .handler  = settime_post_handler,
};

static const httpd_uri_t uri_gettime = {
    .uri      = "/api/time",
    .method   = HTTP_GET,
    .handler  = time_get_handler,
};

esp_err_t time_handler_register(httpd_handle_t server)
{
    esp_err_t ret;

    ret = httpd_register_uri_handler(server, &uri_settime);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed POST /api/settime: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_gettime);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed GET /api/time: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Registered: /api/settime (POST), /api/time (GET)");
    return ESP_OK;
}
