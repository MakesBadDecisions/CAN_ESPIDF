/**
 * @file status_handler.c
 * @brief HTTP handler — GET /api/status (JSON system status)
 *
 * Pure JSON API. The HTML portal is served by portal_handler.c.
 */

#include "wifi_internal.h"
#include "wifi_manager.h"
#include "comm_link.h"
#include "data_logger.h"
#include "display_driver.h"
#include "system.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "http_stat";

// ============================================================================
// GET /api/status — JSON system status
// ============================================================================

static esp_err_t status_get_handler(httpd_req_t *req)
{
    uint32_t heap_free    = esp_get_free_heap_size();
    uint32_t heap_min     = esp_get_minimum_free_heap_size();
    uint32_t heap_int     = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t heap_psram   = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    int64_t  uptime_us    = esp_timer_get_time();
    uint32_t uptime_sec   = (uint32_t)(uptime_us / 1000000);

    comm_link_state_t link = comm_link_get_state();
    const char *link_str;
    switch (link) {
        case COMM_LINK_CONNECTED:    link_str = "connected";    break;
        case COMM_LINK_ERROR:        link_str = "error";        break;
        default:                     link_str = "disconnected"; break;
    }

    const comm_link_stats_t *stats = comm_link_get_stats();

    bool sd_mounted              = logger_is_sd_mounted();
    logger_status_t log_status   = logger_get_status();
    const char *logger_str;
    switch (log_status.state) {
        case LOGGER_STATE_READY:   logger_str = "ready";   break;
        case LOGGER_STATE_LOGGING: logger_str = "logging"; break;
        case LOGGER_STATE_ERROR:   logger_str = "error";   break;
        default:                   logger_str = "uninit";  break;
    }

    int clients     = wifi_manager_get_client_count();
    uint8_t bl_pct  = display_get_brightness();

    char json[640];
    int len = snprintf(json, sizeof(json),
        "{"
        "\"heap_free\":%lu,"
        "\"heap_min\":%lu,"
        "\"heap_internal\":%lu,"
        "\"heap_psram\":%lu,"
        "\"uptime_s\":%lu,"
        "\"link\":\"%s\","
        "\"rx_frames\":%lu,"
        "\"tx_frames\":%lu,"
        "\"pid_updates\":%lu,"
        "\"rx_errors\":%lu,"
        "\"sd_mounted\":%s,"
        "\"logger\":\"%s\","
        "\"log_rows\":%lu,"
        "\"wifi_clients\":%d,"
        "\"ssid\":\"%s\","
        "\"ip\":\"%s\","
        "\"brightness\":%d"
        "}",
        (unsigned long)heap_free,
        (unsigned long)heap_min,
        (unsigned long)heap_int,
        (unsigned long)heap_psram,
        (unsigned long)uptime_sec,
        link_str,
        (unsigned long)stats->rx_frames,
        (unsigned long)stats->tx_frames,
        (unsigned long)stats->pid_updates,
        (unsigned long)stats->rx_errors,
        sd_mounted ? "true" : "false",
        logger_str,
        (unsigned long)log_status.rows_written,
        clients,
        wifi_manager_get_ssid(),
        wifi_manager_get_ip(),
        bl_pct);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// Registration
// ============================================================================

static const httpd_uri_t uri_status = {
    .uri      = "/api/status",
    .method   = HTTP_GET,
    .handler  = status_get_handler,
};

esp_err_t status_handler_register(httpd_handle_t server)
{
    esp_err_t ret = httpd_register_uri_handler(server, &uri_status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register /api/status: %s", esp_err_to_name(ret));
    }
    return ret;
}
