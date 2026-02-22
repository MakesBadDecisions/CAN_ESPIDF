/**
 * @file http_server.c
 * @brief HTTP server lifecycle — start, stop, route registration
 *
 * Wraps esp_http_server. All URI handlers are registered by their
 * respective handler modules (status_handler, etc.) via the
 * server handle passed during registration.
 */

#include "wifi_internal.h"
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "http_srv";

static httpd_handle_t s_server = NULL;

// ============================================================================
// Lifecycle
// ============================================================================

esp_err_t http_server_start(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "HTTP server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size       = 8192;
    config.max_uri_handlers = 32;
    config.lru_purge_enable = true;
    config.uri_match_fn     = httpd_uri_match_wildcard;

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);

    // Register all endpoint handlers (each module owns its own routes)
    status_handler_register(s_server);
    log_handler_register(s_server);
    config_handler_register(s_server);
    pid_handler_register(s_server);
    portal_handler_register(s_server);
    time_handler_register(s_server);
    scan_handler_register(s_server);

    return ESP_OK;
}

esp_err_t http_server_stop(void)
{
    if (!s_server) {
        return ESP_OK;
    }

    esp_err_t ret = httpd_stop(s_server);
    s_server = NULL;

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HTTP server stopped");
    } else {
        ESP_LOGE(TAG, "httpd_stop failed: %s", esp_err_to_name(ret));
    }

    return ret;
}
