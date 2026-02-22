/**
 * @file log_handler.c
 * @brief HTTP handlers — log file list, download, delete, SD stats
 *
 * Endpoints:
 *   GET    /api/logs          → JSON array of log files [{name, size}]
 *   GET    /api/logs/<file>   → chunked file download
 *   DELETE /api/logs/<file>   → delete file
 *   GET    /api/sd            → JSON SD card stats {total, free, mounted}
 *
 * All file operations use LOGGER_LOG_DIR ("/sdcard/logs").
 * Downloads use 4KB chunked transfer to avoid buffering large files.
 */

#include "wifi_internal.h"
#include "data_logger.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "http_log";

// ============================================================================
// Helpers
// ============================================================================

/** Extract filename from URI after /api/logs/ prefix */
static const char *uri_to_filename(const char *uri)
{
    const char *prefix = "/api/logs/";
    size_t plen = strlen(prefix);
    if (strncmp(uri, prefix, plen) == 0 && uri[plen] != '\0') {
        return &uri[plen];
    }
    return NULL;
}

/** Validate filename: must be alphanumeric + dots + underscores, no path traversal */
static bool is_safe_filename(const char *name)
{
    if (!name || name[0] == '\0' || name[0] == '.') return false;
    if (strstr(name, "..") != NULL) return false;
    if (strchr(name, '/') != NULL || strchr(name, '\\') != NULL) return false;
    return true;
}

// ============================================================================
// GET /api/logs — list all log files
// ============================================================================

static esp_err_t logs_list_handler(httpd_req_t *req)
{
    if (!logger_is_sd_mounted()) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"error\":\"SD card not mounted\"}", -1);
        return ESP_OK;
    }

    DIR *dir = opendir(LOGGER_LOG_DIR);
    if (!dir) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"error\":\"Cannot open log directory\"}", -1);
        return ESP_OK;
    }

    // Build JSON array of log files
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr_chunk(req, "[");

    struct dirent *entry;
    struct stat st;
    char path[280];
    bool first = true;
    char item[192];

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type != DT_REG) continue;

        snprintf(path, sizeof(path), "%s/%s", LOGGER_LOG_DIR, entry->d_name);
        if (stat(path, &st) != 0) continue;

        int len = snprintf(item, sizeof(item),
            "%s{\"name\":\"%s\",\"size\":%ld}",
            first ? "" : ",",
            entry->d_name,
            (long)st.st_size);
        httpd_resp_send_chunk(req, item, len);
        first = false;
    }
    closedir(dir);

    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_send_chunk(req, NULL, 0);  // End chunked response
    return ESP_OK;
}

// ============================================================================
// GET /api/logs/<filename> — download a log file (chunked)
// ============================================================================

static esp_err_t logs_download_handler(httpd_req_t *req)
{
    const char *filename = uri_to_filename(req->uri);
    if (!filename || !is_safe_filename(filename)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_OK;
    }

    char path[280];
    snprintf(path, sizeof(path), "%s/%s", LOGGER_LOG_DIR, filename);

    FILE *f = fopen(path, "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Content-Disposition header for download
    char disp[128];
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", filename);
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    // Stream file in 4KB chunks
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            ESP_LOGW(TAG, "Download aborted: %s", filename);
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);

    httpd_resp_send_chunk(req, NULL, 0);  // End chunked response
    ESP_LOGI(TAG, "Downloaded: %s", filename);
    return ESP_OK;
}

// ============================================================================
// DELETE /api/logs/<filename> — delete a log file
// ============================================================================

static esp_err_t logs_delete_handler(httpd_req_t *req)
{
    const char *filename = uri_to_filename(req->uri);
    if (!filename || !is_safe_filename(filename)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_OK;
    }

    char path[280];
    snprintf(path, sizeof(path), "%s/%s", LOGGER_LOG_DIR, filename);

    if (unlink(path) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    ESP_LOGI(TAG, "Deleted: %s", filename);
    return ESP_OK;
}

// ============================================================================
// GET /api/sd — SD card stats
// ============================================================================

static esp_err_t sd_stats_handler(httpd_req_t *req)
{
    logger_status_t st = logger_get_status();
    char json[256];
    int len = snprintf(json, sizeof(json),
        "{"   
        "\"mounted\":%s,"
        "\"total_bytes\":%llu,"
        "\"free_bytes\":%llu,"
        "\"low_space\":%s,"
        "\"file_name\":\"%s\""
        "}",
        logger_is_sd_mounted() ? "true" : "false",
        (unsigned long long)st.sd_total_bytes,
        (unsigned long long)st.sd_free_bytes,
        st.sd_low_space ? "true" : "false",
        st.file_name);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json, len);
    return ESP_OK;
}

// ============================================================================
// Registration
// ============================================================================

static const httpd_uri_t uri_logs_list = {
    .uri      = "/api/logs",
    .method   = HTTP_GET,
    .handler  = logs_list_handler,
};

/* Wildcard URI — matches /api/logs/<anything> for GET */
static const httpd_uri_t uri_logs_download = {
    .uri      = "/api/logs/*",
    .method   = HTTP_GET,
    .handler  = logs_download_handler,
};

static const httpd_uri_t uri_logs_delete = {
    .uri      = "/api/logs/*",
    .method   = HTTP_DELETE,
    .handler  = logs_delete_handler,
};

static const httpd_uri_t uri_sd_stats = {
    .uri      = "/api/sd",
    .method   = HTTP_GET,
    .handler  = sd_stats_handler,
};

esp_err_t log_handler_register(httpd_handle_t server)
{
    esp_err_t ret;

    ret = httpd_register_uri_handler(server, &uri_logs_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed /api/logs: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_logs_download);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed /api/logs/*: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_logs_delete);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed DELETE /api/logs/*: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = httpd_register_uri_handler(server, &uri_sd_stats);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed /api/sd: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Registered: log & SD endpoints");
    return ESP_OK;
}
