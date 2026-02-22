/**
 * @file wifi_manager.c
 * @brief WiFi Manager — lifecycle, NVS settings, QR/HTTP orchestration
 *
 * Top-level coordinator for WiFi AP, QR code display, and HTTP server.
 * All sub-modules (wifi_ap, qr_screen, http_server) are private —
 * consumers interact only through the wifi_manager.h public API.
 */

#include "wifi_manager.h"
#include "wifi_internal.h"
#include "display_driver.h"
#include "system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "wifi_mgr";

// ============================================================================
// NVS Keys
// ============================================================================

#define WIFI_NVS_NAMESPACE  "wifi"
#define WIFI_NVS_SSID       "ssid"
#define WIFI_NVS_PASS       "pass"
#define WIFI_NVS_CHANNEL    "channel"
#define WIFI_NVS_AUTO_START "ap_on"

// ============================================================================
// Internal State
// ============================================================================

static struct {
    bool        initialized;
    bool        running;
    char        ssid[33];           // Current SSID (MAC-derived or NVS)
    char        password[17];       // Current password (random or NVS)
    uint8_t     channel;            // WiFi channel
    bool        auto_start;         // Start AP on boot
    char        ssid_override[33];  // NVS SSID override (empty = default)
    char        pass_override[65];  // NVS pass override (empty = random)
} s_wifi;

// ============================================================================
// NVS Load / Save Helpers
// ============================================================================

static void load_nvs_settings(void)
{
    nvs_handle_t nvs;
    if (nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t len;

        len = sizeof(s_wifi.ssid_override);
        if (nvs_get_str(nvs, WIFI_NVS_SSID, s_wifi.ssid_override, &len) != ESP_OK) {
            s_wifi.ssid_override[0] = '\0';
        }

        len = sizeof(s_wifi.pass_override);
        if (nvs_get_str(nvs, WIFI_NVS_PASS, s_wifi.pass_override, &len) != ESP_OK) {
            s_wifi.pass_override[0] = '\0';
        }

        if (nvs_get_u8(nvs, WIFI_NVS_CHANNEL, &s_wifi.channel) != ESP_OK) {
            s_wifi.channel = 1;
        }

        uint8_t auto_on = 0;
        if (nvs_get_u8(nvs, WIFI_NVS_AUTO_START, &auto_on) == ESP_OK) {
            s_wifi.auto_start = (auto_on != 0);
        }

        nvs_close(nvs);
        ESP_LOGI(TAG, "NVS loaded: ssid_ovr='%s' pass_ovr=%s ch=%d auto=%d",
                 s_wifi.ssid_override,
                 s_wifi.pass_override[0] ? "(set)" : "(random)",
                 s_wifi.channel, s_wifi.auto_start);
    } else {
        s_wifi.channel = 1;
        s_wifi.auto_start = false;
    }
}

static esp_err_t save_nvs_str(const char *key, const char *value)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_str(nvs, key, value);
    if (ret == ESP_OK) nvs_commit(nvs);
    nvs_close(nvs);
    return ret;
}

static esp_err_t save_nvs_u8(const char *key, uint8_t value)
{
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_u8(nvs, key, value);
    if (ret == ESP_OK) nvs_commit(nvs);
    nvs_close(nvs);
    return ret;
}

// ============================================================================
// Client Connected Callback (from wifi_ap.c — triggers QR stage swap)
// ============================================================================

static void on_client_connected(void)
{
    ESP_LOGI(TAG, "First client connected — swapping to URL QR");

    if (display_lock(200)) {
        char url[64];
        wifi_manager_get_url_qr_data(url, sizeof(url));
        qr_screen_show_url(url);
        qr_screen_update_clients(wifi_ap_get_client_count());
        display_unlock();
    }
}

// ============================================================================
// Lifecycle API
// ============================================================================

esp_err_t wifi_manager_init(void)
{
    if (s_wifi.initialized) {
        return ESP_OK;
    }

    memset(&s_wifi, 0, sizeof(s_wifi));

    // Load NVS settings
    load_nvs_settings();

    // Initialize WiFi AP subsystem
    esp_err_t ret = wifi_ap_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wifi_ap_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_wifi.initialized = true;
    ESP_LOGI(TAG, "WiFi Manager initialized");

    // Auto-start if NVS flag set
    if (s_wifi.auto_start) {
        ESP_LOGI(TAG, "Auto-starting AP (NVS ap_on=1)");
        return wifi_manager_start();
    }

    return ESP_OK;
}

esp_err_t wifi_manager_start(void)
{
    if (!s_wifi.initialized) {
        ESP_LOGE(TAG, "Not initialized — call wifi_manager_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_wifi.running) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    // Determine SSID
    if (s_wifi.ssid_override[0]) {
        strncpy(s_wifi.ssid, s_wifi.ssid_override, sizeof(s_wifi.ssid) - 1);
    } else {
        wifi_ap_generate_ssid(s_wifi.ssid, sizeof(s_wifi.ssid));
    }

    // Determine password
    if (s_wifi.pass_override[0]) {
        strncpy(s_wifi.password, s_wifi.pass_override, sizeof(s_wifi.password) - 1);
    } else {
        wifi_ap_generate_password(s_wifi.password, sizeof(s_wifi.password));
    }

    ESP_LOGI(TAG, "Starting WiFi AP: SSID=%s PASS=%s CH=%d",
             s_wifi.ssid, s_wifi.password, s_wifi.channel);

    // Set client-connected callback for QR stage swap
    wifi_ap_set_on_client_connected(on_client_connected);

    // Start SoftAP
    esp_err_t ret = wifi_ap_start(s_wifi.ssid, s_wifi.password, s_wifi.channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "wifi_ap_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start HTTP server
    ret = http_server_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "http_server_start failed: %s", esp_err_to_name(ret));
        wifi_ap_stop();
        return ret;
    }

    // Show QR code screen on LCD
    if (display_lock(1000)) {
        qr_screen_show(s_wifi.ssid, s_wifi.password);
        display_unlock();
    } else {
        ESP_LOGW(TAG, "Could not acquire display lock for QR screen");
    }

    s_wifi.running = true;
    ESP_LOGI(TAG, "WiFi Manager started successfully");
    return ESP_OK;
}

esp_err_t wifi_manager_stop(void)
{
    if (!s_wifi.running) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping WiFi Manager...");

    // Hide QR screen first (return to gauge UI)
    if (display_lock(1000)) {
        qr_screen_hide();
        display_unlock();
    }

    // Stop HTTP server
    http_server_stop();

    // Stop WiFi AP
    wifi_ap_stop();

    s_wifi.running = false;
    ESP_LOGI(TAG, "WiFi Manager stopped");
    return ESP_OK;
}

bool wifi_manager_is_running(void)
{
    return s_wifi.running;
}

// ============================================================================
// Client Tracking
// ============================================================================

int wifi_manager_get_client_count(void)
{
    return wifi_ap_get_client_count();
}

// ============================================================================
// Credentials
// ============================================================================

const char *wifi_manager_get_ssid(void)
{
    return s_wifi.ssid;
}

const char *wifi_manager_get_password(void)
{
    return s_wifi.password;
}

const char *wifi_manager_get_ip(void)
{
    return "192.168.4.1";
}

// ============================================================================
// QR Data
// ============================================================================

esp_err_t wifi_manager_get_wifi_qr_data(char *buffer, size_t buf_size)
{
    int needed = snprintf(buffer, buf_size, "WIFI:T:WPA;S:%s;P:%s;;",
                          s_wifi.ssid, s_wifi.password);
    return (needed < (int)buf_size) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

esp_err_t wifi_manager_get_url_qr_data(char *buffer, size_t buf_size)
{
    int needed = snprintf(buffer, buf_size, "http://%s/", wifi_manager_get_ip());
    return (needed < (int)buf_size) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

// ============================================================================
// Settings (NVS-backed)
// ============================================================================

esp_err_t wifi_manager_set_ssid(const char *ssid)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;
    strncpy(s_wifi.ssid_override, ssid, sizeof(s_wifi.ssid_override) - 1);
    return save_nvs_str(WIFI_NVS_SSID, ssid);
}

esp_err_t wifi_manager_set_password(const char *pass)
{
    if (!pass) return ESP_ERR_INVALID_ARG;
    strncpy(s_wifi.pass_override, pass, sizeof(s_wifi.pass_override) - 1);
    return save_nvs_str(WIFI_NVS_PASS, pass);
}

esp_err_t wifi_manager_set_channel(uint8_t channel)
{
    if (channel < 1 || channel > 13) return ESP_ERR_INVALID_ARG;
    s_wifi.channel = channel;
    return save_nvs_u8(WIFI_NVS_CHANNEL, channel);
}

esp_err_t wifi_manager_set_auto_start(bool enable)
{
    s_wifi.auto_start = enable;
    return save_nvs_u8(WIFI_NVS_AUTO_START, enable ? 1 : 0);
}