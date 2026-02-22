/**
 * @file wifi_ap.c
 * @brief WiFi SoftAP — init, start, stop, client tracking
 *
 * Handles ESP-IDF WiFi driver initialization in APSTA mode,
 * SoftAP configuration, random password generation, and
 * client connect/disconnect event tracking.
 *
 * Adapted from ToolTruck OTA WiFi AP pattern.
 */

#include "wifi_internal.h"
#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "wifi_ap";

// ============================================================================
// Internal State
// ============================================================================

static esp_netif_t *s_ap_netif = NULL;
static esp_event_handler_instance_t s_wifi_event_handle = NULL;
static volatile int s_client_count = 0;
static bool s_ap_running = false;

// Callback to notify wifi_manager when a client connects (for QR stage swap)
static void (*s_on_client_connected)(void) = NULL;

// ============================================================================
// WiFi Event Handler
// ============================================================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base != WIFI_EVENT) return;

    switch (event_id) {
        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *ev =
                (wifi_event_ap_staconnected_t *)event_data;
            s_client_count++;
            ESP_LOGI(TAG, "Client connected (MAC: " MACSTR ", AID: %d, total: %d)",
                     MAC2STR(ev->mac), ev->aid, s_client_count);

            // Notify wifi_manager on first client for QR stage swap
            if (s_client_count == 1 && s_on_client_connected) {
                s_on_client_connected();
            }
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *ev =
                (wifi_event_ap_stadisconnected_t *)event_data;
            if (s_client_count > 0) s_client_count--;
            ESP_LOGI(TAG, "Client disconnected (MAC: " MACSTR ", AID: %d, total: %d)",
                     MAC2STR(ev->mac), ev->aid, s_client_count);
            break;
        }
        default:
            break;
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t wifi_ap_init(void)
{
    // Initialize TCP/IP stack (idempotent)
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create default event loop (idempotent)
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Event loop create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create AP netif (only once)
    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            ESP_LOGE(TAG, "Failed to create AP netif");
            return ESP_FAIL;
        }
    }

    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set APSTA mode (AP + STA for future coexistence)
    ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set APSTA mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi AP subsystem initialized (APSTA mode)");
    return ESP_OK;
}

esp_err_t wifi_ap_start(const char *ssid, const char *password, uint8_t channel)
{
    if (s_ap_running) {
        ESP_LOGW(TAG, "AP already running");
        return ESP_OK;
    }

    // Register WiFi event handler
    esp_err_t ret = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        wifi_event_handler, NULL, &s_wifi_event_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Event handler register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure AP
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid) - 1);
    strncpy((char *)wifi_config.ap.password, password, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.ssid_len     = strlen(ssid);
    wifi_config.ap.channel      = channel;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode     = WIFI_AUTH_WPA2_PSK;

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AP config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_client_count = 0;
    s_ap_running = true;

    ESP_LOGI(TAG, "SoftAP started: SSID=%s PASS=%s CH=%d IP=192.168.4.1",
             ssid, password, channel);
    return ESP_OK;
}

esp_err_t wifi_ap_stop(void)
{
    if (!s_ap_running) {
        return ESP_OK;
    }

    esp_wifi_stop();

    if (s_wifi_event_handle) {
        esp_event_handler_instance_unregister(
            WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_event_handle);
        s_wifi_event_handle = NULL;
    }

    s_client_count = 0;
    s_ap_running = false;
    s_on_client_connected = NULL;

    ESP_LOGI(TAG, "SoftAP stopped");
    return ESP_OK;
}

void wifi_ap_generate_password(char *buf, size_t buf_size)
{
    // Unambiguous charset: no 0/O/o, 1/l/I
    static const char charset[] =
        "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghjkmnpqrstuvwxyz23456789";
    const int len = 8;

    if (buf_size < (size_t)(len + 1)) return;

    for (int i = 0; i < len; i++) {
        buf[i] = charset[esp_random() % (sizeof(charset) - 1)];
    }
    buf[len] = '\0';
}

void wifi_ap_generate_ssid(char *buf, size_t buf_size)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(buf, buf_size, "CAN_%02X%02X", mac[4], mac[5]);
}

int wifi_ap_get_client_count(void)
{
    return s_client_count;
}

// Called by wifi_manager to set the client-connected callback
void wifi_ap_set_on_client_connected(void (*cb)(void))
{
    s_on_client_connected = cb;
}
