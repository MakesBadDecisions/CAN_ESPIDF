/**
 * @file wifi_manager.h
 * @brief WiFi AP & Configuration Server
 *
 * The only WiFi access point in the CAN_ESPIDF system. Provides:
 * - SoftAP with random password (or NVS-persisted override)
 * - QR code display on LCD (two-stage: WiFi QR → URL QR)
 * - HTTP REST API for logs, gauge config, display settings, CAN proxy
 *
 * QR code flow (adapted from ToolTruck OTA):
 *   1. wifi_manager_start() → AP starts, LCD shows WiFi QR code
 *   2. User scans QR → phone auto-connects to AP
 *   3. WIFI_EVENT_AP_STACONNECTED → LCD swaps to URL QR code
 *   4. User scans URL QR → browser opens http://192.168.4.1/
 *   5. User taps "Close" on LCD → wifi_manager_stop()
 *
 * Password behavior:
 *   - Default: random 8-char password generated each AP start
 *   - If NVS "pass" key is non-empty: use that persistent password
 *   - Password always displayed as plain text below QR code on LCD
 *
 * SSID behavior:
 *   - Default: "CAN_<4hex>" derived from ESP32 MAC address
 *   - If NVS "ssid" key is non-empty: use that persistent SSID
 *
 * @see README.md for full architecture, endpoint table, and QR code design
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* ── Lifecycle ─────────────────────────────────────────── */

/**
 * @brief Initialize WiFi subsystem, load NVS settings.
 *
 * Initializes esp_netif, WiFi driver (APSTA mode), and loads
 * NVS settings (ssid, pass, channel, ap_on). Does NOT start
 * the AP unless the NVS "ap_on" key is set.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Start WiFi AP and HTTP server, show QR screen on LCD.
 *
 * Generates a random password (unless NVS override exists),
 * starts the SoftAP, starts the HTTP server, and renders the
 * WiFi QR code screen on the LCD.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief Stop HTTP server, WiFi AP, and free QR screen.
 *
 * Tears down the HTTP server, stops the AP, frees the QR code
 * LVGL objects, and returns to the gauge UI.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_stop(void);

/**
 * @brief Check if the AP is currently running.
 * @return true if AP is active and serving
 */
bool wifi_manager_is_running(void);

/* ── Client Tracking ───────────────────────────────────── */

/**
 * @brief Get the number of clients currently connected to the AP.
 *
 * Updated by the WiFi event handler on STACONNECTED/STADISCONNECTED.
 *
 * @return Number of connected clients (0 if AP not running)
 */
int wifi_manager_get_client_count(void);

/* ── Credentials (valid while AP is running) ───────────── */

/**
 * @brief Get the current AP SSID.
 * @return SSID string (MAC-derived default or NVS override)
 */
const char *wifi_manager_get_ssid(void);

/**
 * @brief Get the current AP password.
 * @return Password string (random or NVS override)
 */
const char *wifi_manager_get_password(void);

/**
 * @brief Get the AP IP address.
 * @return IP address string (always "192.168.4.1")
 */
const char *wifi_manager_get_ip(void);

/* ── QR Code Data ──────────────────────────────────────── */

/**
 * @brief Fill buffer with WiFi QR URI string.
 *
 * Format: "WIFI:T:WPA;S:<ssid>;P:<password>;;"
 * This is the standard WiFi QR format that phones auto-detect.
 *
 * @param buffer    Output buffer
 * @param buf_size  Buffer size in bytes
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer too small
 */
esp_err_t wifi_manager_get_wifi_qr_data(char *buffer, size_t buf_size);

/**
 * @brief Fill buffer with portal URL QR data.
 *
 * Format: "http://192.168.4.1/"
 * Shown on LCD after a client connects (stage 2 of QR flow).
 *
 * @param buffer    Output buffer
 * @param buf_size  Buffer size in bytes
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer too small
 */
esp_err_t wifi_manager_get_url_qr_data(char *buffer, size_t buf_size);

/* ── Settings (NVS-backed) ─────────────────────────────── */

/**
 * @brief Override the default MAC-derived SSID.
 *
 * Pass empty string to revert to MAC-derived SSID.
 * Takes effect on next wifi_manager_start().
 *
 * @param ssid  New SSID (1-32 chars) or "" for default
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_set_ssid(const char *ssid);

/**
 * @brief Override random password with a persistent one.
 *
 * Pass empty string to revert to random password each start.
 * Takes effect on next wifi_manager_start().
 *
 * @param pass  New password (8-63 chars) or "" for random
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_set_password(const char *pass);

/**
 * @brief Set WiFi channel.
 * @param channel  Channel number (1-13)
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_set_channel(uint8_t channel);

/**
 * @brief Enable or disable auto-start AP on boot.
 * @param enable  true = start AP during wifi_manager_init()
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_set_auto_start(bool enable);

