/**
 * @file wifi_internal.h
 * @brief WiFi Manager internal shared state
 *
 * Private header shared between wifi_manager sub-modules.
 * NOT part of the public API — consumers use wifi_manager.h only.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"

/* ── WiFi AP (wifi_ap.c) ──────────────────────────────── */

/// Initialize WiFi driver in APSTA mode. Call once from wifi_manager_init().
esp_err_t wifi_ap_init(void);

/// Start SoftAP with current SSID/password. Registers WiFi event handler.
esp_err_t wifi_ap_start(const char *ssid, const char *password, uint8_t channel);

/// Stop SoftAP, unregister events.
esp_err_t wifi_ap_stop(void);

/// Generate a random 8-char password (unambiguous charset).
void wifi_ap_generate_password(char *buf, size_t buf_size);

/// Generate MAC-derived SSID: "CAN_XXXX"
void wifi_ap_generate_ssid(char *buf, size_t buf_size);

/// Get current connected client count.
int wifi_ap_get_client_count(void);

/// Set callback invoked when first client connects (for QR stage swap).
void wifi_ap_set_on_client_connected(void (*cb)(void));

/* ── QR Screen (qr_screen.c) ─────────────────────────── */

/// Create and show the QR code screen (WiFi stage).
/// Must be called under display_lock().
void qr_screen_show(const char *ssid, const char *password);

/// Swap QR to URL stage (called when first client connects).
/// Must be called under display_lock().
void qr_screen_show_url(const char *url);

/// Update client count text on QR screen.
/// Must be called under display_lock().
void qr_screen_update_clients(int count);

/// Delete QR screen and return to previous screen.
/// Must be called under display_lock().
void qr_screen_hide(void);

/// Check if QR screen is currently active.
bool qr_screen_is_active(void);

/* ── HTTP Server (http_server.c) ──────────────────────── */

/// Start HTTP server, register all URI handlers.
esp_err_t http_server_start(void);

/// Stop HTTP server.
esp_err_t http_server_stop(void);

/* ── Status Handler (status_handler.c) ────────────────── */

/// Register GET /api/status on the given server.
esp_err_t status_handler_register(httpd_handle_t server);

/* ── Log Handler (log_handler.c) ──────────────────────── */

/// Register /api/logs and /api/sd endpoints.
esp_err_t log_handler_register(httpd_handle_t server);

/* ── Config Handler (config_handler.c) ────────────────── */

/// Register /api/gauges/*, /api/config/* endpoints.
esp_err_t config_handler_register(httpd_handle_t server);

/* ── PID Handler (pid_handler.c) ───────────────────────── */

/// Register /api/pids and /api/pids/poll endpoints.
esp_err_t pid_handler_register(httpd_handle_t server);

/* ── Portal Handler (portal_handler.c) ────────────────── */

/// Register GET / (HTML SPA) and GET /favicon.ico.
esp_err_t portal_handler_register(httpd_handle_t server);

/* ── Time Handler (time_handler.c) ────────────────────── */

/// Register POST /api/settime and GET /api/time endpoints.
esp_err_t time_handler_register(httpd_handle_t server);

/* ── Scan Handler (scan_handler.c) ────────────────────── */

/// Register POST /api/can/scan and GET /api/can/vehicle endpoints.
esp_err_t scan_handler_register(httpd_handle_t server);
