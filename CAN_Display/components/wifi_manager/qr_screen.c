/**
 * @file qr_screen.c
 * @brief LVGL QR Code Screen — two-stage WiFi/URL display
 *
 * Creates a dedicated LVGL screen (overlays gauge UI) showing:
 *   Stage 1: WiFi QR code + SSID/password text
 *   Stage 2: URL QR code + portal address (after client connects)
 *
 * The screen includes a Close button that calls wifi_manager_stop().
 * Adapted from ToolTruck OTA ui_ota_popup.c QR layout.
 */

#include "wifi_internal.h"
#include "wifi_manager.h"
#include "display_driver.h"
#include "lvgl.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "qr_screen";

// ============================================================================
// Internal State
// ============================================================================

static lv_obj_t *s_qr_screen     = NULL;   // Dedicated LVGL screen
static lv_obj_t *s_qr_code       = NULL;   // QR code widget
static lv_obj_t *s_title_label   = NULL;   // "WiFi Setup" / "Config Portal"
static lv_obj_t *s_info_label    = NULL;   // SSID/pass or URL text
static lv_obj_t *s_instr_label   = NULL;   // Instruction text
static lv_obj_t *s_client_label  = NULL;   // Client count
static lv_obj_t *s_close_btn     = NULL;   // Close button
static lv_obj_t *s_prev_screen   = NULL;   // Screen to return to

// ============================================================================
// Close Button Callback
// ============================================================================

static void close_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    ESP_LOGI(TAG, "Close button pressed — stopping WiFi");
    wifi_manager_stop();
}

// ============================================================================
// QR Screen Show (Stage 1 — WiFi QR)
// ============================================================================

void qr_screen_show(const char *ssid, const char *password)
{
    if (s_qr_screen) {
        ESP_LOGW(TAG, "QR screen already active");
        return;
    }

    // Remember current screen for return
    s_prev_screen = lv_scr_act();

    // Get display dimensions for sizing
    lv_coord_t scr_w = lv_disp_get_hor_res(NULL);
    lv_coord_t scr_h = lv_disp_get_ver_res(NULL);

    // QR code size — fit nicely on screen
    lv_coord_t qr_size = (scr_h < scr_w) ? (scr_h / 3) : (scr_w / 3);
    if (qr_size > 200) qr_size = 200;
    if (qr_size < 100) qr_size = 100;

    // Create new screen
    s_qr_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_qr_screen, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(s_qr_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_qr_screen, LV_OBJ_FLAG_SCROLLABLE);

    // Main container — flex column, centered
    lv_obj_t *container = lv_obj_create(s_qr_screen);
    lv_obj_set_size(container, LV_PCT(90), LV_PCT(90));
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0x3498DB), 0);
    lv_obj_set_style_border_width(container, 2, 0);
    lv_obj_set_style_radius(container, 8, 0);
    lv_obj_set_style_pad_all(container, 10, 0);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Title
    s_title_label = lv_label_create(container);
    lv_label_set_text(s_title_label, "WiFi Setup");
    lv_obj_set_style_text_color(s_title_label, lv_color_hex(0x3498DB), 0);
    lv_obj_set_style_text_font(s_title_label, &lv_font_montserrat_14, 0);

    // QR code container (white background for contrast)
    lv_obj_t *qr_container = lv_obj_create(container);
    lv_obj_set_size(qr_container, qr_size + 16, qr_size + 16);
    lv_obj_set_style_bg_color(qr_container, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(qr_container, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(qr_container, 6, 0);
    lv_obj_set_style_pad_all(qr_container, 8, 0);
    lv_obj_set_style_border_width(qr_container, 0, 0);
    lv_obj_clear_flag(qr_container, LV_OBJ_FLAG_SCROLLABLE);

#if LV_USE_QRCODE
    // Create QR code widget
    s_qr_code = lv_qrcode_create(qr_container, qr_size,
                                  lv_color_black(), lv_color_white());
    lv_obj_align(s_qr_code, LV_ALIGN_CENTER, 0, 0);

    // Encode WiFi QR data
    char qr_data[128];
    snprintf(qr_data, sizeof(qr_data), "WIFI:T:WPA;S:%s;P:%s;;", ssid, password);
    lv_qrcode_update(s_qr_code, qr_data, strlen(qr_data));
    ESP_LOGI(TAG, "WiFi QR: %s", qr_data);
#else
    lv_obj_t *placeholder = lv_label_create(qr_container);
    lv_label_set_text(placeholder, "QR\nDisabled");
    lv_obj_set_style_text_align(placeholder, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(placeholder, LV_ALIGN_CENTER, 0, 0);
    ESP_LOGW(TAG, "LV_USE_QRCODE not enabled!");
#endif

    // SSID/Password info
    s_info_label = lv_label_create(container);
    char info_text[96];
    snprintf(info_text, sizeof(info_text), "Network: %s\nPassword: %s", ssid, password);
    lv_label_set_text(s_info_label, info_text);
    lv_obj_set_style_text_color(s_info_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_info_label, LV_TEXT_ALIGN_CENTER, 0);

    // Instruction
    s_instr_label = lv_label_create(container);
    lv_label_set_text(s_instr_label, "Scan QR code to connect");
    lv_obj_set_style_text_color(s_instr_label, lv_color_hex(0x888888), 0);

    // Client count (hidden initially)
    s_client_label = lv_label_create(container);
    lv_label_set_text(s_client_label, "");
    lv_obj_set_style_text_color(s_client_label, lv_color_hex(0x00FF00), 0);

    // Close button
    s_close_btn = lv_btn_create(container);
    lv_obj_set_size(s_close_btn, 100, 36);
    lv_obj_set_style_bg_color(s_close_btn, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_color(s_close_btn, lv_color_hex(0x3498DB), 0);
    lv_obj_set_style_border_width(s_close_btn, 1, 0);
    lv_obj_set_style_radius(s_close_btn, 6, 0);
    lv_obj_add_event_cb(s_close_btn, close_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(s_close_btn);
    lv_label_set_text(btn_label, "Close");
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
    lv_obj_center(btn_label);

    // Load the QR screen
    lv_disp_load_scr(s_qr_screen);

    ESP_LOGI(TAG, "QR screen shown (WiFi stage) — %dx%d, QR=%d",
             scr_w, scr_h, qr_size);
}

// ============================================================================
// QR Screen Stage 2 — URL QR (after client connects)
// ============================================================================

void qr_screen_show_url(const char *url)
{
    if (!s_qr_screen) return;

    // Update title
    if (s_title_label) {
        lv_label_set_text(s_title_label, "Config Portal");
    }

    // Update QR code to URL
#if LV_USE_QRCODE
    if (s_qr_code) {
        lv_qrcode_update(s_qr_code, url, strlen(url));
        ESP_LOGI(TAG, "URL QR: %s", url);
    }
#endif

    // Update info label
    if (s_info_label) {
        char info_text[96];
        snprintf(info_text, sizeof(info_text), "Portal: %s", url);
        lv_label_set_text(s_info_label, info_text);
    }

    // Update instruction
    if (s_instr_label) {
        lv_label_set_text(s_instr_label, "Scan QR to open portal");
    }

    ESP_LOGI(TAG, "QR screen swapped to URL stage");
}

// ============================================================================
// Update Client Count Display
// ============================================================================

void qr_screen_update_clients(int count)
{
    if (!s_client_label) return;

    if (count > 0) {
        char text[32];
        snprintf(text, sizeof(text), "Connected: %d client%s",
                 count, count > 1 ? "s" : "");
        lv_label_set_text(s_client_label, text);
    } else {
        lv_label_set_text(s_client_label, "Waiting for connection...");
    }
}

// ============================================================================
// QR Screen Hide — return to previous screen
// ============================================================================

void qr_screen_hide(void)
{
    if (!s_qr_screen) return;

    // Return to previous screen
    if (s_prev_screen) {
        lv_disp_load_scr(s_prev_screen);
    }

    // Delete QR screen and all children
    lv_obj_del(s_qr_screen);
    s_qr_screen    = NULL;
    s_qr_code      = NULL;
    s_title_label  = NULL;
    s_info_label   = NULL;
    s_instr_label  = NULL;
    s_client_label = NULL;
    s_close_btn    = NULL;
    s_prev_screen  = NULL;

    ESP_LOGI(TAG, "QR screen closed");
}

// ============================================================================
// Query
// ============================================================================

bool qr_screen_is_active(void)
{
    return s_qr_screen != NULL;
}
