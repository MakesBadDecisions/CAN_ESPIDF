/**
 * @file imu_display.c
 * @brief IMU Bubble + G-Force / Tilt Visualization
 *
 * Renders inside a gauge slot panel (~144x144 px on 480x480 display).
 * The PID and unit dropdowns occupy the top/bottom ~20% of the panel,
 * so the IMU content is constrained to the center area via s_inner.
 *
 * Layout inside the gauge panel:
 *
 *   ┌──────────────────────────┐
 *   │  [PID dropdown]          │  ← managed by SquareLine/ui_events
 *   │ ┌──────────────────────┐ │
 *   │ │       0.2G↑           │ │  ← lon G / pitch at top of V-axis
 *   │ │         |             │ │
 *   │ │  ───────●────── 0.1G→ │ │  ← lat G / roll at right of H-axis
 *   │ │         |             │ │
 *   │ │         |             │ │
 *   │ └──────────────────────┘ │
 *   │  [Unit dropdown]         │  ← "G-Load" / "Tilt" mode selector
 *   └──────────────────────────┘
 *
 * Modes:
 *   G-Load: bubble tracks lateral/longitudinal acceleration (±1.5G)
 *   Tilt:   bubble tracks pitch/roll angles (±30°)
 *
 * Bubble color: green (<0.3G) → yellow (0.3-0.8G) → red (>0.8G).
 *
 * Data sourced from qmi8658_get_orientation() — lock-free volatile cache
 * written by Core 0 IMU task, safe to read from Core 1 LVGL context.
 */

#include "imu_display.h"
#include "qmi8658.h"
#include "device.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "imu_display";

// ============================================================================
// Configuration
// ============================================================================

#define IMU_UPDATE_MS       100     // 10 Hz LVGL timer
#define BUBBLE_RADIUS       5       // px — dot radius
#define G_RANGE             1.5f    // ±1.5G maps to inner edges (G-Load mode)
#define ANGLE_RANGE         30.0f   // ±30° maps to inner edges (Tilt mode)
#define MARGIN_PX           4       // Edge margin so dot doesn't clip border
#define DROPDOWN_ZONE_PCT   20      // Top/bottom % reserved for dropdowns

// ============================================================================
// Widget handles (only one IMU display instance at a time)
// ============================================================================

static lv_obj_t   *s_parent     = NULL;     // Gauge panel we're attached to
static lv_obj_t   *s_inner      = NULL;     // Inner container (between dropdowns)
static lv_obj_t   *s_bubble     = NULL;     // Moving dot
static lv_obj_t   *s_label_lat  = NULL;     // Lateral G / roll (right of H-axis)
static lv_obj_t   *s_label_lon  = NULL;     // Longitudinal G / pitch (top of V-axis)
static lv_obj_t   *s_line_h     = NULL;     // Horizontal crosshair
static lv_obj_t   *s_line_v     = NULL;     // Vertical crosshair
static lv_timer_t *s_timer      = NULL;

static imu_display_mode_t s_mode = IMU_MODE_G_LOAD;

// Line point arrays (static — LVGL holds pointer to these)
static lv_point_t s_h_points[2];
static lv_point_t s_v_points[2];

// ============================================================================
// Helpers
// ============================================================================

/** Map a value to pixel offset from center within a clamped range */
static int16_t value_to_px(float value, float range, int16_t half_range_px)
{
    float clamped = value;
    if (clamped >  range) clamped =  range;
    if (clamped < -range) clamped = -range;
    return (int16_t)(clamped / range * (float)half_range_px);
}

/** Pick bubble color based on total G-force */
static lv_color_t g_force_color(float g_total)
{
    float dynamic_g = fabsf(g_total - 1.0f);
    if (dynamic_g > 0.8f) return lv_color_hex(0xFF3333);    // Red
    if (dynamic_g > 0.3f) return lv_color_hex(0xFFCC00);    // Yellow
    return lv_color_hex(0x33FF33);                           // Green
}

// ============================================================================
// LVGL Timer Callback (Core 1, ~10 Hz)
// ============================================================================

static void imu_timer_cb(lv_timer_t *timer)
{
    (void)timer;

#if defined(HAS_IMU) && HAS_IMU
    qmi8658_orientation_t orient;
    if (qmi8658_get_orientation(&orient) != ESP_OK) return;
    if (!s_inner || !s_bubble) return;

    lv_coord_t pw = lv_obj_get_content_width(s_inner);
    lv_coord_t ph = lv_obj_get_content_height(s_inner);

    int16_t half_w = (pw / 2) - MARGIN_PX - BUBBLE_RADIUS;
    int16_t half_h = (ph / 2) - MARGIN_PX - BUBBLE_RADIUS;
    if (half_w < 1) half_w = 1;
    if (half_h < 1) half_h = 1;

    int16_t dx, dy;
    static char lat_buf[16], lon_buf[16];

    if (s_mode == IMU_MODE_TILT) {
        // Tilt mode: roll → horizontal, pitch → vertical
        dx = value_to_px(orient.roll,   ANGLE_RANGE, half_w);
        dy = value_to_px(-orient.pitch, ANGLE_RANGE, half_h);
        snprintf(lat_buf, sizeof(lat_buf), "%.1f\xC2\xB0R", orient.roll);
        snprintf(lon_buf, sizeof(lon_buf), "%.1f\xC2\xB0P", orient.pitch);
    } else {
        // G-Load mode: lateral G → horizontal, longitudinal G → vertical
        dx = value_to_px(orient.accel_lat,  G_RANGE, half_w);
        dy = value_to_px(-orient.accel_lon, G_RANGE, half_h);
        snprintf(lat_buf, sizeof(lat_buf), "%.1fG", orient.accel_lat);
        snprintf(lon_buf, sizeof(lon_buf), "%.1fG", orient.accel_lon);
    }

    // Position bubble at center + offset
    lv_coord_t cx = (pw / 2) + dx - BUBBLE_RADIUS;
    lv_coord_t cy = (ph / 2) + dy - BUBBLE_RADIUS;
    lv_obj_set_pos(s_bubble, cx, cy);

    // Color based on dynamic G
    lv_color_t color = g_force_color(orient.g_total);
    lv_obj_set_style_bg_color(s_bubble, color, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Update axis-end labels
    if (s_label_lat) lv_label_set_text(s_label_lat, lat_buf);
    if (s_label_lon) lv_label_set_text(s_label_lon, lon_buf);
#endif
}

// ============================================================================
// Public API
// ============================================================================

void imu_display_attach(lv_obj_t *parent)
{
    if (!parent) {
        ESP_LOGW(TAG, "Parent panel is NULL — cannot attach IMU display");
        return;
    }

#if !defined(HAS_IMU) || !HAS_IMU
    ESP_LOGI(TAG, "No IMU on this board — skipping attach");
    return;
#else
    // Detach from previous parent if we're already rendering somewhere
    if (s_parent) {
        imu_display_detach();
    }

    s_parent = parent;

    lv_coord_t pw = lv_obj_get_content_width(parent);
    lv_coord_t ph = lv_obj_get_content_height(parent);

    // Create inner container that avoids dropdown zones (top/bottom 20%)
    lv_coord_t top_margin = (ph * DROPDOWN_ZONE_PCT) / 100;
    lv_coord_t bot_margin = top_margin;
    lv_coord_t inner_h = ph - top_margin - bot_margin;

    s_inner = lv_obj_create(parent);
    lv_obj_set_size(s_inner, pw, inner_h);
    lv_obj_align(s_inner, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(s_inner, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_inner, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_inner, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_inner, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_coord_t iw = lv_obj_get_content_width(s_inner);
    lv_coord_t ih = inner_h;

    // --- Crosshair lines (subtle dark gray) ---
    static lv_style_t line_style;
    lv_style_init(&line_style);
    lv_style_set_line_width(&line_style, 1);
    lv_style_set_line_color(&line_style, lv_color_hex(0x333333));
    lv_style_set_line_opa(&line_style, LV_OPA_60);

    // Horizontal center line
    s_h_points[0].x = 0;        s_h_points[0].y = ih / 2;
    s_h_points[1].x = iw;       s_h_points[1].y = ih / 2;
    s_line_h = lv_line_create(s_inner);
    lv_line_set_points(s_line_h, s_h_points, 2);
    lv_obj_add_style(s_line_h, &line_style, LV_PART_MAIN);

    // Vertical center line
    s_v_points[0].x = iw / 2;   s_v_points[0].y = 0;
    s_v_points[1].x = iw / 2;   s_v_points[1].y = ih;
    s_line_v = lv_line_create(s_inner);
    lv_line_set_points(s_line_v, s_v_points, 2);
    lv_obj_add_style(s_line_v, &line_style, LV_PART_MAIN);

    // --- Bubble dot ---
    s_bubble = lv_obj_create(s_inner);
    lv_obj_set_size(s_bubble, BUBBLE_RADIUS * 2, BUBBLE_RADIUS * 2);
    lv_obj_set_style_radius(s_bubble, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bubble, lv_color_hex(0x33FF33), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_bubble, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_bubble, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_bubble, 0, LV_PART_MAIN);
    lv_obj_clear_flag(s_bubble, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(s_bubble, (iw / 2) - BUBBLE_RADIUS, (ih / 2) - BUBBLE_RADIUS);

    // --- Axis-end labels (font 10, gray) ---
    // Lateral G / Roll — right end of horizontal crosshair
    s_label_lat = lv_label_create(s_inner);
    lv_obj_set_style_text_font(s_label_lat, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_label_lat, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
    lv_obj_align(s_label_lat, LV_ALIGN_RIGHT_MID, -2, 0);
    lv_label_set_text(s_label_lat, "0.0G");

    // Longitudinal G / Pitch — top of vertical crosshair
    s_label_lon = lv_label_create(s_inner);
    lv_obj_set_style_text_font(s_label_lon, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_label_lon, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
    lv_obj_align(s_label_lon, LV_ALIGN_TOP_MID, 0, 1);
    lv_label_set_text(s_label_lon, "0.0G");

    // --- Start LVGL update timer ---
    s_timer = lv_timer_create(imu_timer_cb, IMU_UPDATE_MS, NULL);

    ESP_LOGI(TAG, "IMU display attached to panel (%dx%d, inner %dx%d, mode %s)",
             pw, ph, iw, ih,
             s_mode == IMU_MODE_TILT ? "Tilt" : "G-Load");
#endif
}

void imu_display_detach(void)
{
    if (s_timer) {
        lv_timer_del(s_timer);
        s_timer = NULL;
    }

    // Delete the inner container — takes all children (bubble, lines, labels)
    if (s_inner && s_parent) {
        lv_obj_del(s_inner);
    }

    s_parent    = NULL;
    s_inner     = NULL;
    s_bubble    = NULL;
    s_label_lat = NULL;
    s_label_lon = NULL;
    s_line_h    = NULL;
    s_line_v    = NULL;

    ESP_LOGI(TAG, "IMU display detached");
}

bool imu_display_is_attached(void)
{
    return s_parent != NULL;
}

lv_obj_t *imu_display_get_parent(void)
{
    return s_parent;
}

void imu_display_set_mode(imu_display_mode_t mode)
{
    s_mode = mode;
    ESP_LOGI(TAG, "IMU display mode -> %s",
             mode == IMU_MODE_TILT ? "Tilt" : "G-Load");
}

// ---------- Legacy API (backward compat) ----------

void imu_display_init(lv_obj_t *parent)
{
    imu_display_attach(parent);
}

void imu_display_deinit(void)
{
    imu_display_detach();
}
