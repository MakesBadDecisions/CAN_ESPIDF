/**
 * @file display_driver.c
 * @brief Display Hardware Abstraction Layer Implementation
 *
 * Initializes ESP32-S3 RGB LCD panel and LVGL for CrowPanel displays.
 *
 * Uses the Espressif-recommended anti-tearing approach:
 *   - Double framebuffer in PSRAM (num_fbs=2)
 *   - LVGL direct_mode: draws directly into the PSRAM framebuffers
 *   - Bounce buffers: SRAM cache between PSRAM and LCD peripheral
 *   - VSYNC-synced buffer swap: only swap on last flush, after VSYNC
 *   - Dirty area sync: copies invalidated regions to the other buffer
 *
 * This ensures DMA reads from FB-A while LVGL writes to FB-B.
 * They never access the same framebuffer simultaneously.
 * The dirty area copy keeps both buffers consistent so switching
 * between them never shows stale content.
 *
 * Touch input is handled by the separate touch_driver component.
 */

#include "display_driver.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lvgl.h"

#if defined(DEVICE_DIS06043H)
    #include "dis06043h.h"
#elif defined(DEVICE_DIS07050H)
    #include "dis07050h.h"
#elif defined(DEVICE_DIS08070H)
    #include "dis08070h.h"
#else
    #error "No device defined! Set DEVICE_DIS06043H, DEVICE_DIS07050H, or DEVICE_DIS08070H"
#endif

static const char *TAG = "display";

// ============================================================================
// Private Data
// ============================================================================

static esp_lcd_panel_handle_t s_panel_handle = NULL;
static lv_disp_draw_buf_t s_disp_buf;
static lv_disp_drv_t s_disp_drv;
static SemaphoreHandle_t s_lvgl_mutex = NULL;
static SemaphoreHandle_t s_vsync_sem = NULL;
static TaskHandle_t s_lvgl_task_handle = NULL;

// ============================================================================
// VSYNC Callback — signals when frame transmission is complete (ISR context)
// ============================================================================

static bool IRAM_ATTR on_vsync_cb(esp_lcd_panel_handle_t panel,
                                   const esp_lcd_rgb_panel_event_data_t *edata,
                                   void *user_ctx)
{
    BaseType_t high_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync_sem, &high_task_woken);
    return (high_task_woken == pdTRUE);
}

// ============================================================================
// LVGL Flush Callback — swaps framebuffers on the last flush after VSYNC
// ============================================================================

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;

    // In direct_mode, LVGL may call flush multiple times per frame
    // (once per dirty area). Only swap buffers on the LAST flush.
    if (lv_disp_flush_is_last(drv)) {
        // Wait for current frame to finish transmitting
        xSemaphoreTake(s_vsync_sem, pdMS_TO_TICKS(100));

        // Tell the RGB peripheral to start reading from the buffer
        // LVGL just finished drawing into
        esp_lcd_panel_draw_bitmap(panel, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, color_map);

        // CRITICAL: Copy dirty areas from the just-drawn buffer to the other buffer.
        // In direct_mode with double buffers, LVGL only draws dirty regions into the
        // active buffer. The other buffer still has stale content for those regions.
        // Without this copy, switching buffers next frame shows stale pixels where
        // widgets changed — causing visible flicker on touch interactions.
        lv_disp_t *disp = _lv_refr_get_disp_refreshing();
        lv_color_t *other_buf = (color_map == drv->draw_buf->buf1)
                                ? drv->draw_buf->buf2
                                : drv->draw_buf->buf1;

        for (int i = 0; i < disp->inv_p; i++) {
            if (disp->inv_area_joined[i]) continue;

            lv_area_t *a = &disp->inv_areas[i];
            int32_t w = lv_area_get_width(a) * sizeof(lv_color_t);
            for (int32_t y = a->y1; y <= a->y2; y++) {
                memcpy(other_buf + y * DISPLAY_WIDTH + a->x1,
                       color_map  + y * DISPLAY_WIDTH + a->x1, w);
            }
        }
    }

    lv_disp_flush_ready(drv);
}

// ============================================================================
// LVGL Task
// ============================================================================

static void lvgl_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "LVGL task started");

    while (1) {
        if (xSemaphoreTake(s_lvgl_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lv_timer_handler();
            xSemaphoreGive(s_lvgl_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// Backlight Control
// ============================================================================

static esp_err_t backlight_init(void)
{
    uint64_t pin_mask = (1ULL << PIN_BACKLIGHT);
#if PIN_PANEL_ENABLE >= 0
    pin_mask |= (1ULL << PIN_PANEL_ENABLE);
#endif

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

#if PIN_PANEL_ENABLE >= 0
    gpio_set_level(PIN_PANEL_ENABLE, 1);
    ESP_LOGI(TAG, "Backlight ON, Panel enable ON");
#else
    ESP_LOGI(TAG, "Backlight ON");
#endif
    gpio_set_level(PIN_BACKLIGHT, 1);

    return ESP_OK;
}

// ============================================================================
// RGB Panel Initialization
// ============================================================================

static esp_err_t rgb_panel_init(void)
{
    ESP_LOGI(TAG, "Initializing RGB LCD panel %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);

    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = DISPLAY_FREQ_WRITE,
            .h_res = DISPLAY_WIDTH,
            .v_res = DISPLAY_HEIGHT,
            .hsync_pulse_width = HSYNC_PULSE_WIDTH,
            .hsync_back_porch = HSYNC_BACK_PORCH,
            .hsync_front_porch = HSYNC_FRONT_PORCH,
            .vsync_pulse_width = VSYNC_PULSE_WIDTH,
            .vsync_back_porch = VSYNC_BACK_PORCH,
            .vsync_front_porch = VSYNC_FRONT_PORCH,
            .flags = {
                .hsync_idle_low = HSYNC_POLARITY,
                .vsync_idle_low = VSYNC_POLARITY,
                .de_idle_high = DE_IDLE_HIGH,
                .pclk_active_neg = PCLK_ACTIVE_NEG,
                .pclk_idle_high = PCLK_IDLE_HIGH,
            },
        },
        .data_width = DISPLAY_DATA_WIDTH,
        .bits_per_pixel = DISPLAY_BITS_PER_PX,
        .num_fbs = 2,          // Double framebuffer for tear-free rendering
        .bounce_buffer_size_px = DISPLAY_WIDTH * BOUNCE_BUFFER_LINES,
        .sram_trans_align = 8,
        .psram_trans_align = 64,
        .hsync_gpio_num = PIN_HSYNC,
        .vsync_gpio_num = PIN_VSYNC,
        .de_gpio_num = PIN_DE,
        .pclk_gpio_num = PIN_PCLK,
        .disp_gpio_num = -1,
        .data_gpio_nums = {
            PIN_B0, PIN_B1, PIN_B2, PIN_B3, PIN_B4,
            PIN_G0, PIN_G1, PIN_G2, PIN_G3, PIN_G4, PIN_G5,
            PIN_R0, PIN_R1, PIN_R2, PIN_R3, PIN_R4,
        },
        .flags = {
            .fb_in_psram = 1,
        },
    };

    esp_err_t ret = esp_lcd_new_rgb_panel(&panel_config, &s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RGB panel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_reset(s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_init(s_panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init panel: %s", esp_err_to_name(ret));
        return ret;
    }

    // VSYNC semaphore for framebuffer swap synchronization
    s_vsync_sem = xSemaphoreCreateBinary();
    if (s_vsync_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create VSYNC semaphore");
        return ESP_ERR_NO_MEM;
    }

    // Register VSYNC callback for tear-free buffer swap
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = on_vsync_cb,
    };
    ret = esp_lcd_rgb_panel_register_event_callbacks(s_panel_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register VSYNC callback: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "RGB panel initialized (double FB, VSYNC-synced swap)");
    return ESP_OK;
}

// ============================================================================
// LVGL Initialization
// ============================================================================

static esp_err_t lvgl_init(void)
{
    ESP_LOGI(TAG, "Initializing LVGL");

    lv_init();

    s_lvgl_mutex = xSemaphoreCreateMutex();
    if (s_lvgl_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create LVGL mutex");
        return ESP_ERR_NO_MEM;
    }

    // Get the two PSRAM framebuffer pointers from the RGB panel
    void *fb1 = NULL, *fb2 = NULL;
    esp_err_t ret = esp_lcd_rgb_panel_get_frame_buffer(s_panel_handle, 2, &fb1, &fb2);
    if (ret != ESP_OK || fb1 == NULL || fb2 == NULL) {
        ESP_LOGE(TAG, "Failed to get framebuffer pointers: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Framebuffers: fb1=%p, fb2=%p (%u bytes each)",
             fb1, fb2, (unsigned)(DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(lv_color_t)));

    // Use the PSRAM framebuffers as LVGL's draw buffers (full-screen, double-buffered)
    lv_disp_draw_buf_init(&s_disp_buf, fb1, fb2, DISPLAY_WIDTH * DISPLAY_HEIGHT);

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res = DISPLAY_WIDTH;
    s_disp_drv.ver_res = DISPLAY_HEIGHT;
    s_disp_drv.flush_cb = lvgl_flush_cb;
    s_disp_drv.draw_buf = &s_disp_buf;
    s_disp_drv.user_data = s_panel_handle;
    s_disp_drv.direct_mode = 1;  // LVGL draws directly into PSRAM framebuffers

    lv_disp_t *disp = lv_disp_drv_register(&s_disp_drv);
    if (disp == NULL) {
        ESP_LOGE(TAG, "Failed to register display driver");
        return ESP_FAIL;
    }

    // Set default background to black
    lv_disp_set_bg_color(disp, lv_color_black());
    lv_disp_set_bg_opa(disp, LV_OPA_COVER);

    BaseType_t xRet = xTaskCreatePinnedToCore(
        lvgl_task,
        "lvgl",
        16384,
        NULL,
        3,
        &s_lvgl_task_handle,
        1
    );

    if (xRet != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LVGL task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LVGL initialized (direct_mode, double FB)");
    return ESP_OK;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t display_init(void)
{
    esp_err_t ret;

    ret = backlight_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Backlight init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = rgb_panel_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = lvgl_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "Display driver initialized (%dx%d)", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    return ESP_OK;
}

void display_clear(uint16_t color)
{
    if (s_lvgl_mutex && xSemaphoreTake(s_lvgl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lv_obj_t *scr = lv_scr_act();
        lv_obj_set_style_bg_color(scr, lv_color_hex(color), 0);
        xSemaphoreGive(s_lvgl_mutex);
    }
}

void display_flush(void)
{
    // LVGL handles flushing automatically via the timer task
}

bool display_lock(uint32_t timeout_ms)
{
    if (s_lvgl_mutex == NULL) {
        return false;
    }
    return xSemaphoreTake(s_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void display_unlock(void)
{
    if (s_lvgl_mutex) {
        xSemaphoreGive(s_lvgl_mutex);
    }
}
