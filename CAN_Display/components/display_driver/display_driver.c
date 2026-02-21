/**
 * @file display_driver.c
 * @brief Display Hardware Abstraction Layer Implementation
 *
 * Initializes ESP32-S3 RGB LCD panel and LVGL for multiple display boards.
 * Compile-time device selection via device.h provides pin/timing constants.
 *
 * Supported display boards:
 *   - CrowPanel 4.3"/5"/7" (DIS06043H/DIS07050/DIS08070H): Direct GPIO backlight, simple RGB init
 *   - Waveshare 2.1" Round (WS_TOUCH_LCD_21): ST7701S SPI init, TCA9554 reset/CS, PWM backlight
 *
 * Uses the Espressif-recommended anti-tearing approach:
 *   - Double framebuffer in PSRAM (num_fbs=2)
 *   - LVGL direct_mode: draws directly into the PSRAM framebuffers
 *   - Bounce buffers: SRAM cache between PSRAM and LCD peripheral
 *   - VSYNC-synced buffer swap: only swap on last flush, after VSYNC
 *   - Dirty area sync: copies invalidated regions to the other buffer
 *
 * Touch input is handled by the separate touch_driver component.
 */

#include "display_driver.h"
#include "device.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lvgl.h"

#if defined(DEVICE_WS_TOUCH_LCD_21)
    #include "driver/spi_master.h"
    #include "driver/ledc.h"
    #include "tca9554.h"
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

// Current backlight brightness (0-100%)
static uint8_t s_brightness = 0;

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
// Backlight Control — device-specific
// ============================================================================

#if defined(DEVICE_WS_TOUCH_LCD_21)

// Waveshare 2.1": PWM backlight via LEDC on GPIO6
// Matches Waveshare demo: gpio_config → ledc_timer → ledc_channel → fade_install → Set_Backlight
static esp_err_t backlight_init(void)
{
    // Step 1: Configure backlight GPIO as output (matches Waveshare demo)
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_BACKLIGHT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    // Step 2: LEDC timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz         = BL_PWM_FREQUENCY,
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 3: LEDC channel
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = PIN_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 4: Install LEDC fade ISR (required for reliable duty updates)
    ledc_fade_func_install(0);

    // Load saved brightness from NVS, fall back to compiled default
    uint8_t saved_pct = BL_DEFAULT_BRIGHTNESS;
    nvs_handle_t nvs;
    if (nvs_open("display", NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t val;
        if (nvs_get_u8(nvs, "bl_pct", &val) == ESP_OK) {
            saved_pct = val;
        }
        nvs_close(nvs);
    }

    // Apply initial brightness via the same path as runtime changes
    s_brightness = saved_pct;
    display_set_brightness(saved_pct);

    ESP_LOGI(TAG, "PWM backlight ON (GPIO %d, %d%%)", PIN_BACKLIGHT, saved_pct);
    return ESP_OK;
}

#else

// CrowPanel: Simple GPIO toggle backlight
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
    s_brightness = 100;

    return ESP_OK;
}

#endif // DEVICE_WS_TOUCH_LCD_21

// ============================================================================
// Brightness Control — public API
// ============================================================================

esp_err_t display_set_brightness(uint8_t percent)
{
    if (percent > 100) percent = 100;

#if defined(DEVICE_WS_TOUCH_LCD_21)
    // Waveshare: PWM via LEDC (formula from Waveshare demo ST7701S.c)
    // Duty = LEDC_MAX_Duty - 81 * (Backlight_MAX - Light); if Light==0 → Duty=0
    uint32_t max_duty = (1 << 13) - 1;  // 8191
    uint32_t duty = max_duty - 81 * (100 - percent);
    if (percent == 0) duty = 0;
    ESP_LOGI(TAG, "LEDC duty=%lu (percent=%d)", (unsigned long)duty, percent);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
#else
    // CrowPanel: GPIO on/off only
    gpio_set_level(PIN_BACKLIGHT, percent > 0 ? 1 : 0);
#endif

    s_brightness = percent;

    // Persist to NVS
    nvs_handle_t nvs;
    if (nvs_open("display", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, "bl_pct", percent);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Brightness set to %d%%", percent);
    return ESP_OK;
}

uint8_t display_get_brightness(void)
{
    return s_brightness;
}

// ============================================================================
// ST7701S SPI Panel Init (Waveshare 2.1" only)
// ============================================================================

#if defined(DEVICE_WS_TOUCH_LCD_21)

// ST7701S init sequence — 9-bit SPI (1 cmd bit + 8 addr bits)
// cmd=0 → register command, cmd=1 → register data
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_len;
    uint16_t delay_ms;
} st7701s_cmd_t;

static const st7701s_cmd_t st7701s_init_cmds[] = {
    // CMD2 BK0
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, {0x3B, 0x00}, 2, 0},
    {0xC1, {0x0B, 0x02}, 2, 0},
    {0xC2, {0x07, 0x02}, 2, 0},
    {0xCC, {0x10}, 1, 0},
    {0xCD, {0x08}, 1, 0},
    // Positive gamma
    {0xB0, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x06, 0x05, 0x09,
            0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31, 0x18}, 16, 0},
    // Negative gamma
    {0xB1, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x07, 0x05, 0x09,
            0x09, 0x21, 0x05, 0x13, 0x11, 0x2a, 0x31, 0x18}, 16, 0},

    // CMD2 BK1
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, {0x6d}, 1, 0},
    {0xB1, {0x37}, 1, 0},
    {0xB2, {0x81}, 1, 0},
    {0xB3, {0x80}, 1, 0},
    {0xB5, {0x43}, 1, 0},
    {0xB7, {0x85}, 1, 0},
    {0xB8, {0x20}, 1, 0},
    {0xC1, {0x78}, 1, 0},
    {0xC2, {0x78}, 1, 0},
    {0xD0, {0x88}, 1, 0},
    // GIP timing
    {0xE0, {0x00, 0x00, 0x02}, 3, 0},
    {0xE1, {0x03, 0xA0, 0x00, 0x00, 0x04, 0xA0, 0x00, 0x00,
            0x00, 0x20, 0x20}, 11, 0},
    {0xE2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00}, 13, 0},
    {0xE3, {0x00, 0x00, 0x11, 0x00}, 4, 0},
    {0xE4, {0x22, 0x00}, 2, 0},
    {0xE5, {0x05, 0xEC, 0xA0, 0xA0, 0x07, 0xEE, 0xA0, 0xA0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16, 0},
    {0xE6, {0x00, 0x00, 0x11, 0x00}, 4, 0},
    {0xE7, {0x22, 0x00}, 2, 0},
    {0xE8, {0x06, 0xED, 0xA0, 0xA0, 0x08, 0xEF, 0xA0, 0xA0,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16, 0},
    {0xEB, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 7, 0},
    {0xED, {0xFF, 0xFF, 0xFF, 0xBA, 0x0A, 0xBF, 0x45, 0xFF,
            0xFF, 0x54, 0xFB, 0xA0, 0xAB, 0xFF, 0xFF, 0xFF}, 16, 0},
    {0xEF, {0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F}, 6, 0},

    // CMD2 BK3
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, {0x08}, 1, 0},

    // Back to CMD1
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x36, {0x00}, 1, 0},      // MADCTL: no rotation
    {0x3A, {0x66}, 1, 0},      // Pixel format: RGB666

    // Sleep Out
    {0x11, {0}, 0, 480},
    // Display Inversion Off
    {0x20, {0}, 0, 120},
    // Display On
    {0x29, {0}, 0, 0},
};

#define ST7701S_CMD_COUNT (sizeof(st7701s_init_cmds) / sizeof(st7701s_init_cmds[0]))

static esp_err_t st7701s_spi_init(void)
{
    // Step 1: Reset LCD via TCA9554 EXIO1
    tca9554_set_pin(EXIO_PIN_1, false);     // LCD Reset LOW
    vTaskDelay(pdMS_TO_TICKS(10));
    tca9554_set_pin(EXIO_PIN_1, true);      // LCD Reset HIGH
    vTaskDelay(pdMS_TO_TICKS(50));

    // Step 2: Enable LCD SPI CS via TCA9554 EXIO3 (active LOW)
    tca9554_set_pin(EXIO_PIN_3, true);      // CS HIGH first
    vTaskDelay(pdMS_TO_TICKS(10));
    tca9554_set_pin(EXIO_PIN_3, false);     // CS LOW (active)
    vTaskDelay(pdMS_TO_TICKS(50));

    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 3: Init SPI bus for 9-bit ST7701S communication
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = LCD_SPI_MOSI,
        .miso_io_num   = -1,
        .sclk_io_num   = LCD_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 9-bit SPI: 1 command bit (D/C) + 8 address bits
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = LCD_SPI_FREQ,
        .mode           = 0,
        .spics_io_num   = -1,      // CS via TCA9554
        .queue_size     = 1,
        .command_bits   = 1,        // D/C bit
        .address_bits   = 8,        // Register address
    };

    spi_device_handle_t spi_dev;
    ret = spi_bus_add_device(LCD_SPI_HOST, &dev_cfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LCD SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Step 4: Send ST7701S init commands
    for (int i = 0; i < ST7701S_CMD_COUNT; i++) {
        const st7701s_cmd_t *c = &st7701s_init_cmds[i];

        // Write command (cmd bit = 0)
        spi_transaction_t t = {
            .rxlength = 0,
            .length   = 0,
            .cmd      = 0,          // command
            .addr     = c->cmd,
        };
        spi_device_transmit(spi_dev, &t);

        // Write data bytes (cmd bit = 1)
        for (int j = 0; j < c->data_len; j++) {
            spi_transaction_t dt = {
                .rxlength = 0,
                .length   = 0,
                .cmd      = 1,      // data
                .addr     = c->data[j],
            };
            spi_device_transmit(spi_dev, &dt);
        }

        if (c->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(c->delay_ms));
        }
    }

    // Step 5: Done with SPI — remove device and free bus
    // (GPIO1/2 will be reused by SD card SDMMC later)
    spi_bus_remove_device(spi_dev);
    spi_bus_free(LCD_SPI_HOST);

    ESP_LOGI(TAG, "ST7701S SPI init complete (%d commands)", ST7701S_CMD_COUNT);
    return ESP_OK;
}

#endif // DEVICE_WS_TOUCH_LCD_21

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

#if defined(DEVICE_WS_TOUCH_LCD_21)
    // Waveshare 2.1": ST7701S needs SPI register programming before RGB mode
    ret = st7701s_spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ST7701S SPI init failed: %s", esp_err_to_name(ret));
        return ret;
    }
#endif

    ret = backlight_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Backlight init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = rgb_panel_init();
    if (ret != ESP_OK) {
        return ret;
    }

#if defined(DEVICE_WS_TOUCH_LCD_21)
    // Disable LCD SPI CS after RGB panel is running
    tca9554_set_pin(EXIO_PIN_3, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    tca9554_set_pin(EXIO_PIN_3, true);
    vTaskDelay(pdMS_TO_TICKS(50));
#endif

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
