/**
 * @file touch_driver.c
 * @brief XPT2046 Touch Driver with Core 0 Polling and NVS Calibration
 *
 * Touch SPI reads run on a dedicated Core 0 task to avoid stalling the
 * LCD bounce buffer DMA ISR on Core 1. The LVGL input device callback
 * only reads a volatile cache struct — zero SPI, instant return.
 *
 * Calibration uses a 4-corner crosshair screen. Results are stored in
 * NVS namespace "touch_cal" and loaded automatically on init.
 */

#include "touch_driver.h"
#include "display_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lvgl.h"

#if defined(DEVICE_DIS06043H)
    #include "dis06043h.h"
#elif defined(DEVICE_DIS07050H)
    #include "dis07050h.h"
#elif defined(DEVICE_DIS08070H)
    #include "dis08070h.h"
#else
    #error "No device defined!"
#endif

static const char *TAG = "touch";

// XPT2046 SPI commands (12-bit, differential reference)
#define XPT2046_CMD_X   0xD0  // Channel 5: X position
#define XPT2046_CMD_Y   0x90  // Channel 1: Y position
#define XPT2046_CMD_Z1  0xB0  // Channel 3: Z1 pressure

// Pressure threshold (higher = more pressure required)
#define TOUCH_PRESSURE_THRESHOLD  100

// Calibration crosshair offset from screen edge (pixels)
#define CAL_OFFSET  20

// NVS namespace and keys
#define NVS_NAMESPACE   "touch_cal"
#define NVS_KEY_X_MIN   "x_min"
#define NVS_KEY_X_MAX   "x_max"
#define NVS_KEY_Y_MIN   "y_min"
#define NVS_KEY_Y_MAX   "y_max"
#define NVS_KEY_X_INV   "x_inv"
#define NVS_KEY_Y_INV   "y_inv"

// ============================================================================
// Calibration Data
// ============================================================================

typedef struct {
    uint16_t x_min;
    uint16_t x_max;
    uint16_t y_min;
    uint16_t y_max;
    bool     x_inverted;
    bool     y_inverted;
    bool     valid;
} touch_cal_t;

// ============================================================================
// Touch State Cache (written by Core 0 task, read by Core 1 LVGL callback)
// ============================================================================

typedef struct {
    volatile int16_t  x;
    volatile int16_t  y;
    volatile bool     pressed;
} touch_state_t;

// ============================================================================
// Private Data
// ============================================================================

static spi_device_handle_t s_touch_spi = NULL;
static TaskHandle_t        s_touch_task_handle = NULL;
static lv_indev_drv_t      s_indev_drv;
static touch_state_t       s_touch_state = { .x = 0, .y = 0, .pressed = false };
static touch_cal_t         s_cal = {
    .x_min = TOUCH_X_MIN,
    .x_max = TOUCH_X_MAX,
    .y_min = TOUCH_Y_MIN,
    .y_max = TOUCH_Y_MAX,
    .x_inverted = TOUCH_X_INVERTED,
    .y_inverted = false,
    .valid = true,
};

// Flag: when true, touch_task writes raw ADC values (no calibration mapping)
static volatile bool s_raw_mode = false;

// ============================================================================
// SPI Read Helper
// ============================================================================

static uint16_t xpt2046_read_channel(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 24,       // 8-bit command + 16-bit response
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data = { cmd, 0, 0 },
    };

    esp_err_t ret = spi_device_polling_transmit(s_touch_spi, &t);
    if (ret != ESP_OK) {
        return 0;
    }

    // 12-bit result in bits [14:3] of the 16-bit response
    uint16_t raw = ((uint16_t)t.rx_data[1] << 8) | t.rx_data[2];
    return raw >> 3;
}

// ============================================================================
// Coordinate Mapping
// ============================================================================

static void map_raw_to_screen(uint16_t raw_x, uint16_t raw_y,
                              int16_t *screen_x, int16_t *screen_y)
{
    int32_t sx, sy;

    // Map raw ADC range to screen pixels
    if (s_cal.x_inverted) {
        sx = (int32_t)(s_cal.x_max - raw_x) * DISPLAY_WIDTH / (s_cal.x_max - s_cal.x_min);
    } else {
        sx = (int32_t)(raw_x - s_cal.x_min) * DISPLAY_WIDTH / (s_cal.x_max - s_cal.x_min);
    }

    if (s_cal.y_inverted) {
        sy = (int32_t)(s_cal.y_max - raw_y) * DISPLAY_HEIGHT / (s_cal.y_max - s_cal.y_min);
    } else {
        sy = (int32_t)(raw_y - s_cal.y_min) * DISPLAY_HEIGHT / (s_cal.y_max - s_cal.y_min);
    }

    // Clamp to screen bounds
    if (sx < 0) sx = 0;
    if (sx >= DISPLAY_WIDTH) sx = DISPLAY_WIDTH - 1;
    if (sy < 0) sy = 0;
    if (sy >= DISPLAY_HEIGHT) sy = DISPLAY_HEIGHT - 1;

    *screen_x = (int16_t)sx;
    *screen_y = (int16_t)sy;
}

// ============================================================================
// Touch Polling Task (Core 0)
// ============================================================================

static void touch_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Touch task started on Core %d", xPortGetCoreID());

    while (1) {
        // Check INT pin — HIGH means no touch
        if (gpio_get_level(TOUCH_INT_PIN)) {
            s_touch_state.pressed = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Read pressure to confirm real touch
        uint16_t z1 = xpt2046_read_channel(XPT2046_CMD_Z1);
        if (z1 < TOUCH_PRESSURE_THRESHOLD) {
            s_touch_state.pressed = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Read raw X and Y
        uint16_t raw_x = xpt2046_read_channel(XPT2046_CMD_X);
        uint16_t raw_y = xpt2046_read_channel(XPT2046_CMD_Y);

        if (s_raw_mode) {
            // During calibration: store raw ADC values directly
            s_touch_state.x = (int16_t)raw_x;
            s_touch_state.y = (int16_t)raw_y;
            s_touch_state.pressed = true;
        } else {
            // Normal mode: map through calibration
            int16_t sx, sy;
            map_raw_to_screen(raw_x, raw_y, &sx, &sy);
            s_touch_state.x = sx;
            s_touch_state.y = sy;
            s_touch_state.pressed = true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// LVGL Input Device Callback (Core 1 — instant, no SPI)
// ============================================================================

static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;

    if (s_touch_state.pressed) {
        data->point.x = s_touch_state.x;
        data->point.y = s_touch_state.y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ============================================================================
// NVS Load / Save
// ============================================================================

static esp_err_t cal_load_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t x_min, x_max, y_min, y_max;
    uint8_t x_inv, y_inv;

    ret = nvs_get_u16(handle, NVS_KEY_X_MIN, &x_min);
    if (ret != ESP_OK) goto fail;
    ret = nvs_get_u16(handle, NVS_KEY_X_MAX, &x_max);
    if (ret != ESP_OK) goto fail;
    ret = nvs_get_u16(handle, NVS_KEY_Y_MIN, &y_min);
    if (ret != ESP_OK) goto fail;
    ret = nvs_get_u16(handle, NVS_KEY_Y_MAX, &y_max);
    if (ret != ESP_OK) goto fail;
    ret = nvs_get_u8(handle, NVS_KEY_X_INV, &x_inv);
    if (ret != ESP_OK) goto fail;
    ret = nvs_get_u8(handle, NVS_KEY_Y_INV, &y_inv);
    if (ret != ESP_OK) goto fail;

    s_cal.x_min = x_min;
    s_cal.x_max = x_max;
    s_cal.y_min = y_min;
    s_cal.y_max = y_max;
    s_cal.x_inverted = (x_inv != 0);
    s_cal.y_inverted = (y_inv != 0);
    s_cal.valid = true;

    nvs_close(handle);
    ESP_LOGI(TAG, "Calibration loaded from NVS: x[%u-%u] y[%u-%u] inv(%d,%d)",
             x_min, x_max, y_min, y_max, x_inv, y_inv);
    return ESP_OK;

fail:
    nvs_close(handle);
    return ret;
}

static esp_err_t cal_save_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    nvs_set_u16(handle, NVS_KEY_X_MIN, s_cal.x_min);
    nvs_set_u16(handle, NVS_KEY_X_MAX, s_cal.x_max);
    nvs_set_u16(handle, NVS_KEY_Y_MIN, s_cal.y_min);
    nvs_set_u16(handle, NVS_KEY_Y_MAX, s_cal.y_max);
    nvs_set_u8(handle, NVS_KEY_X_INV, s_cal.x_inverted ? 1 : 0);
    nvs_set_u8(handle, NVS_KEY_Y_INV, s_cal.y_inverted ? 1 : 0);

    ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved to NVS: x[%u-%u] y[%u-%u] inv(%d,%d)",
                 s_cal.x_min, s_cal.x_max, s_cal.y_min, s_cal.y_max,
                 s_cal.x_inverted, s_cal.y_inverted);
    }
    return ret;
}

// ============================================================================
// SPI Bus Initialization
// ============================================================================

static esp_err_t spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = TOUCH_SPI_MOSI,
        .miso_io_num = TOUCH_SPI_MISO,
        .sclk_io_num = TOUCH_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    // No DMA — small transfers, avoids PSRAM bus contention
    esp_err_t ret = spi_bus_initialize(TOUCH_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = TOUCH_SPI_FREQ,
        .mode = 0,
        .spics_io_num = TOUCH_SPI_CS,
        .queue_size = 1,
    };

    ret = spi_bus_add_device(TOUCH_SPI_HOST, &dev_cfg, &s_touch_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // INT pin — input, no pull (external pull-up on board)
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << TOUCH_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT pin config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI initialized (Host %d, CS=%d, INT=%d, %d Hz)",
             TOUCH_SPI_HOST, TOUCH_SPI_CS, TOUCH_INT_PIN, TOUCH_SPI_FREQ);
    return ESP_OK;
}

// ============================================================================
// Calibration Screen
// ============================================================================

// Corner positions for calibration (screen coordinates)
static const int16_t cal_points[4][2] = {
    { CAL_OFFSET,                    CAL_OFFSET },                     // Top-left
    { DISPLAY_WIDTH - CAL_OFFSET,    CAL_OFFSET },                     // Top-right
    { DISPLAY_WIDTH - CAL_OFFSET,    DISPLAY_HEIGHT - CAL_OFFSET },    // Bottom-right
    { CAL_OFFSET,                    DISPLAY_HEIGHT - CAL_OFFSET },    // Bottom-left
};

static const char *cal_corner_names[4] = {
    "top-left", "top-right", "bottom-right", "bottom-left"
};

static void draw_crosshair(lv_obj_t *screen, int16_t cx, int16_t cy,
                           lv_obj_t **h_line, lv_obj_t **v_line)
{
    // Horizontal line of crosshair
    *h_line = lv_obj_create(screen);
    lv_obj_remove_style_all(*h_line);
    lv_obj_set_size(*h_line, 20, 2);
    lv_obj_set_pos(*h_line, cx - 10, cy - 1);
    lv_obj_set_style_bg_color(*h_line, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_bg_opa(*h_line, LV_OPA_COVER, 0);
    lv_obj_clear_flag(*h_line, LV_OBJ_FLAG_SCROLLABLE);

    // Vertical line of crosshair
    *v_line = lv_obj_create(screen);
    lv_obj_remove_style_all(*v_line);
    lv_obj_set_size(*v_line, 2, 20);
    lv_obj_set_pos(*v_line, cx - 1, cy - 10);
    lv_obj_set_style_bg_color(*v_line, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_bg_opa(*v_line, LV_OPA_COVER, 0);
    lv_obj_clear_flag(*v_line, LV_OBJ_FLAG_SCROLLABLE);
}

esp_err_t touch_start_calibration(void)
{
    ESP_LOGI(TAG, "Starting touch calibration");

    // Create calibration screen — strip all default padding so positions are
    // absolute to the screen edge, not offset by theme padding
    lv_obj_t *cal_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(cal_scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(cal_scr, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(cal_scr, 0, 0);
    lv_obj_set_style_border_width(cal_scr, 0, 0);
    lv_obj_clear_flag(cal_scr, LV_OBJ_FLAG_SCROLLABLE);

    // Instruction label — full width, text centered
    lv_obj_t *label = lv_label_create(cal_scr);
    lv_obj_set_width(label, DISPLAY_WIDTH);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_align(label, LV_ALIGN_CENTER);
    lv_label_set_text(label, "Touch the crosshair");

    // Save the active screen so we can restore it after calibration
    lv_obj_t *prev_scr = lv_scr_act();
    lv_disp_load_scr(cal_scr);

    // Enable raw mode so touch_task gives us raw ADC values
    s_raw_mode = true;

    // Collect raw touch data for each corner
    uint16_t raw_x[4], raw_y[4];

    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "Calibration point %d: %s (%d, %d)",
                 i, cal_corner_names[i], cal_points[i][0], cal_points[i][1]);

        // Draw crosshair at this corner
        lv_obj_t *h_line, *v_line;
        draw_crosshair(cal_scr, cal_points[i][0], cal_points[i][1], &h_line, &v_line);

        // Update label with shorter text to fit screen
        char buf[48];
        snprintf(buf, sizeof(buf), "Touch %s", cal_corner_names[i]);
        lv_label_set_text(label, buf);

        // Release lock briefly so LVGL can render the crosshair
        display_unlock();
        vTaskDelay(pdMS_TO_TICKS(200));

        // Wait for touch release first (debounce from previous touch)
        while (s_touch_state.pressed) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // Wait for stable touch press
        int stable_count = 0;
        int32_t sum_x = 0, sum_y = 0;
        while (stable_count < 5) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (s_touch_state.pressed) {
                sum_x += s_touch_state.x;
                sum_y += s_touch_state.y;
                stable_count++;
            } else {
                stable_count = 0;
                sum_x = 0;
                sum_y = 0;
            }
        }

        raw_x[i] = (uint16_t)(sum_x / 5);
        raw_y[i] = (uint16_t)(sum_y / 5);

        ESP_LOGI(TAG, "  Raw: X=%u, Y=%u", raw_x[i], raw_y[i]);

        // Re-acquire display lock
        display_lock(5000);

        // Remove crosshair
        lv_obj_del(h_line);
        lv_obj_del(v_line);

        // Wait for touch release
        while (s_touch_state.pressed) {
            display_unlock();
            vTaskDelay(pdMS_TO_TICKS(20));
            display_lock(5000);
        }
    }

    // Disable raw mode
    s_raw_mode = false;

    // Calculate calibration from 4 corners
    // Left side: points 0 (top-left) and 3 (bottom-left)
    // Right side: points 1 (top-right) and 2 (bottom-right)
    // Top side: points 0 (top-left) and 1 (top-right)
    // Bottom side: points 2 (bottom-right) and 3 (bottom-left)
    uint16_t left_x_avg  = (raw_x[0] + raw_x[3]) / 2;
    uint16_t right_x_avg = (raw_x[1] + raw_x[2]) / 2;
    uint16_t top_y_avg   = (raw_y[0] + raw_y[1]) / 2;
    uint16_t bot_y_avg   = (raw_y[2] + raw_y[3]) / 2;

    // Determine inversion: if left screen edge gives higher raw values, X is inverted
    bool x_inv = (left_x_avg > right_x_avg);
    bool y_inv = (top_y_avg > bot_y_avg);

    // Store ordered min/max (min < max always)
    if (x_inv) {
        s_cal.x_min = right_x_avg;
        s_cal.x_max = left_x_avg;
    } else {
        s_cal.x_min = left_x_avg;
        s_cal.x_max = right_x_avg;
    }

    if (y_inv) {
        s_cal.y_min = bot_y_avg;
        s_cal.y_max = top_y_avg;
    } else {
        s_cal.y_min = top_y_avg;
        s_cal.y_max = bot_y_avg;
    }

    s_cal.x_inverted = x_inv;
    s_cal.y_inverted = y_inv;
    s_cal.valid = true;

    ESP_LOGI(TAG, "Calibration result: x[%u-%u] y[%u-%u] inv(%d,%d)",
             s_cal.x_min, s_cal.x_max, s_cal.y_min, s_cal.y_max,
             s_cal.x_inverted, s_cal.y_inverted);

    // Save to NVS
    esp_err_t ret = cal_save_to_nvs();

    // Show completion message
    lv_label_set_text(label, "Calibration complete!");
    display_unlock();
    vTaskDelay(pdMS_TO_TICKS(1000));
    display_lock(5000);

    // Restore previous screen and clean up
    lv_disp_load_scr(prev_scr);
    lv_obj_del(cal_scr);

    return ret;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t touch_init(void)
{
    esp_err_t ret;

    ret = spi_init();
    if (ret != ESP_OK) {
        return ret;
    }

    // Try to load calibration from NVS; fall back to header defaults
    if (cal_load_from_nvs() != ESP_OK) {
        ESP_LOGW(TAG, "No NVS calibration found, using header defaults");
    }

    // Start touch polling task on Core 0
    BaseType_t xRet = xTaskCreatePinnedToCore(
        touch_task,
        "touch",
        4096,
        NULL,
        2,
        &s_touch_task_handle,
        0  // Core 0 — away from LCD ISR on Core 1
    );

    if (xRet != pdPASS) {
        ESP_LOGE(TAG, "Failed to create touch task");
        return ESP_FAIL;
    }

    // Register LVGL input device under display lock (LVGL task is already running)
    if (!display_lock(1000)) {
        ESP_LOGE(TAG, "Failed to acquire display lock for indev registration");
        return ESP_FAIL;
    }

    lv_indev_drv_init(&s_indev_drv);
    s_indev_drv.type = LV_INDEV_TYPE_POINTER;
    s_indev_drv.read_cb = lvgl_touch_cb;

    lv_indev_t *indev = lv_indev_drv_register(&s_indev_drv);
    display_unlock();

    if (indev == NULL) {
        ESP_LOGE(TAG, "Failed to register LVGL input device");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Touch initialized (task on Core 0, LVGL indev registered)");
    return ESP_OK;
}

bool touch_has_calibration(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return false;
    }

    uint16_t dummy;
    ret = nvs_get_u16(handle, NVS_KEY_X_MIN, &dummy);
    nvs_close(handle);

    return (ret == ESP_OK);
}

esp_err_t touch_clear_calibration(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    nvs_erase_all(handle);
    ret = nvs_commit(handle);
    nvs_close(handle);

    // Revert to header defaults
    s_cal.x_min = TOUCH_X_MIN;
    s_cal.x_max = TOUCH_X_MAX;
    s_cal.y_min = TOUCH_Y_MIN;
    s_cal.y_max = TOUCH_Y_MAX;
    s_cal.x_inverted = TOUCH_X_INVERTED;
    s_cal.y_inverted = false;

    ESP_LOGI(TAG, "Calibration cleared from NVS");
    return ret;
}
