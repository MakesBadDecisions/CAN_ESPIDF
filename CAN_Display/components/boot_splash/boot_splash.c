/**
 * @file boot_splash.c
 * @brief Boot splash screen — BMP loader and LVGL display
 *
 * Decodes a BMP file from the mounted SD card and renders it as a
 * centered LVGL image on a black background screen. The entire BMP
 * pixel data is bulk-read into a PSRAM temp buffer in one fread(),
 * then decoded row-by-row to RGB565 — fast SD throughput, no per-row
 * I/O overhead.
 *
 * BMP format support:
 *   - 16-bit (RGB565) — direct copy, no conversion
 *   - 24-bit (BGR888) — standard BMP from image editors
 *   - 32-bit (BGRA8888) — alpha channel ignored
 *   - Bottom-up and top-down row order
 *   - 4-byte row padding handled automatically
 *
 * Memory layout (peak during decode, temp freed after):
 *   ┌──────────────┐
 *   │ raw_buf      │  ← PSRAM, full BMP pixel data (freed after decode)
 *   ├──────────────┤
 *   │ s_pixel_buf  │  ← PSRAM, full RGB565 image (freed on hide)
 *   ├──────────────┤
 *   │ lv_img obj   │  ← LVGL heap (freed on hide)
 *   └──────────────┘
 */

#include "boot_splash.h"
#include "display_driver.h"
#include "device.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "boot_splash";

/* ── BMP structures ─────────────────────────────────────────────── */

/** BMP file header — 14 bytes, always at offset 0 */
typedef struct __attribute__((packed)) {
    uint16_t type;          /**< Magic 'BM' (0x4D42)              */
    uint32_t file_size;     /**< Total file size in bytes          */
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t data_offset;   /**< Byte offset to pixel data         */
} bmp_file_header_t;

/** BITMAPINFOHEADER — 40 bytes, immediately after file header */
typedef struct __attribute__((packed)) {
    uint32_t header_size;   /**< Must be 40 for BITMAPINFOHEADER   */
    int32_t  width;         /**< Image width in pixels             */
    int32_t  height;        /**< Positive=bottom-up, negative=top  */
    uint16_t planes;        /**< Must be 1                         */
    uint16_t bpp;           /**< Bits per pixel: 16, 24, or 32     */
    uint32_t compression;   /**< 0=BI_RGB (uncompressed)           */
    uint32_t image_size;    /**< May be 0 for BI_RGB               */
    int32_t  x_ppm;         /**< Horizontal pixels per meter       */
    int32_t  y_ppm;         /**< Vertical pixels per meter         */
    uint32_t colors_used;
    uint32_t colors_important;
} bmp_info_header_t;

/* ── Module state ───────────────────────────────────────────────── */

static lv_obj_t    *s_splash_screen = NULL;   /**< Splash LVGL screen      */
static lv_img_dsc_t s_img_dsc;                /**< LVGL image descriptor   */
static uint8_t     *s_pixel_buf = NULL;        /**< RGB565 pixels in PSRAM  */
static int64_t      s_show_time_us = 0;        /**< Timestamp when splash was displayed */

/* ── NVS helpers ────────────────────────────────────────────────── */

uint32_t boot_splash_get_duration(void)
{
    nvs_handle_t nvs;
    uint32_t ms = SPLASH_DEFAULT_MS;
    if (nvs_open(SPLASH_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, SPLASH_NVS_KEY, &ms);
        nvs_close(nvs);
    }
    return ms;
}

esp_err_t boot_splash_set_duration(uint32_t ms)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(SPLASH_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u32(nvs, SPLASH_NVS_KEY, ms);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Splash duration set to %lu ms", (unsigned long)ms);
    return err;
}

/* ── Helpers ────────────────────────────────────────────────────── */

/** Convert 8-bit RGB components to RGB565 */
static inline uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

/**
 * @brief Decode one BMP row into the RGB565 output buffer
 *
 * @param src       Raw BMP row bytes (BGR order for 24/32 bpp)
 * @param dst       Destination RGB565 row
 * @param width     Pixels per row
 * @param bpp       Source bits per pixel (16, 24, 32)
 */
static void decode_row(const uint8_t *src, uint16_t *dst,
                       int32_t width, uint16_t bpp)
{
    switch (bpp) {
    case 16:
        /* RGB565 — direct copy */
        memcpy(dst, src, width * 2);
        break;

    case 24:
        /* BGR888 → RGB565 */
        for (int32_t x = 0; x < width; x++) {
            uint8_t b = src[x * 3 + 0];
            uint8_t g = src[x * 3 + 1];
            uint8_t r = src[x * 3 + 2];
            dst[x] = rgb888_to_rgb565(r, g, b);
        }
        break;

    case 32:
        /* BGRA8888 → RGB565 (alpha discarded) */
        for (int32_t x = 0; x < width; x++) {
            uint8_t b = src[x * 4 + 0];
            uint8_t g = src[x * 4 + 1];
            uint8_t r = src[x * 4 + 2];
            dst[x] = rgb888_to_rgb565(r, g, b);
        }
        break;

    default:
        memset(dst, 0, width * 2);
        break;
    }
}

/* ── Public API ─────────────────────────────────────────────────── */

esp_err_t boot_splash_show(void)
{
    /* --- Check for splash image on SD -------------------------------- */
    struct stat st;
    if (stat(SPLASH_IMAGE_PATH, &st) != 0) {
        ESP_LOGW(TAG, "No splash image at %s", SPLASH_IMAGE_PATH);
        return ESP_ERR_NOT_FOUND;
    }

    FILE *f = fopen(SPLASH_IMAGE_PATH, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s", SPLASH_IMAGE_PATH);
        return ESP_FAIL;
    }

    /* --- Read and validate BMP headers ------------------------------- */
    bmp_file_header_t fh;
    bmp_info_header_t ih;

    if (fread(&fh, 1, sizeof(fh), f) != sizeof(fh) ||
        fread(&ih, 1, sizeof(ih), f) != sizeof(ih)) {
        ESP_LOGE(TAG, "Failed to read BMP headers");
        fclose(f);
        return ESP_FAIL;
    }

    if (fh.type != 0x4D42) {   /* 'BM' */
        ESP_LOGE(TAG, "Not a BMP file (magic: 0x%04X)", fh.type);
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }

    if (ih.compression != 0) {
        ESP_LOGE(TAG, "Compressed BMP not supported (compression=%lu)",
                 (unsigned long)ih.compression);
        fclose(f);
        return ESP_ERR_NOT_SUPPORTED;
    }

    bool top_down  = (ih.height < 0);
    int32_t width  = ih.width;
    int32_t height = top_down ? -ih.height : ih.height;

    ESP_LOGI(TAG, "Splash BMP: %ldx%ld, %d bpp, %s",
             (long)width, (long)height, ih.bpp,
             top_down ? "top-down" : "bottom-up");

    if (ih.bpp != 16 && ih.bpp != 24 && ih.bpp != 32) {
        ESP_LOGE(TAG, "Unsupported BMP depth: %d bpp (need 16, 24, or 32)",
                 ih.bpp);
        fclose(f);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (width <= 0 || height <= 0 || width > 2048 || height > 2048) {
        ESP_LOGE(TAG, "Invalid BMP dimensions: %ldx%ld", (long)width, (long)height);
        fclose(f);
        return ESP_ERR_INVALID_ARG;
    }

    /* --- Allocate RGB565 output buffer in PSRAM ---------------------- */
    size_t out_size = (size_t)width * height * 2;
    s_pixel_buf = heap_caps_malloc(out_size, MALLOC_CAP_SPIRAM);
    if (!s_pixel_buf) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes in PSRAM for splash", out_size);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    /* --- Bulk-read entire BMP pixel data into PSRAM temp buffer ------- */
    fseek(f, fh.data_offset, SEEK_SET);

    int32_t src_stride = ((width * (ih.bpp / 8) + 3) & ~3);
    size_t raw_size = (size_t)src_stride * height;
    uint8_t *raw_buf = heap_caps_malloc(raw_size, MALLOC_CAP_SPIRAM);
    if (!raw_buf) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for BMP raw data", raw_size);
        heap_caps_free(s_pixel_buf);
        s_pixel_buf = NULL;
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t bytes_read = fread(raw_buf, 1, raw_size, f);
    fclose(f);

    if (bytes_read != raw_size) {
        ESP_LOGE(TAG, "BMP read incomplete: got %zu of %zu bytes", bytes_read, raw_size);
        heap_caps_free(raw_buf);
        heap_caps_free(s_pixel_buf);
        s_pixel_buf = NULL;
        return ESP_FAIL;
    }

    /* --- Decode rows from memory (fast — no I/O per row) -------------- */
    for (int32_t y = 0; y < height; y++) {
        int32_t dst_y = top_down ? y : (height - 1 - y);
        const uint8_t *src_row = raw_buf + (size_t)y * src_stride;
        uint16_t *dst_row = (uint16_t *)(s_pixel_buf + dst_y * width * 2);
        decode_row(src_row, dst_row, width, ih.bpp);
    }

    heap_caps_free(raw_buf);

    ESP_LOGI(TAG, "BMP decoded (%zu bytes RGB565 in PSRAM)", out_size);

    /* --- Create LVGL image descriptor -------------------------------- */
    s_img_dsc.header.always_zero = 0;
    s_img_dsc.header.w  = width;
    s_img_dsc.header.h  = height;
    s_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    s_img_dsc.data_size = out_size;
    s_img_dsc.data      = s_pixel_buf;

    /* --- Create and load splash screen ------------------------------- */
    if (!display_lock(2000)) {
        ESP_LOGE(TAG, "Failed to acquire display lock for splash");
        heap_caps_free(s_pixel_buf);
        s_pixel_buf = NULL;
        return ESP_FAIL;
    }

    s_splash_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_splash_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_splash_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_splash_screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *img = lv_img_create(s_splash_screen);
    lv_img_set_src(img, &s_img_dsc);
    lv_obj_center(img);

    lv_scr_load(s_splash_screen);

    display_unlock();

    s_show_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Boot splash displayed (%ldx%ld)", (long)width, (long)height);
    return ESP_OK;
}

void boot_splash_wait(void)
{
    if (s_show_time_us == 0) {
        return;     /* Splash was never shown */
    }

    uint32_t duration_ms = boot_splash_get_duration();
    int64_t elapsed_ms = (esp_timer_get_time() - s_show_time_us) / 1000;
    int64_t remain_ms  = (int64_t)duration_ms - elapsed_ms;
    if (remain_ms > 0) {
        ESP_LOGI(TAG, "Holding splash for %lld ms more (configured: %lu ms)",
                 (long long)remain_ms, (unsigned long)duration_ms);
        vTaskDelay(pdMS_TO_TICKS((uint32_t)remain_ms));
    }
}

void boot_splash_hide(void)
{
    if (!s_splash_screen && !s_pixel_buf) {
        return;     /* Nothing to clean up */
    }

    if (s_splash_screen) {
        if (display_lock(1000)) {
            /*
             * ui_init() has already loaded the main UI screen, so
             * s_splash_screen is no longer the active display. Safe
             * to delete it and all its children.
             */
            lv_obj_del(s_splash_screen);
            s_splash_screen = NULL;
            display_unlock();
        } else {
            ESP_LOGW(TAG, "Display lock timeout — splash screen leaked");
        }
    }

    if (s_pixel_buf) {
        heap_caps_free(s_pixel_buf);
        s_pixel_buf = NULL;
        ESP_LOGI(TAG, "Splash memory freed");
    }
}
