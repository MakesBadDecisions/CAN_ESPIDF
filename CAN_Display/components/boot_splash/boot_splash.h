/**
 * @file boot_splash.h
 * @brief Boot splash screen from SD card BMP image
 *
 * Loads a BMP image from /sdcard/images/splash.bmp and displays it
 * as a full-screen LVGL image during boot. Supports 16-bit (RGB565),
 * 24-bit (RGB888), and 32-bit (ARGB8888) BMP files.
 *
 * Usage:
 *   1. Call boot_splash_show() after display_init() and logger_init()
 *   2. Splash remains visible during remaining boot phases
 *   3. Call boot_splash_hide() after ui_init() to free memory
 *
 * If no splash image is found on SD, boot_splash_show() returns
 * ESP_ERR_NOT_FOUND and boot continues normally with a black screen.
 */

#pragma once

#include "esp_err.h"

/** Default splash image path on SD card */
#define SPLASH_IMAGE_PATH       "/sdcard/images/splash.bmp"

/** Default minimum splash display time (ms), overridden by NVS */
#define SPLASH_DEFAULT_MS       3000

/** NVS namespace and key for splash duration */
#define SPLASH_NVS_NAMESPACE    "display"
#define SPLASH_NVS_KEY          "splash_ms"

/**
 * @brief Load and display splash image from SD card
 *
 * Reads BMP file, decodes to RGB565 in PSRAM, creates LVGL screen
 * and shows the image centered on a black background.
 *
 * @return ESP_OK              Splash displayed successfully
 * @return ESP_ERR_NOT_FOUND   No splash image on SD card (non-fatal)
 * @return ESP_ERR_NO_MEM      Insufficient PSRAM for image
 * @return ESP_FAIL            File I/O or decode error
 */
esp_err_t boot_splash_show(void);

/**
 * @brief Block until minimum splash display time has elapsed
 *
 * Call this BEFORE ui_init() so the splash remains visible on screen.
 * If enough time has already passed, returns immediately.
 * Safe to call even if boot_splash_show() was never called or failed.
 */
void boot_splash_wait(void);

/**
 * @brief Remove splash screen and free pixel buffer
 *
 * Safe to call even if boot_splash_show() was never called or failed.
 * Must be called after ui_init() has loaded the main UI screen.
 */
void boot_splash_hide(void);

/**
 * @brief Set splash display duration (persisted to NVS)
 * @param ms Duration in milliseconds (0 = skip splash wait)
 * @return ESP_OK on success
 */
esp_err_t boot_splash_set_duration(uint32_t ms);

/**
 * @brief Get current splash display duration
 * @return Duration in ms (reads NVS, falls back to SPLASH_DEFAULT_MS)
 */
uint32_t boot_splash_get_duration(void);

