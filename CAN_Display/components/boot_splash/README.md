# boot_splash — SD Card Boot Splash Screen

## Purpose

Displays a BMP image from the SD card as a splash screen during boot. The splash remains visible while remaining subsystems initialize (comm_link, gauge_engine, etc.), providing visual feedback that the device is starting.

## Status: NEW

Component created, not yet tested on hardware.

## How It Works

```
SD card
  └── /images/splash.bmp
          │
          ▼
  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
  │ Read BMP hdr │ ──► │ Decode rows  │ ──► │ LVGL img obj │
  │ (validate)   │     │ → RGB565     │     │ on new screen│
  └──────────────┘     │ (PSRAM buf)  │     │ centered     │
                       └──────────────┘     └──────────────┘
                                                    │
                                               lv_scr_load()
                                                    │
                                            ┌───────▼───────┐
                                            │ Splash visible│
                                            │ during boot   │
                                            └───────┬───────┘
                                                    │
                                              ui_init() loads
                                              main UI screen
                                                    │
                                            ┌───────▼───────┐
                                            │ boot_splash_  │
                                            │ hide() frees  │
                                            │ PSRAM + screen │
                                            └───────────────┘
```

## Boot Sequence Integration

```c
// main.c — splash appears between SD mount and UI init

display_init();          // Phase 1: LVGL running
touch_init();            // Phase 2: Touch ready (SPI bus on CrowPanel)
logger_init();           // Phase 3: SD card mounted (moved up)
boot_splash_show();      // Phase 3.5: Splash visible
touch_calibration();     // Phase 4: Only first boot (over splash)
comm_link_init();        // Phase 5: UART
gauge_engine_init();     // Phase 6: Gauge state
ui_init();               // Phase 7: Main UI → replaces splash
boot_splash_hide();      // Phase 7.5: Free splash memory
ui_events_post_init();   // Phase 8: Timers, status
```

## BMP Format Support

| Depth | Format         | Notes                            |
|-------|----------------|----------------------------------|
| 16    | RGB565         | Direct copy, no conversion       |
| 24    | BGR888         | Standard from image editors      |
| 32    | BGRA8888       | Alpha channel discarded          |

- Uncompressed only (`BI_RGB`, compression=0)
- Bottom-up and top-down row order
- 4-byte row padding handled
- Max dimensions: 2048×2048 (sanity check)

## Image Recommendations

| Board               | Resolution | Format | File Size (24-bit) |
|---------------------|-----------|--------|---------------------|
| Waveshare 2.1"      | 480×480   | BMP    | ~691 KB             |
| CrowPanel 4.3"      | 480×272   | BMP    | ~392 KB             |

- **Round display** (Waveshare): Use black corners matching the circular bezel
- **Rectangular** (CrowPanel): Any aspect — centered on black background
- Save as **24-bit BMP** (most compatible) from any image editor

## Memory Usage

- **PSRAM**: width × height × 2 bytes (RGB565 output)
  - 480×480: 450 KB
  - 480×272: 255 KB
- **SRAM**: One BMP row during decode (≤ 1920 bytes for 480px @ 32bpp)
- All memory freed by `boot_splash_hide()`

## API

```c
esp_err_t boot_splash_show(void);   // Load BMP from SD, display as LVGL screen
void      boot_splash_hide(void);   // Delete screen, free PSRAM pixel buffer
```

## Error Handling

All errors are non-fatal — boot continues with a black screen:

| Return                | Meaning                               |
|-----------------------|---------------------------------------|
| `ESP_OK`              | Splash displayed successfully         |
| `ESP_ERR_NOT_FOUND`   | No file at /sdcard/images/splash.bmp  |
| `ESP_ERR_NO_MEM`      | Insufficient PSRAM                    |
| `ESP_ERR_NOT_SUPPORTED`| Compressed or unsupported BMP depth  |
| `ESP_FAIL`            | File I/O error                        |

## Dependencies

- `lvgl` — Image widget, screen management
- `display_driver` — Display lock/unlock for thread-safe LVGL access
- `devices` — Device header for display dimensions (informational)

## Files

```
boot_splash/
├── README.md           # This file
├── CMakeLists.txt      # Component build config
├── boot_splash.h       # Public API (show/hide)
└── boot_splash.c       # BMP decoder + LVGL display (~230 lines)
```
