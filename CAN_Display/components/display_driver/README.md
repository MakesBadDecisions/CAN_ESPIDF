# display_driver - RGB LCD + LVGL Display HAL

## Purpose

Initializes the ESP32-S3 RGB parallel LCD panel and LVGL graphics library for
CrowPanel displays. Provides a thread-safe LVGL environment with an anti-tearing
double-framebuffer architecture.

Touch input is handled by the separate `touch_driver` component.

## Architecture

```
┌────────────────────────────────┐
│     ui / gauge_engine          │
│  (LVGL widgets and screens)    │
└──────────────┬─────────────────┘
               │  display_lock() / display_unlock()
┌──────────────┴─────────────────┐
│     LVGL v8.3 (direct_mode)    │
│  - Draws into PSRAM framebuffers│
│  - Timer handler on Core 1     │
└──────────────┬─────────────────┘
               │  lvgl_flush_cb()
┌──────────────┴─────────────────┐
│     esp_lcd_panel_rgb           │
│  - Double FB in PSRAM (num_fbs=2)│
│  - Bounce buffers in SRAM       │
│  - VSYNC-synced buffer swap     │
│  - DMA to RGB parallel LCD      │
└─────────────────────────────────┘
```

## Anti-Tearing Strategy

Uses the Espressif-recommended approach for flicker-free RGB LCD rendering:

1. **Double framebuffer in PSRAM** (`num_fbs=2`): DMA reads from FB-A while
   LVGL writes to FB-B. They never access the same buffer simultaneously.

2. **LVGL direct_mode**: LVGL draws directly into the PSRAM framebuffers
   (obtained via `esp_lcd_rgb_panel_get_frame_buffer()`), avoiding an extra
   memcpy from a separate draw buffer.

3. **Bounce buffers in SRAM**: Small SRAM staging buffers (`BOUNCE_BUFFER_LINES`
   lines, defined per device) between PSRAM and the LCD DMA peripheral.
   Required because the LCD peripheral cannot read directly from PSRAM at
   the sustained rate needed.

4. **VSYNC-synced buffer swap**: The flush callback only swaps framebuffers
   on the last flush of a render cycle, after waiting for VSYNC.

5. **Dirty area synchronization**: After swapping, all invalidated regions
   are copied from the drawn buffer to the other buffer. This prevents stale
   pixels when LVGL switches buffers (LVGL only draws dirty regions in
   direct_mode, so the other buffer would otherwise have old content).

## Required sdkconfig Settings

These are critical for the bounce buffer + PSRAM architecture to work:

| Setting | Value | Why |
|---------|-------|-----|
| `CONFIG_SPIRAM_SPEED_80M` | y | 40 MHz is insufficient bandwidth for 480x272@60fps |
| `CONFIG_ESP32S3_DATA_CACHE_LINE_64B` | y | Required by ESP-IDF for bounce buffer DMA alignment |
| `CONFIG_LCD_RGB_ISR_IRAM_SAFE` | y | LCD DMA ISR must run from IRAM |
| `CONFIG_SPIRAM_FETCH_INSTRUCTIONS` | y | Reduces flash/PSRAM bus contention |
| `CONFIG_SPIRAM_RODATA` | y | Reduces flash/PSRAM bus contention |

**Warning:** `sdkconfig.defaults` only applies when no sdkconfig exists. After
changing defaults, run `fullclean` to regenerate.

## API

```c
#include "display_driver.h"

esp_err_t display_init(void);            // Init RGB panel + LVGL + task
void      display_clear(uint16_t color); // Set background color (RGB565)
void      display_flush(void);           // No-op (LVGL auto-flushes)
bool      display_lock(uint32_t ms);     // Acquire LVGL mutex
void      display_unlock(void);          // Release LVGL mutex
```

All LVGL API calls from outside the LVGL task must be wrapped in
`display_lock()` / `display_unlock()`.

## FreeRTOS Tasks

| Task | Core | Priority | Stack | Purpose |
|------|------|----------|-------|---------|
| `lvgl` | 1 | 3 | 16 KB | Runs `lv_timer_handler()` every 10 ms |

## Device Selection

The display panel is selected at compile time via a `DEVICE_DIS*` define.
Each device header (in `components/devices/`) provides all pin assignments,
timing parameters, display dimensions, and driver configuration:

| Define | Example (4.3") | Purpose |
|--------|----------------|---------|
| `DISPLAY_WIDTH` | 480 | Horizontal resolution |
| `DISPLAY_HEIGHT` | 272 | Vertical resolution |
| `DISPLAY_DATA_WIDTH` | 16 | RGB parallel bus width (bits) |
| `DISPLAY_BITS_PER_PX` | 16 | Bits per pixel (RGB565) |
| `BOUNCE_BUFFER_LINES` | 8 | Bounce buffer height (must divide `DISPLAY_HEIGHT` evenly) |
| `PIN_PANEL_ENABLE` | 38 / -1 | Panel enable GPIO (-1 if not present) |
| `PIN_BACKLIGHT` | 2 | Backlight GPIO |
| `DISPLAY_FREQ_WRITE` | 7000000 | Pixel clock (Hz) |
| `HSYNC_*`, `VSYNC_*` | — | Panel timing parameters |
| `PIN_R0-R4`, `PIN_G0-G5`, `PIN_B0-B4` | — | RGB data pins |

## Files

```
display_driver/
├── README.md
├── CMakeLists.txt
├── display_driver.h    # Public API
└── display_driver.c    # RGB panel init, LVGL init, flush callback, task
```

## Dependencies

- `esp_lcd` — RGB panel driver
- `driver` — GPIO (backlight, panel enable)
- `esp_timer`
- `devices` — Pin definitions and timing parameters
- `lvgl` — Graphics library
