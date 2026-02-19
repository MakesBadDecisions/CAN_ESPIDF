# Touch Driver Component

XPT2046 resistive touch driver for CrowPanel ESP32-S3 displays, designed to
work alongside the RGB LCD bounce buffer architecture without causing screen
flicker.

## Architecture

### Core Isolation

The LCD bounce buffer DMA ISR runs on **Core 1** alongside LVGL. Performing
blocking SPI reads on Core 1 would stall the ISR and cause visible flicker.

This driver solves the problem by running all SPI communication on a dedicated
**Core 0** FreeRTOS task (`touch_task`, Priority 2, 4 KB stack). The task
polls the XPT2046 every 10 ms and writes results to a volatile cache struct.

The LVGL input device callback (`lvgl_touch_cb`) runs on Core 1 inside
`lv_timer_handler()` and only reads the cache — **zero SPI, instant return**.

```
Core 0                          Core 1
-----------                     -----------
touch_task                      lv_timer_handler()
  poll INT pin                    lvgl_touch_cb()
  SPI read Z1/X/Y                  read s_touch_state (volatile)
  map to screen coords             return immediately
  write s_touch_state
```

### SPI Configuration

- Host: `SPI2_HOST`
- DMA: Disabled (small 3-byte transfers; avoids PSRAM bus contention)
- Clock: 1 MHz
- Mode: 0 (CPOL=0, CPHA=0)
- Pins defined in the device header (`dis06043h.h`, etc.)

## Calibration

### 4-Corner Calibration Screen

`touch_start_calibration()` creates a temporary LVGL screen with a red
crosshair (+) at each corner. The user touches each crosshair in sequence:

1. Top-left
2. Top-right
3. Bottom-right
4. Bottom-left

For each point, 5 stable readings are averaged. The driver auto-detects axis
inversion by comparing raw ADC values to expected screen positions.

### NVS Persistence

Calibration data is stored in NVS namespace `"touch_cal"`:

| Key     | Type | Description                          |
|---------|------|--------------------------------------|
| `x_min` | u16  | Minimum raw X ADC value              |
| `x_max` | u16  | Maximum raw X ADC value              |
| `y_min` | u16  | Minimum raw Y ADC value              |
| `y_max` | u16  | Maximum raw Y ADC value              |
| `x_inv` | u8   | X axis inverted (1=yes, 0=no)        |
| `y_inv` | u8   | Y axis inverted (1=yes, 0=no)        |

On boot, `touch_init()` loads from NVS. If no data exists, it falls back to
hardcoded defaults from the device header.

## API

```c
#include "touch_driver.h"

// Initialize SPI, start polling task, register LVGL indev
esp_err_t touch_init(void);

// Check if NVS has stored calibration
bool touch_has_calibration(void);

// Run interactive calibration (caller must hold display lock)
esp_err_t touch_start_calibration(void);

// Erase NVS calibration (forces recalibration on next boot)
esp_err_t touch_clear_calibration(void);
```

### Typical Boot Sequence

```c
display_init();           // RGB panel + LVGL
vTaskDelay(pdMS_TO_TICKS(100));

touch_init();             // SPI + task + LVGL indev

if (!touch_has_calibration()) {
    if (display_lock(5000)) {
        touch_start_calibration();
        display_unlock();
    }
}
```

## Dependencies

- `driver` — SPI master, GPIO
- `esp_timer`
- `devices` — Pin definitions (dis06043h.h, etc.)
- `lvgl` — Input device registration
- `nvs_flash` — Calibration persistence
- `display_driver` — Display lock for thread-safe LVGL access
