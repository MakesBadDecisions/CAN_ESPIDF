# Touch Driver Component

Multi-controller touch input HAL for ESP32-S3 display boards. Supports both
resistive (XPT2046 SPI) and capacitive (CST820 I2C) touch controllers through
compile-time device selection. Designed to work alongside the RGB LCD bounce
buffer architecture without causing screen flicker.

## Supported Controllers

| Controller | Protocol | Boards | Calibration |
|------------|----------|--------|-------------|
| XPT2046 | SPI (1MHz) | CrowPanel 4.3"/5"/7" | 4-corner NVS |
| CST820 | I2C (400kHz) | Waveshare 2.1" | None (capacitive) |

The active controller is selected at compile time via `TOUCH_TYPE_XPT2046`
or `TOUCH_TYPE_CST820` defined in the device header.

## Architecture

### Core Isolation

The LCD bounce buffer DMA ISR runs on **Core 1** alongside LVGL. Performing
blocking I/O on Core 1 would stall the ISR and cause visible flicker.

Both backends run a dedicated **Core 0** FreeRTOS task (`touch_task`,
Priority 2, 4 KB stack) that polls the controller and writes results to a
volatile cache struct. The LVGL input device callback reads only the cache —
**zero I/O, instant return**.

```
Core 0                          Core 1
-----------                     -----------
touch_task                      lv_timer_handler()
  [XPT2046] SPI read              lvgl_touch_cb()
  [CST820]  I2C read                read s_touch_state (volatile)
  write s_touch_state               return immediately
```

### XPT2046 Backend (Resistive)
- SPI2_HOST, DMA disabled, 1MHz, Mode 0
- INT pin gating — only reads when touch detected
- Raw ADC mapped to screen coords with affine calibration
- SPI bus shared with SD card (separate CS pins)

### CST820 Backend (Capacitive)
- I2C on shared bus (via `i2c_bus` component), address 0x15
- Continuous polling every 15ms (no INT pin gating)
- TCA9554 EXIO2 used for hardware reset
- Native 480x480 coordinates — no calibration needed

## Calibration (XPT2046 Only)

Capacitive touch (CST820) reports native pixel coordinates and requires no
calibration. The sections below apply only to the XPT2046 resistive backend.

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

// Initialize touch controller, start polling task, register LVGL indev
// XPT2046: configures SPI bus and loads NVS calibration
// CST820:  issues TCA9554 reset, sends wake cmd, verifies chip ID via I2C
esp_err_t touch_init(void);

// Check if NVS has stored calibration (XPT2046 only, always true for CST820)
bool touch_has_calibration(void);

// Run interactive calibration (XPT2046 only, no-op for CST820)
esp_err_t touch_start_calibration(void);

// Erase NVS calibration (XPT2046 only, no-op for CST820)
esp_err_t touch_clear_calibration(void);
```

### Typical Boot Sequence

```c
display_init();           // RGB panel + LVGL
vTaskDelay(pdMS_TO_TICKS(100));

touch_init();             // SPI or I2C + task + LVGL indev

if (!touch_has_calibration()) {      // always true for CST820
    if (display_lock(5000)) {
        touch_start_calibration();   // no-op for CST820
        display_unlock();
    }
}
```

## Dependencies

- `driver` — SPI master (XPT2046), GPIO
- `esp_timer`
- `devices` — Pin definitions, controller selection macros
- `lvgl` — Input device registration
- `nvs_flash` — Calibration persistence (XPT2046)
- `display_driver` — Display lock for thread-safe LVGL access
- `i2c_bus` — Shared I2C bus (CST820)
- `tca9554` — GPIO expander for touch reset (CST820)
