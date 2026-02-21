# devices/ - Hardware Board Definitions (Display Node)

## Purpose

Contains hardware-specific configuration headers for each display panel used
as the Display Node. Each header defines pin assignments, display timing
parameters, touch controller config, peripheral mappings, and capability flags.

## How It Works

The build includes exactly one device header based on the target display
panel. All component code references the generic `PIN_*`, `DISPLAY_*`, and
`HAS_*` defines rather than hard-coded values, making it easy to swap display
hardware without modifying application code.

```c
#include "dis08070h.h"

// All components use the same defines:
// DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_DATA_WIDTH, DISPLAY_BITS_PER_PX,
// BOUNCE_BUFFER_LINES, PIN_HSYNC, PIN_PANEL_ENABLE, SD_CS_PIN, etc.
```

## Display Panels

| File | Panel | Size | Resolution | MCU | Flash | PSRAM | Touch |
|------|-------|------|------------|-----|-------|-------|-------|
| `dis06043h.h` | CrowPanel ESP32 HMI 4.3" | 4.3" | 480×272 | ESP32-S3-N4R2 | 4MB | 2MB | Resistive (XPT2046 SPI) |
| `dis07050.h` | CrowPanel ESP32 HMI 5.0" | 5.0" | 800×480 | ESP32-S3-N4R8 | 4MB | 8MB | Capacitive (GT911 I2C) |
| `dis08070h.h` | CrowPanel ESP32 HMI 7.0" | 7.0" | 800×480 | ESP32-S3-N4R8 | 4MB | 8MB | Capacitive (GT911 I2C) |
| `waveshare_esp32s3_touch_lcd_2_1.h` | Waveshare 2.1" Round | 2.1" | 480×480 | ESP32-S3R8 | 16MB | 8MB (Octal) | Capacitive (CST820 I2C) |

## Common Features

All supported display boards share:
- **ESP32-S3 SoC** (no separate MCU board needed)
- **RGB parallel LCD interface** (16-bit RGB565)
- **SD card slot** (SPI or SDMMC)
- **Touch controller** (resistive or capacitive)
- **UART available** for inter-node communication

## Key Differences

### CrowPanel 4.3" (DIS06043H) — RECOMMENDED for CAN project
- **UART1 on dedicated HY2.0-4P connector** (GPIO17/18) — direct wire to CAN_Interface
- **Resistive touch** via XPT2046 (SPI) — GPIO 0, 11, 12, 13, 36
- **Lower PSRAM** (2MB) — sufficient for 480×272 display
- **No I2C touch conflict** — I2C pins (19/20) available for other uses

### CrowPanel 5.0" / 7.0" (DIS07050, DIS08070H)
- **Capacitive touch** via GT911 (I2C) on GPIO 19/20
- **UART conflict** — GPIO 17/18 used for I2S, must use UART0 for comm link
- **More PSRAM** (8MB) for larger framebuffers

### Waveshare 2.1" Round (waveshare_esp32s3_touch_lcd_2_1)
- **Round 480×480 IPS LCD** driven by ST7701S (SPI init → RGB data)
- **Capacitive touch** via CST820 (I2C, 0x15) — polling-based, no INT pin gating
- **SDMMC 1-bit** SD card (GPIO 1/2/42) — higher throughput than SPI
- **PWM backlight** via LEDC (GPIO 5)
- **TCA9554 GPIO expander** (I2C, 0x20) — LCD/touch/SD reset and enable
- **QMI8658 6-axis IMU** (I2C, 0x6B) — accelerometer + gyroscope
- **PCF85063 RTC** (I2C, 0x51, not yet implemented)
- **Shared I2C bus** (SDA=GPIO15, SCL=GPIO7, 400kHz) — 4 devices
- **Octal PSRAM** (8MB) with 16MB flash
- **UART0** (GPIO43/44) for comm link

## Pin Budget

Available pins for inter-node communication vary by board:

| Display | UART Comm | Pins | Notes |
|---------|-----------|------|-------|
| CrowPanel 4.3" | UART1 | GPIO17/18 | Dedicated HY2.0-4P connector |
| CrowPanel 5.0"/7.0" | UART0 | GPIO43/44 | Shared with USB-UART |
| Waveshare 2.1" | UART0 | GPIO43/44 | USB-CDC available for debug |

## Adding a New Display

1. Create a new `.h` file named after the display model
2. Define all required values — the display driver uses these directly with
   no hardcoded values:
   - Device Identification (`DEVICE_NAME`, `DEVICE_MODEL`)
   - Display Specifications (`DISPLAY_WIDTH`, `DISPLAY_HEIGHT`, `DISPLAY_COLOR_DEPTH`, `DISPLAY_DATA_WIDTH`, `DISPLAY_BITS_PER_PX`)
   - Driver Configuration (`BOUNCE_BUFFER_LINES` — must divide `DISPLAY_HEIGHT` evenly)
   - RGB Interface Pins (`PIN_R0-R4`, `PIN_G0-G5`, `PIN_B0-B4`, `PIN_HSYNC`, `PIN_VSYNC`, `PIN_DE`, `PIN_PCLK`)
   - Control Pins (`PIN_BACKLIGHT`, `PIN_PANEL_ENABLE` — use `-1` if not present)
   - Display Timing (`DISPLAY_FREQ_WRITE`, `HSYNC_*`, `VSYNC_*`, `PCLK_*`, `DE_*`)
   - Touch Configuration (SPI for XPT2046 or I2C for GT911)
   - Hardware Interface Pins (UART, I2S, SD Card)
   - Device Capabilities (`HAS_*`, `FLASH_SIZE_MB`, `PSRAM_SIZE_MB`)
     - `HAS_GPIO_EXPANDER`, `HAS_IMU`, `SD_USE_SDMMC`, `TOUCH_TYPE_*`
     - `BACKLIGHT_USE_LEDC`, `DISPLAY_INIT_SPI`, etc.
3. Update the build system to select the correct header

## Device Verification

Each new display panel should be verified:
- Flash and PSRAM sizes match header defines
- Display initializes and shows test pattern
- Touch controller responds (SPI for XPT2046 on 4.3", I2C for GT911 on 5"/7")
- Touch calibration screen works correctly
- SD card mount succeeds
- All declared capabilities are functional
