# CAN_ESPIDF - Display Node

## Purpose

The Display Node is one of two ESP32-S3 nodes in the CAN_ESPIDF system. It receives real-time decoded PID data from the CAN Interface Node over UART via a USB-C cable, renders live gauges on an RGB parallel LCD with touch, logs all data to SD card in HP Tuners-compatible CSV format, and hosts a WiFi access point serving a full web-based configuration portal.

The web portal is the central hub for the entire system — display settings, CAN Interface settings, extended poll lists, custom PID definitions, math channels, DTC management, log file browsing/download, vehicle profiles, and hardware addon configuration (BME/BMP sensors, gyro, QST attitude sensor, buzzers, RGB LEDs).

This node does NOT interact with the vehicle CAN bus directly. All vehicle data arrives from the CAN Interface Node over a wired UART link (USB-C cable). The Display Node is the only node in the system with WiFi, serving as the user-facing hub for all configuration and data access.

Two display boards are currently supported:

| Board | Display | Resolution | Shape | Touch | SD Card | MCU |
|-------|---------|------------|-------|-------|---------|-----|
| **Elecrow CrowPanel 4.3" (DIS06043H)** | RGB parallel | 480x272 | Rectangle | XPT2046 SPI resistive | SPI (shared bus) | ESP32-S3-N4R2 (4MB/2MB) |
| **Waveshare ESP32-S3-Touch-LCD-2.1** | ST7701S SPI+RGB | 480x480 | Round | CST820 I2C capacitive | SDMMC 1-bit | ESP32-S3-R8 (16MB/8MB) |

The Waveshare board also includes onboard peripherals: TCA9554 GPIO expander, QMI8658 6-axis IMU, PCF85063 RTC, and buzzer — all on a shared I2C bus.

### Responsibilities

- Receive and parse UART messages from the CAN Interface Node
- Maintain a local PID data store with the latest values and timestamps
- Render configurable gauge layouts on the CrowPanel RGB parallel LCD
- Provide touch screen interaction for local configuration
- Monitor critical values and trigger visual/audible alerts (overtemp, low oil pressure, etc.)
- Log all received PID data to SD card as HP Tuners-compatible CSV
- Manage logging sessions (start/stop, file naming, file rotation)
- Host a WiFi AP with full web configuration portal:
  - Display settings (gauge layout, units, alerts, backlight, themes)
  - CAN Interface settings (bitrate, controller, filters, poll rates) — proxied over UART
  - Extended poll list management (poll more PIDs than displayed gauges)
  - Custom PID editor (user-defined addresses, formulas, units, Mode 0x22)
  - Custom math channels (virtual channels derived from real PIDs)
  - Log file browsing, download, and management
  - DTC read/clear, freeze frames, ECU info
  - Vehicle profiles (per-VIN config save/load/auto-detect)
  - Hardware addon config (BME/BMP sensors, gyro, QST attitude, buzzers, RGB LEDs)
- Read hardware addon sensors (environmental, motion) and merge into data stream
- Send configuration commands back to the CAN Interface Node over UART TX

## Component Overview

| Component | Directory | Purpose |
|-----------|-----------|---------|
| comm_link | `components/comm_link/` | UART receive handler, message parsing, connection health monitoring, PID data store |
| devices | `components/devices/` | Per-device pin definitions and hardware config for each display board |
| display_driver | `components/display_driver/` | RGB parallel LCD driver, LVGL integration, double-FB anti-tearing, ST7701S SPI init, PWM backlight, brightness API with NVS persistence |
| touch_driver | `components/touch_driver/` | Touch input HAL — XPT2046 SPI resistive or CST820 I2C capacitive (compile-time selection) |
| ui | `components/ui/` | SquareLine Studio generated LVGL screens, widgets, and custom event logic (settings, theme coloring, brightness, colorwheel) |
| gauge_engine | `components/gauge_engine/` | Gauge data manager -- per-slot PID assignment, unit conversion, poll list aggregation, NVS persistence |
| data_logger | `components/data_logger/` | SD card CSV logging (SPI or SDMMC), session management, write buffering |
| i2c_bus | `components/i2c_bus/` | Shared I2C bus manager for onboard peripherals (Waveshare) |
| tca9554 | `components/tca9554/` | TCA9554 I2C GPIO expander — LCD/touch reset, SPI CS, SD D3, buzzer |
| qmi8658 | `components/qmi8658/` | QMI8658 6-axis IMU — accel + gyro, orientation fusion, NVS calibration |
| imu_display | `components/imu_display/` | IMU bubble visualization — pitch/roll dot + G-force labels in LVGL panel |
| wifi_manager | `components/wifi_manager/` | WiFi AP, HTTP server, REST API, full web config portal |
| system | `components/system/` | Logging wrapper, NVS init, timing utilities |

Shared definitions used by both nodes live in `../shared/`:

| Shared Component | Directory | Purpose |
|------------------|-----------|---------|
| comm_protocol | `shared/comm_protocol/` | UART binary message format definitions |
| pid_types | `shared/pid_types/` | PID data types, enums, unit definitions |
| utils | `shared/utils/` | Common utility functions |

## Project Structure

```
CAN_Display/
├── README.md
├── TODO.md
├── CMakeLists.txt
├── platformio.ini
├── sdkconfig.defaults
├── sdkconfig.waveshare_2_1
├── partitions.csv
├── partitions_16mb.csv
├── boards/
│   ├── crowpanel-esp32s3-n4r2.json
│   └── waveshare-esp32s3-touch-lcd-2-1.json
├── main/
│   ├── main.c
│   └── CMakeLists.txt
└── components/
    ├── comm_link/          # UART RX/TX, PID data store
    ├── devices/            # Per-board pin definitions (DIS06043H, Waveshare 2.1")
    ├── display_driver/     # RGB parallel LCD + LVGL (ST7701S SPI init, PWM backlight)
    ├── touch_driver/       # Touch HAL (XPT2046 SPI or CST820 I2C)
    ├── ui/                 # SquareLine Studio generated LVGL UI
    ├── gauge_engine/       # Gauge data manager (PID state, unit conversion, NVS)
    ├── data_logger/        # SD card CSV logging (SPI or SDMMC)
    ├── i2c_bus/            # Shared I2C bus manager
    ├── tca9554/            # TCA9554 GPIO expander driver
    ├── qmi8658/            # QMI8658 6-axis IMU driver (fusion, NVS cal)
    ├── imu_display/        # IMU bubble visualization for LVGL
    ├── wifi_manager/       # WiFi AP, REST API, web config portal
    ├── system/             # Logging, NVS, timing
    └── lvgl/               # LVGL v8.3 library
```

## FreeRTOS Task Layout

Tasks currently running in the firmware:

| Task | Core | Priority | Stack | Purpose |
|------|------|----------|-------|---------|
| `lvgl` | 1 | 3 | 16 KB | LVGL timer handler (rendering + input processing) |
| `touch` | 0 | 2 | 4 KB | Touch polling (XPT2046 SPI or CST820 I2C), writes to volatile cache |
| `comm_rx` | 0 | 4 | 4 KB | UART RX from CAN Interface Node |
| `imu_task` | 0 | 3 | 4 KB | IMU polling (50Hz), orientation fusion, calibration |

Core 1 is dedicated to display rendering (LVGL + LCD bounce buffer DMA ISR).
Touch reads are isolated on Core 0 to prevent stalling the LCD DMA ISR.

### Future Tasks (not yet implemented)

| Task | Core | Priority | Purpose |
|------|------|----------|---------|
| WiFi/Config | 0 | 1 | HTTP server for log download and config |
| System Monitor | 0 | 0 | Heap usage, watchdog, health reporting |

Data logging runs inline in the LVGL timer callback (gauge_update_cb) — no separate task needed. Buffered writes (4KB) minimize SD card I/O overhead.

## Pin Assignments

### CrowPanel 4.3" (DIS06043H)

#### RGB LCD (16-bit parallel)
All 16 data lines + HSYNC/VSYNC/DE/PCLK — see `dis06043h.h` for full list.

#### Touch (XPT2046 SPI)
| Signal | GPIO | Notes |
|--------|------|-------|
| SCK | 12 | Shared SPI bus with SD card |
| MOSI | 11 | Shared SPI bus with SD card |
| MISO | 13 | Shared SPI bus with SD card |
| CS | 0 | Touch chip select |
| INT | 36 | Touch interrupt (active low) |

#### UART - Inter-Node Communication
| Signal | GPIO | Notes |
|--------|------|-------|
| UART1 TX | 17 | To CAN Interface Node RX (GPIO18) |
| UART1 RX | 18 | From CAN Interface Node TX (GPIO17) |

#### SD Card (SPI, shared bus with touch)
| Signal | GPIO |
|--------|------|
| MOSI | 11 |
| MISO | 13 |
| SCLK | 12 |
| CS | 10 |

#### Backlight / Panel Enable
| Signal | GPIO | Notes |
|--------|------|-------|
| Backlight | 2 | Active high (GPIO on/off) |
| Panel Enable | 38 | Must be HIGH for display to work |

### Waveshare ESP32-S3-Touch-LCD-2.1

#### RGB LCD (16-bit parallel) + ST7701S SPI Init
RGB data pins + HSYNC/VSYNC/DE/PCLK — see `ws_touch_lcd_2_1.h` for full list.
ST7701S requires 9-bit SPI initialization sequence before RGB panel starts.

| Signal | GPIO/Method | Notes |
|--------|-------------|-------|
| SPI MOSI | 1 | Shared with SD CMD (used only during init) |
| SPI SCLK | 2 | Shared with SD CLK (used only during init) |
| SPI CS | TCA9554 EXIO3 | Via GPIO expander |
| LCD Reset | TCA9554 EXIO1 | Via GPIO expander |
| Backlight | 6 | PWM via LEDC (13-bit, 5kHz) |

#### Touch (CST820 I2C Capacitive)
| Signal | GPIO/Method | Notes |
|--------|-------------|-------|
| I2C SDA | 15 | Shared I2C bus (400kHz) |
| I2C SCL | 7 | Shared I2C bus |
| INT | 16 | Touch interrupt (unused — polling-based) |
| Reset | TCA9554 EXIO2 | Via GPIO expander |

#### UART - Inter-Node Communication
| Signal | GPIO | Notes |
|--------|------|-------|
| UART TX | 43 | Via USB-C or direct wire |
| UART RX | 44 | Via USB-C or direct wire |

#### SD Card (SDMMC 1-bit)
| Signal | GPIO | Notes |
|--------|------|-------|
| CLK | 2 | Shared with LCD SPI SCLK (SPI freed after init) |
| CMD | 1 | Shared with LCD SPI MOSI (SPI freed after init) |
| D0 | 42 | Data line |
| D3 | TCA9554 EXIO4 | Must be HIGH for 1-bit mode |

#### I2C Bus (shared, 400kHz)
| Address | Device | Purpose |
|---------|--------|---------|
| 0x15 | CST820 | Capacitive touch controller |
| 0x20 | TCA9554 | GPIO expander |
| 0x51 | PCF85063 | Real-time clock |
| 0x6B | QMI8658 | 6-axis IMU |

## Boot Sequence

The actual boot sequence as implemented in `main.c`:

```
0. system_init()
   - Initialize NVS flash
   - Print device info (chip, flash, PSRAM, MAC)

0.5 I2C bus + peripherals (if present)
   - i2c_bus_init() — shared I2C bus for touch, GPIO expander, IMU, RTC
   - tca9554_init() — GPIO expander (LCD/touch reset, SPI CS, SD D3)
   - qmi8658_init() — 6-axis IMU (non-fatal if not present)
   - qmi8658_start_task() — background polling + orientation fusion
     - Loads saved calibration from NVS, or performs 2s live calibration
     - Saves new calibration to NVS on completion

1. display_init()
   - [Waveshare] ST7701S SPI init (39 commands), PWM backlight via LEDC
     - Backlight NVS restore: loads brightness % from "display"/"bl_pct"
   - [CrowPanel] Backlight + panel enable GPIOs
   - Initialize RGB parallel LCD panel (double FB, bounce buffers, VSYNC)
   - Initialize LVGL (direct_mode, register display driver)
   - Create LVGL task on Core 1
   - 100ms delay for LVGL task startup

2. touch_init()
   - [CrowPanel] Initialize SPI bus for XPT2046, start polling task
   - [Waveshare] TCA9554 touch reset, CST820 I2C wake, start polling task
   - Register LVGL input device

3. logger_init()
   - [CrowPanel] Mount SD card on SPI bus (FAT filesystem)
   - [Waveshare] Mount SD card via SDMMC 1-bit (TCA9554 D3 enable)
   - Non-fatal: continues without logging if SD card missing

3.5. boot_splash_show()
   - Load /sdcard/images/splash.bmp → display during subsequent init phases
   - Visible during phases 4-6

4. Touch calibration check (resistive only)
   - If no NVS calibration data: show 4-corner calibration screen over splash
   - Capacitive touch skips this (no calibration needed)

5. comm_link_init() + comm_link_start()
   - Initialize UART peripheral, start RX/TX tasks

6. gauge_engine_init()
   - Zero all gauge slots, create mutex

7. ui_init()
   - Load SquareLine Studio generated UI screens (replaces splash)
   - boot_splash_hide() — free PSRAM splash buffer

8. ui_events_post_init()
   - Register settingsButton callback on ui_settingsButton (SquareLine-safe)
   - Create status label and start status polling timer
   - Load theme color from NVS ("display"/"theme")
   - Apply theme color to all widgets on Screen1 (text + border colors)
```

Non-critical failures (touch, SD card) are logged but do not halt boot.

## NVS Namespaces

All persistent settings are stored in ESP-IDF NVS (Non-Volatile Storage):

| Namespace | Key | Type | Default | Component |
|-----------|-----|------|---------|-----------|
| `imu_cal` | `bias` | blob | — | qmi8658 (gyro bias vector) |
| `imu_cal` | `R_matrix` | blob | — | qmi8658 (Rodrigues rotation) |
| `imu_cal` | `valid` | u8 | 0 | qmi8658 (calibration flag) |
| `gauge_cfg` | `slot0`..`slot19` | blob | — | gauge_engine (pid_id + unit per slot) |
| `vehicle` | `vin` | str | "" | comm_link (last known VIN) |
| `vehicle` | `ecu_count` | u8 | 0 | comm_link |
| `touch_cal` | `x_min`, `x_max`, etc. | u16 | header defaults | touch_driver |
| `logger` | `file_counter` | u32 | 0 | data_logger (sequential filename) |
| `display` | `splash_ms` | u32 | 3000 | boot_splash (splash duration ms) |
| `display` | `bl_pct` | u8 | 100 | display_driver (backlight brightness %) |
| `display` | `theme` | u16 | 0x34DB | ui_events (theme color RGB565) |

## Settings Screen (Screen2)

The Settings Screen is accessed via the settings button on Screen1. It provides:

- **Brightness Slider**: Adjusts PWM backlight (Waveshare) or GPIO on/off (CrowPanel). Value persisted to NVS immediately on change.
- **Color Wheel**: Dynamic LVGL colorwheel for selecting theme color. Hex label (#RRGGBB) shows current color in the center. Saved to NVS on every drag.
- **Theme Coloring**: Selected color is applied as text color to all labels and border color to all interactive widgets (buttons, panels, gauge borders, dropdowns) across both Screen1 and Screen2.
- **WiFi AP Button**: Placeholder for WiFi Manager activation (not yet implemented).
- **System Panel**: Placeholder for system info display.

### SquareLine Integration Notes 

- Screen2 widgets are destroyed and recreated on every screen transition (`_ui_screen_change()`)
- All widget callbacks and color wheel creation are re-done on each Screen2 visit via `settings_screen_init_values()`
- `settingsButton()` uses a deferred 200ms one-shot LVGL timer to wait for screen transition to complete
- SquareLine's event handler for the settings button never calls `settingsButton()` — our code registers it separately in `ui_events_post_init()`

## Data Flow

```
                CAN Interface Node
                       |
                UART (USB-C cable)
                       |
                       v
             ┌───────────────────┐
             │    Comm Link      │
             │   (UART RX)      │
             │                   │
             │  - Parse message  │
             │  - Validate CRC   │
             │  - Update PID     │
             │    data store     │
             └────────┬──────────┘
                      |
             ┌────────┴──────────┐
             │   PID Data Store  │
             │  (shared memory)  │
             └──┬─────────────┬──┘
                |             |
       ┌────────┴───┐   ┌────┴──────────┐
       │  Gauge     │   │  Data Logger   │
       │  Engine    │   │               │
       │            │   │  logger_log_  │
       │  Per-slot  │   │  row() on     │
       │  PID state,│   │  each new PID │
       │  unit conv,│   │               │
       │  value fmt │   │  Output:       │
       └──────┬─────┘   │  CSV on SD     │
              |         └───────────────┘
              |
       ┌──────┴──────┐
       │  UI Layer   │
       │  (LVGL)     │
       │             │
       │  Table-     │
       │  driven     │
       │  widget map │
       │             │
       │  Output:    │
       │  Display    │
       └─────────────┘
```

## Build Instructions

This project uses **PlatformIO** with the ESP-IDF framework. Two environments are available:

| Environment | Board | Display |
|-------------|-------|---------|
| `display_node` | CrowPanel 4.3" (N4R2) | 480x272 rectangle |
| `waveshare_2_1` | Waveshare Touch LCD 2.1 (R8) | 480x480 round |

### Build

```bash
pio run -e display_node        # CrowPanel 4.3"
pio run -e waveshare_2_1       # Waveshare 2.1"
```

### Flash and Monitor

```bash
pio run -e display_node -t upload && pio device monitor
pio run -e waveshare_2_1 -t upload -t monitor --upload-port COM11 --monitor-port COM11
```

### Full Clean Build

Required after changing `sdkconfig.defaults`:

```bash
pio run -e display_node -t fullclean && pio run -e display_node
pio run -e waveshare_2_1 -t fullclean && pio run -e waveshare_2_1
```
