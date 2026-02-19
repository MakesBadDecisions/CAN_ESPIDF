# CAN_ESPIDF - Display Node

## Purpose

The Display Node is one of two ESP32-S3 nodes in the CAN_ESPIDF system. It receives real-time decoded PID data from the CAN Interface Node over UART via a USB-C cable, renders live gauges on an Elecrow CrowPanel (built-in RGB parallel LCD with touch), logs all data to SD card in HPTuners-compatible CSV format, and serves a WiFi access point for log file download, gauge configuration, and CAN Interface Node settings.

This node does NOT interact with the vehicle CAN bus directly. All vehicle data arrives from the CAN Interface Node over a wired UART link (USB-C cable). The Display Node is the only node in the system with WiFi, serving as the user-facing hub for configuration and log download.

The primary target hardware is the **Elecrow CrowPanel 4.3" (DIS06043H)** — an ESP32-S3-WROOM-1-N4R2 with 480x272 RGB parallel LCD, XPT2046 resistive touch (SPI), SD card slot, and I2S audio. Two larger panels are also supported: DIS07050 (5" 800x480) and DIS08070H (7" 800x480), both with capacitive GT911 touch (I2C).

### Responsibilities

- Receive and parse UART messages from the CAN Interface Node
- Maintain a local PID data store with the latest values and timestamps
- Render configurable gauge layouts on the CrowPanel RGB parallel LCD
- Provide touch screen interaction for local configuration
- Monitor critical values and trigger visual/audible alerts (overtemp, low oil pressure, etc.)
- Log all received PID data to SD card as HPTuners-compatible CSV
- Manage logging sessions (start/stop, file naming, file rotation)
- Host a WiFi AP for browsing/downloading log files and configuring gauge layouts
- Proxy CAN Interface Node settings over the UART link (user configures CAN node through display's web UI)
- Send configuration commands back to the CAN Interface Node over UART TX (poll list changes, scan requests)

## Component Overview

| Component | Directory | Purpose |
|-----------|-----------|---------|
| comm_link | `components/comm_link/` | UART receive handler, message parsing, connection health monitoring, PID data store |
| devices | `components/devices/` | Per-device pin definitions and hardware config for each CrowPanel variant |
| display_driver | `components/display_driver/` | RGB parallel LCD driver, LVGL integration, double-FB anti-tearing |
| touch_driver | `components/touch_driver/` | XPT2046 resistive touch (SPI), Core 0 polling task, 4-corner calibration, NVS persistence |
| ui | `components/ui/` | SquareLine Studio generated LVGL screens and widgets |
| gauge_engine | `components/gauge_engine/` | Gauge rendering (numeric, bar, sweep/dial), layout management, alert/warning system |
| data_logger | `components/data_logger/` | SD card CSV logging, session management, write buffering, file rotation |
| wifi_manager | `components/wifi_manager/` | WiFi AP mode, HTTP server for log browsing/download, gauge config web UI, CAN node config proxy |
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
├── partitions.csv
├── main/
│   ├── main.c
│   └── CMakeLists.txt
└── components/
    ├── comm_link/          # UART RX/TX, PID data store
    ├── devices/            # Per-panel pin definitions (DIS06043H, DIS07050, DIS08070H)
    ├── display_driver/     # RGB parallel LCD + LVGL (double FB, bounce buffers, VSYNC)
    ├── touch_driver/       # XPT2046 SPI touch (Core 0 task, calibration, NVS)
    ├── ui/                 # SquareLine Studio generated LVGL UI
    ├── gauge_engine/       # Gauge rendering and alerts
    ├── data_logger/        # SD card CSV logging
    ├── wifi_manager/       # WiFi AP, HTTP server, CAN node config proxy
    ├── system/             # Logging, NVS, timing
    └── lvgl/               # LVGL v8.3 library
```

## FreeRTOS Task Layout

Tasks currently running in the firmware:

| Task | Core | Priority | Stack | Purpose |
|------|------|----------|-------|---------|
| `lvgl` | 1 | 3 | 16 KB | LVGL timer handler (rendering + input processing) |
| `touch` | 0 | 2 | 4 KB | XPT2046 SPI polling, writes to volatile cache |
| `comm_rx` | 0 | 4 | 4 KB | UART RX from CAN Interface Node |

Core 1 is dedicated to display rendering (LVGL + LCD bounce buffer DMA ISR).
Touch SPI reads are isolated on Core 0 to prevent stalling the LCD DMA ISR.

### Future Tasks (not yet implemented)

| Task | Core | Priority | Purpose |
|------|------|----------|---------|
| Data Logger | 0 | 3 | SD card CSV writes |
| WiFi/Config | 0 | 1 | HTTP server for log download and config |
| System Monitor | 0 | 0 | Heap usage, watchdog, health reporting |

## Pin Assignments (CrowPanel 4.3" DIS06043H)

### RGB LCD (16-bit parallel)
All 16 data lines + HSYNC/VSYNC/DE/PCLK — see `dis06043h.h` for full list.

### Touch (XPT2046 SPI)
| Signal | GPIO | Notes |
|--------|------|-------|
| SCK | 12 | Shared SPI bus with SD card |
| MOSI | 11 | Shared SPI bus with SD card |
| MISO | 13 | Shared SPI bus with SD card |
| CS | 0 | Touch chip select |
| INT | 36 | Touch interrupt (active low) |

### UART - Inter-Node Communication
| Signal | GPIO | Notes |
|--------|------|-------|
| UART1 TX | 17 | To CAN Interface Node RX (GPIO18) |
| UART1 RX | 18 | From CAN Interface Node TX (GPIO17) |

On the 4.3" panel, UART1 is on a dedicated HY2.0-4P connector, matching the
CAN Interface Node pins for direct wiring.

### SD Card (SPI)
| Signal | GPIO | Notes |
|--------|------|-------|
| MOSI | 11 | Shared SPI bus with touch |
| MISO | 13 | Shared SPI bus with touch |
| SCLK | 12 | Shared SPI bus with touch |
| CS | 10 | SD card chip select |

### Backlight / Panel Enable
| Signal | GPIO | Notes |
|--------|------|-------|
| Backlight | 2 | Active high |
| Panel Enable | 38 | Must be HIGH for display to work |

## Boot Sequence

The actual boot sequence as implemented in `main.c`:

```
1. system_init()
   - Initialize NVS flash
   - Print device info (chip, flash, PSRAM, MAC)

2. display_init()
   - Configure backlight and panel enable GPIOs
   - Initialize RGB parallel LCD panel (double FB, bounce buffers, VSYNC)
   - Initialize LVGL (direct_mode, register display driver)
   - Create LVGL task on Core 1
   - 100ms delay for LVGL task startup

3. touch_init()
   - Initialize SPI bus for XPT2046 (DMA disabled)
   - Load calibration from NVS (or use header defaults)
   - Start touch polling task on Core 0
   - Register LVGL input device

4. Touch calibration check
   - If no NVS calibration data: show 4-corner calibration screen
   - User touches crosshairs at each corner
   - Saves calibration to NVS for future boots

5. ui_init()
   - Load SquareLine Studio generated UI screens
   - (Requires display lock)

6. comm_link_init() + comm_link_start()
   - Initialize UART peripheral
   - Start RX task for incoming PID data
```

Non-critical failures (touch, SD card) are logged but do not halt boot.

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
       │  Engine    │   │  (TODO)        │
       │  + LVGL UI │   │               │
       │            │   │  Output:       │
       │  Output:   │   │  CSV on SD     │
       │  Display   │   │               │
       └────────────┘   └───────────────┘
```

## Build Instructions

This project uses **PlatformIO** with the ESP-IDF framework.

### Build

```bash
pio run -e display_node
```

### Flash and Monitor

```bash
pio run -e display_node -t upload && pio device monitor
```

### Full Clean Build

Required after changing `sdkconfig.defaults`:

```bash
pio run -e display_node -t fullclean && pio run -e display_node
```

### Configuration

```bash
pio run -e display_node -t menuconfig
```
