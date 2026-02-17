# CAN_ESPIDF - Display Node

## Purpose

The Display Node is one of two ESP32-S3 nodes in the CAN_ESPIDF system. It receives real-time decoded PID data from the CAN Interface Node over UART via a USB-C cable, renders live gauges on an Elecrow CrowPanel (built-in RGB parallel LCD with touch), logs all data to SD card in HPTuners-compatible CSV format, and serves a WiFi access point for log file download, gauge configuration, and CAN Interface Node settings.

This node does NOT interact with the vehicle CAN bus directly. All vehicle data arrives from the CAN Interface Node over a wired UART link (USB-C cable). The Display Node is the only node in the system with WiFi, serving as the user-facing hub for configuration and log download.

The Display Node hardware is an Elecrow CrowPanel -- a self-contained ESP32-S3 module with a built-in RGB parallel LCD, capacitive touch screen, SD card slot, and audio output. Two panels are supported: DIS08070H (7" 800x480) and DIS07050 (5" 800x480). Power is supplied at 5V from the CAN Interface Node over USB-C VBUS; no separate power supply is needed.

### Responsibilities

- Receive and parse UART messages from the CAN Interface Node
- Maintain a local PID data store with the latest values and timestamps
- Render configurable gauge layouts on the CrowPanel RGB parallel LCD at 60fps
- Provide touch screen interaction for local configuration without WiFi
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
| devices | `components/devices/` | Per-device pin definitions and hardware config for each CrowPanel variant (DIS08070H, DIS07050) |
| display_driver | `components/display_driver/` | RGB parallel LCD driver (Elecrow CrowPanel), framebuffer management, touch input |
| gauge_engine | `components/gauge_engine/` | Gauge rendering (numeric, bar, sweep/dial), layout management, alert/warning system |
| data_logger | `components/data_logger/` | SD card CSV logging, session management, write buffering, file rotation |
| wifi_manager | `components/wifi_manager/` | WiFi AP mode, HTTP server for log browsing/download, gauge config web UI, CAN node config proxy |
| system | `components/system/` | Logging wrapper, NVS config storage, timing utilities, task management |

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
├── sdkconfig.defaults
├── partitions.csv
├── main/
│   ├── main.c
│   └── CMakeLists.txt
└── components/
    ├── comm_link/          # UART RX/TX, PID data store
    ├── devices/            # Per-panel pin definitions (DIS08070H, DIS07050)
    ├── display_driver/     # RGB parallel LCD driver
    ├── gauge_engine/       # Gauge rendering and alerts
    ├── data_logger/        # SD card CSV logging
    ├── wifi_manager/       # WiFi AP, HTTP server, CAN node config proxy
    └── system/             # Logging, NVS, timing, task helpers
```

## FreeRTOS Task Layout

All tasks are created during boot in `main.c`. Core affinity and priorities are chosen to keep display rendering isolated on Core 1 while all I/O and communication tasks share Core 0.

| Task | Core | Priority | Period | Stack Size | Purpose |
|------|------|----------|--------|------------|---------|
| Display Render | 1 | 5 | 16ms (~60fps) | 8192 bytes | Gauge rendering, framebuffer push to display |
| Comm Link | 0 | 4 | 5ms | 4096 bytes | UART RX processing, PID store update |
| Data Logger | 0 | 3 | 50ms | 4096 bytes | Flush write buffer to SD card, session management |
| WiFi/Config | 0 | 1 | 100ms | 4096 bytes | HTTP server for log download, gauge config, CAN node config proxy |
| System Monitor | 0 | 0 | 1000ms | 2048 bytes | Heap usage, task watchdog, uptime, connection status |

### Task Details

**Display Render Task (Core 1, Priority 5, 16ms)**
- Pinned to Core 1 to guarantee uninterrupted frame rendering
- Reads latest PID values from the shared PID data store (lock-free or mutex-protected)
- Calls gauge_engine to render the active layout into the framebuffer
- Pushes framebuffer to display via display_driver HAL (RGB parallel interface)
- Processes touch input for local configuration (layout switching, alert ack, etc.)
- Evaluates alert thresholds and triggers warning overlays
- Must complete within 16ms to maintain 60fps; drops to 30fps if a frame overruns

**Comm Link Task (Core 0, Priority 4, 5ms)**
- Highest priority on Core 0 to minimize latency on incoming PID data
- UART RX interrupt places raw bytes into a receive buffer
- Task reads complete frames from buffer, parses them via shared/comm_protocol definitions
- Updates the PID data store with decoded values and receive timestamps
- Monitors heartbeat messages; sets connection status flags on timeout
- Sends CONFIG_CMD messages over UART TX to CAN node when requested (including proxied config from web UI)

**Data Logger Task (Core 0, Priority 3, 50ms)**
- Accumulates PID snapshots in a RAM ring buffer
- Flushes buffer to SD card every 50ms (or when buffer hits threshold)
- Manages CSV file headers, session start/stop, and file rotation
- Uses double-buffering: one buffer fills while the other writes to SD
- Gracefully handles SD card removal and reinsertion

**WiFi/Config Task (Core 0, Priority 1, 100ms)**
- Starts WiFi AP on demand (button press, touch screen, or NVS config)
- Runs lightweight HTTP server for log file browsing and download
- Serves gauge configuration web UI (layout selection, PID assignment, alert thresholds)
- Proxies CAN Interface Node configuration (settings sent over UART to CAN node)
- Handles OTA firmware update endpoint (future)
- Low priority -- yields to comm and logging tasks

**System Monitor Task (Core 0, Priority 0, 1000ms)**
- Reports free heap, minimum free heap, and per-task stack high water marks
- Feeds the task watchdog timer
- Logs connection status (UART link health, time since last heartbeat)
- Monitors SD card usage and remaining space
- Outputs periodic health summary to debug UART

## Pin Assignments (Elecrow CrowPanel)

Pin assignments on the CrowPanel are highly constrained. The RGB parallel display, touch controller, SD card, and audio are all wired internally on the CrowPanel PCB. Available GPIOs for external connections are very limited.

Display pins (RGB parallel interface), backlight, and touch I2C pins are defined per-device in the `components/devices/` headers for each panel variant (DIS08070H and DIS07050).

### UART - Inter-Node Communication (USB-C Cable)

| Signal | GPIO | Notes |
|--------|------|-------|
| UART TX | TBD | To CAN Interface Node (config commands, proxied settings) |
| UART RX | TBD | From CAN Interface Node (PID data, heartbeat) |

### SD Card (SPI)

SD card pins are defined in the per-device headers. Both panels use the same SPI pins:

| Signal | GPIO | Notes |
|--------|------|-------|
| MOSI | GPIO 11 | SPI bus |
| MISO | GPIO 13 | SPI bus |
| SCLK | GPIO 12 | SPI bus |
| CS | GPIO 10 | Chip select |

### Touch (I2C)

| Signal | GPIO | Notes |
|--------|------|-------|
| SDA | GPIO 19 | I2C bus (capacitive touch controller) |
| SCL | GPIO 20 | I2C bus (capacitive touch controller) |

### Debug / Misc

| Signal | GPIO | Notes |
|--------|------|-------|
| UART TX | GPIO43 | USB-UART bridge (debug console, default S3) |
| UART RX | GPIO44 | USB-UART bridge (debug console, default S3) |

### Power

The Display Node receives 5V power from the CAN Interface Node via USB-C VBUS. No separate power supply is needed.

## Boot Sequence

Initialization proceeds in a strict order. Each stage must succeed before the next begins. Non-critical failures (SD card missing, display not detected) are logged but do not halt boot.

```
1. system_init()
   - Initialize ESP-IDF logging
   - Initialize NVS flash (user config storage)
   - Load saved configuration (gauge layout, alert thresholds, WiFi settings)
   - Start system timer

2. display_driver_init()
   - Select device config from devices/ based on build target or NVS
   - Configure RGB parallel interface for CrowPanel LCD
   - Initialize touch controller (I2C)
   - Clear screen, show boot splash / CAN_ESPIDF logo
   - [NON-CRITICAL: if display fails, system continues headless]

3. comm_link_init()
   - Initialize UART peripheral for inter-node communication
   - Configure UART RX/TX pins, baud rate, framing
   - Allocate PID data store
   - Begin listening for heartbeat from CAN Interface Node

4. data_logger_init()
   - Initialize SPI bus for SD card
   - Mount FAT filesystem
   - Scan existing log files, determine next session number
   - Allocate write buffers
   - [NON-CRITICAL: if SD card missing, logging disabled until inserted]

5. gauge_engine_init()
   - Load gauge layout from NVS config
   - Initialize gauge state (smoothing filters, alert states)
   - Link to PID data store from comm_link

6. wifi_manager_init()
   - [DEFERRED: WiFi AP started only on demand, not at boot]
   - Register HTTP server handlers (log browser, gauge config, CAN node config proxy)
   - Prepare file listing for log directory

7. Task creation
   - Create all FreeRTOS tasks with assigned cores and priorities
   - Display Render Task starts immediately
   - Comm Link Task starts immediately (listening for UART RX)
   - Data Logger Task starts in standby (waits for session start trigger)
   - WiFi/Config Task starts in standby (waits for AP enable trigger)
   - System Monitor Task starts immediately
```

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
                 │                   │
                 │  - Latest values  │
                 │  - Timestamps     │
                 │  - Stale flags    │
                 └──┬─────────────┬──┘
                    |             |
           ┌────────┴───┐   ┌────┴──────────┐
           │  Gauge     │   │  Data Logger   │
           │  Engine    │   │               │
           │            │   │  - Snapshot    │
           │  - Render  │   │    PID store   │
           │    gauges  │   │  - Buffer row  │
           │  - Check   │   │  - Flush to   │
           │    alerts  │   │    SD card     │
           │  - Touch   │   │               │
           │    input   │   │  Output:       │
           │  - Push to │   │  CSV file on   │
           │    display │   │  SD card       │
           └────────────┘   └───────────────┘
                                   |
                                   v
                         ┌─────────────────┐
                         │  WiFi Manager   │
                         │                 │
                         │  - Browse logs  │
                         │  - Download CSV │
                         │  - Configure    │
                         │    gauges       │
                         │  - Proxy CAN    │
                         │    node config  │
                         │    (over UART)  │
                         └─────────────────┘
```

### PID Data Store Design

The PID data store is a fixed-size array indexed by PID ID. Each entry contains:

```c
typedef struct {
    float value;            // Decoded value in display units
    int64_t timestamp_us;   // esp_timer_get_time() when received
    uint8_t stale;          // Set if no update within timeout (UART heartbeat lost)
    uint8_t valid;          // Set after first successful receive
} pid_data_entry_t;
```

Concurrency: The Comm Link Task writes entries and the Display Render / Data Logger tasks read them. Access is protected by either:
- A per-entry spinlock (minimal contention, no priority inversion), or
- Atomic copy of the 12-byte struct (if platform supports it), or
- A single mutex with short hold time

The design will be finalized during implementation. The key constraint is that the Display Render Task on Core 1 must never block for more than ~100us waiting on a lock held by Core 0.

## HPTuners CSV Format

Log files are written in a format compatible with HPTuners and VCM Scanner for import and analysis. The CSV structure is:

```
Timestamp,Engine RPM,Coolant Temp (F),Oil Pressure (psi),Boost (psi),AFR,...
0.000,850.0,185.2,42.5,0.0,14.7,...
0.050,855.0,185.3,42.4,0.0,14.7,...
0.100,860.0,185.4,42.3,0.1,14.6,...
```

- First row: column headers with PID name and unit
- Timestamp column: seconds since session start, 3 decimal places
- One row per logging interval (50ms = 20 rows/second)
- Only actively-polled PIDs are included as columns
- File naming: `LOG_YYYYMMDD_HHMMSS_NNN.csv` (NNN = session counter)

## Alert / Warning System

The gauge engine evaluates alert thresholds on every render cycle. Alert definitions are stored in NVS and configurable via the WiFi web UI or touch screen.

| Alert Level | Behavior |
|-------------|----------|
| NORMAL | Standard gauge rendering |
| WARNING | Gauge value highlighted (e.g., yellow), optional border flash |
| CRITICAL | Gauge value in red, full-screen flash overlay, optional buzzer |

Default alert thresholds (user-configurable):

| Parameter | Warning | Critical |
|-----------|---------|----------|
| Coolant Temp | > 220F | > 240F |
| Oil Pressure | < 25 psi | < 15 psi |
| Trans Temp | > 200F | > 220F |
| Boost Pressure | > 15 psi | > 20 psi |
| Battery Voltage | < 13.0V | < 12.0V |

## Build Instructions

The Display Node is an independent ESP-IDF project. It references shared components from `../shared/`.

### Prerequisites

- ESP-IDF v5.x installed and configured
- ESP32-S3 target support
- Python 3.8+ (for ESP-IDF tools)

### Build

```bash
cd CAN_Display
idf.py set-target esp32s3
idf.py build
```

### Flash and Monitor

```bash
idf.py -p COMX flash monitor
```

Replace `COMX` with the actual serial port for the Display Node (Elecrow CrowPanel USB port).

### Configuration

```bash
idf.py menuconfig
```

Project-specific options will be under `CAN_ESPIDF Display Node` in the menuconfig tree (once sdkconfig.defaults and Kconfig files are created).

### Clean Build

```bash
idf.py fullclean
idf.py build
```
