# CAN_ESPIDF - CAN Bus Gauge & Data Logging System

## Overview

CAN_ESPIDF is a two-node automotive CAN bus gauge and data logging system built on ESP-IDF. It replaces the previous Arduino-based project with a clean, modular, production-quality ESP-IDF codebase.

**What it does:** Reads OBD-II and manufacturer-specific data from a vehicle's CAN bus, displays it in real-time gauges, and logs it to SD card in formats compatible with data analysis tools (HPTuners-style CSV).

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     VEHICLE CAN BUS                         │
│               (500kbps CAN 2.0B / CAN-FD)                  │
└──────────┬──────────────────────┬───────────────────────────┘
           │                      │
    ┌──────┴───────┐       ┌──────┴───────┐
    │  Waveshare   │       │  TWAI        │
    │  2-CH CAN FD │       │  (native)    │  Reserved: 1-Wire GM
    │  HAT (SPI)   │       │  via CAN xcvr│
    │  2x MCP2518FD│       └──────┬───────┘
    └──────┬───────┘              │
           │                      │
    ┌──────┴──────────────────────┴───────┐
    │         CAN INTERFACE NODE          │
    │     (ESP32-S3-N16R8-DevKitC)        │
    │                                     │
    │  ┌─────────────────────────────┐    │
    │  │ CAN Driver (HAL)            │    │
    │  │  - MCP2518FD backend (SPI)  │    │
    │  │  - TWAI backend (1-Wire GM) │    │
    │  └────────────┬────────────────┘    │
    │  ┌────────────┴────────────────┐    │
    │  │ OBD-II Protocol Stack       │    │
    │  │  - Request/Response         │    │
    │  │  - ISO-TP multi-frame       │    │
    │  │  - Flow control             │    │
    │  └────────────┬────────────────┘    │
    │  ┌────────────┴────────────────┐    │
    │  │ PID Database (const table)  │    │
    │  │  - Standard OBD-II          │    │
    │  │  - GM Extended (Mode 0x22)  │    │
    │  │  - Formula-based decode     │    │
    │  └────────────┬────────────────┘    │
    │  ┌────────────┴────────────────┐    │
    │  │ Poll Engine                 │    │
    │  │  - Priority scheduling      │    │
    │  │  - Adaptive intervals       │    │
    │  └────────────┬────────────────┘    │
    │  ┌────────────┴────────────────┐    │
    │  │ Diagnostics                 │    │
    │  │  - DTC read/clear           │    │
    │  │  - UDS (Service 0x19)       │    │
    │  │  - VIN/CVN/ECU info         │    │
    │  └─────────────────────────────┘    │
    └──────────────┬──────────────────────┘
                   │
          UART over USB-C Cable
        (2Mbps full-duplex, DMA)
        USB-C carries power + data
        (VBUS/GND + D+/D- as TX/RX)
                   │
    ┌──────────────┴──────────────────────┐
    │         DISPLAY NODE                │
    │    (Elecrow CrowPanel ESP32-S3)     │
    │                                     │
    │  ┌─────────────────────────────┐    │
    │  │ Comm Link (UART RX/TX)     │    │
    │  └────────────┬────────────────┘    │
    │  ┌────────────┴────────────────┐    │
    │  │ Gauge Engine                │    │
    │  │  - Layout management        │    │
    │  │  - Gauge rendering          │    │
    │  │  - Alert/warning system     │    │
    │  └────────────┬────────────────┘    │
    │  ┌────────────┴────────────────┐    │
    │  │ Display Driver (HAL)        │    │
    │  │  - RGB parallel LCD         │    │
    │  │  - Touch input              │    │
    │  └─────────────────────────────┘    │
    │  ┌─────────────────────────────┐    │
    │  │ Data Logger                 │    │
    │  │  - SD card CSV logging      │    │
    │  │  - HPTuners-compatible fmt  │    │
    │  │  - Session management       │    │
    │  └─────────────────────────────┘    │
    │  ┌─────────────────────────────┐    │
    │  │ WiFi AP                     │    │
    │  │  - Config interface         │    │
    │  │  - Log file download        │    │
    │  │  - Gauge layout config      │    │
    │  └─────────────────────────────┘    │
    │  ┌─────────────────────────────┐    │
    │  │ Audio (I2S)                 │    │
    │  │  - Alert tones              │    │
    │  └─────────────────────────────┘    │
    └─────────────────────────────────────┘
```

## Project Structure

```
CAN_ESPIDF/
├── README.md                       # This file
├── TODO.md                         # Project-wide roadmap and tasks
│
├── CAN_Interface/                  # CAN Interface Node firmware
│   ├── README.md                   # Node-specific docs
│   ├── TODO.md                     # Node-specific tasks
│   ├── CMakeLists.txt              # Top-level CMake for this node
│   ├── sdkconfig.defaults          # ESP-IDF config defaults
│   ├── partitions.csv              # Flash partition table
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   └── main.c                  # Entry point, task creation
│   └── components/
│       ├── can_driver/             # CAN hardware abstraction
│       │   └── devices/            # MCP2518FD, TWAI backends
│       ├── pid_db/                 # PID decode table + unit conversions
│       ├── obd2/                   # OBD-II protocol (req/resp, ISO-TP)
│       ├── diagnostics/            # DTC/UDS/vehicle info
│       ├── poll_engine/            # Smart polling scheduler
│       ├── comm_link/              # UART TX to display node
│       └── system/                 # Logging, NVS config, timing
│
├── CAN_Display/                    # Display Node firmware
│   ├── README.md                   # Node-specific docs
│   ├── TODO.md                     # Node-specific tasks
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   ├── partitions.csv
│   ├── main/
│   │   ├── CMakeLists.txt
│   │   └── main.c
│   └── components/
│       ├── comm_link/              # UART RX/TX from CAN node
│       ├── display_driver/         # Display HAL (RGB parallel LCD)
│       │   └── devices/            # Elecrow CrowPanel panel drivers
│       ├── gauge_engine/           # Gauge rendering + layouts
│       ├── data_logger/            # SD card CSV logging
│       ├── wifi_ap/                # WiFi AP for config + log download
│       └── system/                 # Logging, NVS config, timing
│
└── shared/                         # Shared definitions (both nodes)
    ├── README.md
    ├── pid_types/                  # PID data types, enums, units
    ├── comm_protocol/              # UART message format definitions
    └── utils/                      # Common utilities
```

## Hardware

### CAN Interface Node
| Component | Purpose | Interface |
|-----------|---------|-----------|
| ESP32-S3-N16R8-DevKitC | Main MCU (16MB flash, 8MB PSRAM) | - |
| Waveshare 2-CH CAN FD HAT | Primary CAN interface (2x MCP2518FD) | SPI + INT GPIOs |
| - | CAN 2.0B and CAN-FD on both channels | - |
| - | 8-28V input (vehicle 12V), galvanic isolation, surge protection | - |
| CAN Transceiver (SN65HVD230 / TJA1050) | TWAI native CAN (reserved for 1-Wire GM) | GPIO TX/RX |
| USB-C cable to Display Node | UART data + 5V power to Display Node | UART TX/RX on D+/D- |

### Display Node (Elecrow CrowPanel)
| Component | Purpose | Notes |
|-----------|---------|-------|
| Elecrow CrowPanel 5" (DIS07050) | Self-contained ESP32-S3 + display | 800x480 RGB parallel LCD, capacitive touch |
| Elecrow CrowPanel 7" (DIS08070H) | Alternate larger panel | 800x480 RGB parallel LCD, capacitive touch |
| Built-in SD card slot | Data logging | SDMMC interface |
| Built-in I2S audio | Alert tones | On-board speaker/amp |
| Built-in touch screen | User interaction | Capacitive touch controller |

### Inter-Node Communication
| Method | Purpose | Notes |
|--------|---------|-------|
| UART over USB-C | Real-time PID data streaming + commands | 2Mbps full-duplex, DMA-supported |
| USB-C power | CAN Interface powers Display Node | VBUS 5V from CAN Interface to CrowPanel |
| WiFi AP (Display Node only) | Configuration & log download | Served by Display Node only |

## Key Design Decisions

### Why Two Nodes?
1. **Separation of concerns** - CAN bus timing is critical; display rendering shouldn't compete for CPU cycles
2. **Physical placement** - CAN interface near OBD port, display wherever convenient
3. **Independent development** - CAN stack and display can be developed/tested independently
4. **Flexibility** - Swap displays without touching CAN firmware

### Why Wired UART over USB-C?
1. **Reliability** - Hard-wired connection eliminates wireless dropouts, pairing issues, and RF interference in vehicle environments
2. **Speed** - 2Mbps full-duplex with DMA; more than enough bandwidth for PID streaming
3. **Power delivery** - USB-C cable carries both data (D+/D- repurposed as UART TX/RX) and power (VBUS 5V to Display Node), single cable for everything
4. **Simplicity** - No wireless stack, no MAC pairing, no channel management; just a UART peripheral with DMA
5. **Deterministic latency** - No contention, no retransmits, no variable RF conditions

### Why Waveshare 2-CH CAN FD HAT?
1. **Vehicle-grade power** - 8-28V DC input handles vehicle 12V directly with built-in regulation
2. **Galvanic isolation** - Protects the ESP32-S3 from vehicle bus transients and ground loops
3. **Surge protection** - Built-in ESD and surge protection for automotive environments
4. **Dual CAN-FD channels** - Two independent MCP2518FD controllers for multi-bus vehicles (e.g., HS-CAN + MS-CAN)
5. **CAN 2.0B + CAN-FD** - Both channels support classic CAN and CAN-FD, no hardware swap needed
6. **Frees TWAI** - Native ESP32-S3 TWAI peripheral remains available for 1-Wire GM legacy network

### Why Pure C (not C++)?
1. **ESP-IDF native** - ESP-IDF APIs are C; no impedance mismatch
2. **Deterministic memory** - No hidden allocations from std::function, std::vector, String
3. **ROM-resident data** - const tables live in flash, not heap
4. **Smaller binary** - No C++ runtime overhead (exceptions, RTTI, vtables)

### PID Database Redesign
The Arduino version used `std::function<>` lambdas for each PID decoder (~165 entries). This consumed significant heap memory. The ESP-IDF version uses a **formula-type enum** with parameters:

```c
// OLD (Arduino) - each entry allocates heap for a std::function
{ 0x0C, "Engine RPM", PIDType::FORMULA, Unit::RPM,
  [](const uint8_t* d) { return DecodedValue{((d[0] << 8) | d[1]) / 4.0f, "rpm"}; }
}

// NEW (ESP-IDF) - pure const data, lives in flash
{ 0x0C, "Engine RPM", PID_FORMULA_AB_DIV_N, UNIT_RPM, {.divisor = 4.0f} }
```

## Communication Protocol (UART)

Messages between nodes use a compact binary format over the UART link, defined in `shared/comm_protocol/`:

| Message Type | Direction | Purpose |
|-------------|-----------|---------|
| PID_DATA | CAN -> Display | Real-time decoded PID values |
| PID_BATCH | CAN -> Display | Multiple PID values in one frame |
| VEHICLE_INFO | CAN -> Display | VIN, ECU name, supported PIDs |
| DTC_DATA | CAN -> Display | Diagnostic trouble codes |
| HEARTBEAT | Both | Connection health monitoring |
| CONFIG_CMD | Display -> CAN | Change poll list, request scan |
| CONFIG_RESP | CAN -> Display | Configuration acknowledgment |
| LOG_CONTROL | Display -> CAN | Start/stop logging session |

The UART link runs at 2Mbps with DMA on both ends. Messages are framed with a sync byte, length, type, payload, and CRC to provide reliable delivery over the wired connection.

## Build & Flash

Each node is a separate ESP-IDF project. Build and flash independently:

```bash
# CAN Interface Node
cd CAN_Interface
idf.py set-target esp32s3
idf.py build
idf.py -p COMX flash monitor

# Display Node (Elecrow CrowPanel)
cd CAN_Display
idf.py set-target esp32s3
idf.py build
idf.py -p COMY flash monitor
```

## Migrated From

This project migrates the verified PID database and protocol logic from the original Arduino-based project. Key assets preserved:
- **165 PID decode entries** (standard OBD-II 0x00-0xA4 + GM extended 0x83F-0x4268)
- **150 poll configurations** with tuned intervals and 5-tier priorities
- **OBD-II protocol logic** (request/response, ISO-TP, flow control)
- **DTC/UDS diagnostic handling** (Mode 03/07/0x19)
- **Unit conversion system** (temperature, pressure, speed, torque, etc.)
