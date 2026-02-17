# CAN_ESPIDF -- CAN Interface Node

## Purpose

The CAN Interface Node is one of two nodes in the CAN_ESPIDF system. It runs on an ESP32-S3-N16R8-DevKitC and serves as the vehicle-side bridge: it reads raw CAN frames from the OBD-II port, decodes them into engineering values, and transmits the results to the Display Node over UART (USB-C cable).

Phase 1 uses an **MCP2515** SPI CAN controller (proven from the Arduino project). The **Waveshare 2-CH CAN FD HAT** (2x MCP2518FD) will be added as a drop-in backend in Phase 7 behind the same CAN driver HAL.

### Responsibilities

- **CAN bus communication** -- Talk to the vehicle via MCP2515 (Phase 1, SPI) or Waveshare 2-CH CAN FD HAT (Phase 7, 2x MCP2518FD over SPI2). TWAI reserved for 1-Wire GM legacy networks.
- **OBD-II protocol** -- Build Mode 01/02 requests, parse single-frame and ISO-TP multi-frame responses, handle flow control.
- **PID decode** -- Look up 165 PIDs (standard OBD-II 0x00--0xA4 plus GM extended 0x83F--0x4268) in a const table and apply formula-based decoding to produce engineering values with units.
- **Smart polling** -- Schedule PID requests using priority tiers and adaptive intervals so high-value PIDs update fast while low-priority PIDs share remaining bandwidth.
- **Diagnostics** -- Read and clear DTCs (Mode 03/07), UDS service 0x19 extended diagnostics, VIN/CVN/ECU info retrieval.
- **UART TX** -- Batch decoded PID values and send them to the Display Node over UART1 (2Mbps, USB-C cable).
- **System health** -- Watchdog feeding, bus-off recovery, heap monitoring, uptime and error statistics.

> **Note:** This node is a headless backend. There is no WiFi AP or HTTP server on the CAN Interface Node. All user interaction (WiFi AP, display, configuration UI) is handled by the Display Node.

### Future

- Waveshare 2-CH CAN FD HAT (2x MCP2518FD) for CAN-FD support (Phase 7).
- 1-Wire GM legacy network interface for older vehicles (TWAI backend).

---

## Component Overview

All components live under `components/`. Each is a self-contained ESP-IDF component with its own `CMakeLists.txt` and public headers.

| Component      | Directory        | Description                                                                                   |
| -------------- | ---------------- | --------------------------------------------------------------------------------------------- |
| Devices        | `devices/`       | Board-specific hardware pin definitions and capability flags. See `ESP32-S3-N16R8-DevKitC.h`. |
| CAN Driver     | `can_driver/`    | Hardware abstraction layer. MCP2515 backend (Phase 1, SPI), MCP2518FD backend (Phase 7, 2 channels via SPI2), TWAI backend (1-Wire GM legacy). Unified API for open/close/send/receive regardless of backend. |
| PID Database   | `pid_db/`        | Pure C `const` table of 165 PID entries. Each entry holds the PID number, name, formula coefficients, unit string, min/max, and byte length. Provides a lookup function and unit conversion helpers. |
| OBD-II Stack   | `obd2/`          | OBD-II protocol implementation. Request builder (Mode + PID -> CAN frame), response parser, ISO-TP segmentation and reassembly, flow control frame generation. |
| Diagnostics    | `diagnostics/`   | DTC manager (Mode 03 read, Mode 04 clear, Mode 07 pending). UDS processor (service 0x19 extended DTC, 0x22 DID read). Vehicle info: VIN (Mode 09 PID 02), calibration verification numbers, ECU identification. |
| Poll Engine    | `poll_engine/`   | Priority-based scheduler. Maintains a sorted run queue of PID poll jobs with configurable priority (0--7), base interval, and adaptive backoff. Feeds requests to the OBD-II stack and tracks response latency. |
| Comm Link      | `comm_link/`     | UART transmit layer. Batches decoded PID values into UART frames, handles framing, sequence numbers, and CRC. Sends to Display Node over UART1 at 2Mbps via USB-C cable. |
| WiFi AP        | `wifi_ap/`       | **Removed** -- WiFi AP and HTTP config server have moved to the Display Node. The CAN Interface Node is a headless backend. |
| System         | `system/`        | Logging wrapper (tagged, leveled), NVS configuration read/write, timing utilities (microsecond timestamps, interval helpers), and task management (stack high-water-mark checks, task list). |

---

## FreeRTOS Task Layout

All tasks are pinned to a specific core. Core 1 handles time-critical CAN work. Core 0 handles communication and housekeeping.

| Task                | Core | Priority | Period   | Stack  | Description                                                                                  |
| ------------------- | ---- | -------- | -------- | ------ | -------------------------------------------------------------------------------------------- |
| CAN Processing      | 1    | 5        | 5 ms     | 4096 B | Receive CAN frames from the driver queue, pass to OBD-II parser, decode via PID DB, update the shared PID value store. Handles ISO-TP reassembly and flow control responses. |
| Poll Engine         | 1    | 4        | 10 ms    | 3072 B | Walk the priority-sorted run queue, check which PIDs are due, build OBD-II request frames, submit to the CAN driver TX queue. Update adaptive intervals based on response success/failure. |
| Comm Link           | 0    | 3        | 20 ms    | 3072 B | Snapshot the PID value store, batch changed values into UART frames, transmit to Display Node over UART1. Handle TX completion and detect link up/down. |
| System Monitor      | 0    | 0        | 1000 ms  | 2048 B | Feed the task watchdog. Log heap free, stack high-water marks, CAN error counters, bus state, UART TX stats. Trigger bus-off recovery if needed. |

### Task Communication

- **CAN Driver -> CAN Processing**: FreeRTOS queue (`can_rx_queue`, depth 32).
- **Poll Engine -> CAN Driver**: FreeRTOS queue (`can_tx_queue`, depth 16).
- **CAN Processing -> Comm Link**: Shared PID value store protected by a mutex. Comm Link reads a snapshot each cycle.
- **Comm Link -> Display Node**: UART1 TX queue (DMA-backed, 2Mbps).

---

## Pin Assignment (ESP32-S3-N16R8-DevKitC)

Defined in `components/devices/ESP32-S3-N16R8-DevKitC.h`. GPIO 35/36/37 reserved for Octal PSRAM.

| Function                  | GPIO    | Notes                                              |
| ------------------------- | ------- | -------------------------------------------------- |
| SPI2 MOSI                 | GPIO 11 | FSPI bus, shared by CAN SPI devices                |
| SPI2 SCK                  | GPIO 12 | FSPI bus                                           |
| SPI2 MISO                 | GPIO 13 | FSPI bus                                           |
| MCP2515 CS (Phase 1)      | GPIO 10 | MCP2515 chip select (primary OBD-II bus)           |
| MCP2515 INT (Phase 1)     | GPIO 9  | Active-low interrupt from MCP2515                  |
| CAN FD CH0 CS (Phase 7)   | GPIO 10 | MCP2518FD Channel 0 (replaces MCP2515 on same pins)|
| CAN FD CH0 INT (Phase 7)  | GPIO 9  | Active-low interrupt from MCP2518FD CH0            |
| CAN FD CH1 CS (Phase 7)   | GPIO 46 | MCP2518FD Channel 1 (secondary bus / GM extended)  |
| CAN FD CH1 INT (Phase 7)  | GPIO 3  | Active-low interrupt from MCP2518FD CH1            |
| TWAI TX                   | GPIO 4  | Reserved for 1-Wire GM legacy network              |
| TWAI RX                   | GPIO 5  | Reserved for 1-Wire GM legacy network              |
| UART1 TX (inter-node)     | GPIO 17 | To Display Node via USB-C cable, 2Mbps             |
| UART1 RX (inter-node)     | GPIO 18 | From Display Node via USB-C cable                  |
| UART0 TX (debug)          | GPIO 43 | Default USB-UART bridge, used for logging          |
| UART0 RX (debug)          | GPIO 44 | Default USB-UART bridge                            |
| Status LED                | GPIO 48 | External status indicator (active high)            |
| RGB LED                   | GPIO 38 | On-board WS2812 addressable LED                   |

---

## Boot Sequence

Initialization runs in `app_main()` on Core 0 before any application tasks are created. Order matters: later stages depend on earlier ones.

```
1. system_init()
   - Initialize logging (set default tag levels)
   - Initialize NVS flash (nvs_flash_init)
   - Load saved configuration from NVS into RAM config struct

2. can_driver_init()
   - Initialize SPI2 bus (MOSI=11, MISO=13, SCK=12)
   - Phase 1: Configure MCP2515 (CS=10, INT=9) as primary
   - Phase 7: Configure MCP2518FD CH0 (CS=10, INT=9) + CH1 (CS=46, INT=3)
   - (Optional) Initialize TWAI for 1-Wire GM legacy (TX=4, RX=5)

3. pid_db_init()
   - No dynamic init needed (const table), but validate table integrity
   - Build any runtime index structures (hash map for fast PID lookup)

4. obd2_init()
   - Allocate ISO-TP reassembly buffers
   - Register with can_driver for RX callback / queue handle

5. poll_engine_init()
   - Load PID poll list and priorities from NVS config
   - Build the sorted run queue
   - Reset all interval timers

6. diagnostics_init()
   - Allocate DTC storage
   - Register UDS service handlers

7. comm_link_init()
   - Initialize UART1 (TX=17, RX=18, 2Mbps, DMA)
   - Configure UART framing (start byte, length, CRC)
   - Begin link detection (wait for Display Node)

8. Start application tasks
   - xTaskCreatePinnedToCore() for each task in the table above
   - Tasks begin in a blocked state waiting on their first trigger

9. app_main() returns (idle task runs on Core 0)
```

---

## Data Flow

```
  Vehicle OBD-II Port
        |
        | CAN Bus (500 kbps typical, up to 8Mbps CAN-FD)
        v
+------------------+
|   CAN Driver     |  Phase 1: MCP2515 via SPI2 | Phase 7: Waveshare HAT
|   (can_driver/)  |  Primary OBD-II bus (single channel initially)
+--------+---------+
         |
         | can_rx_queue (FreeRTOS queue, raw CAN frames)
         v
+------------------+
| CAN Processing   |  Task on Core 1
|   (obd2/)        |  - Match response to pending request
|   (pid_db/)      |  - ISO-TP reassembly if multi-frame
+--------+---------+  - Formula decode: raw bytes -> eng value
         |
         | Write to shared PID value store (mutex)
         v
+------------------+
|   Comm Link      |  Task on Core 0
|   (comm_link/)   |  - Snapshot changed PIDs
+--------+---------+  - Pack into UART frames (shared/comm_protocol format)
         |
         | UART1 (2Mbps, DMA, over USB-C cable)
         v
  Display Node (separate ESP32-S3)
  - Handles WiFi AP, HTTP config, display UI


  Outbound request path (simultaneous):

+------------------+
|   Poll Engine    |  Task on Core 1
|   (poll_engine/) |  - Check run queue for due PIDs
+--------+---------+  - Build OBD-II request frame
         |
         | can_tx_queue (FreeRTOS queue)
         v
+------------------+
|   CAN Driver     |  Transmit request onto CAN bus
+------------------+


  Side channels:

  System Monitor    --->  Serial log (UART0)
  Diagnostics       <-->  CAN bus (DTC read/clear, VIN, UDS)
```

---

## Power

The CAN Interface Node receives 8-28V power from the vehicle via the Waveshare 2-CH CAN FD HAT, which regulates it to 5V. The node provides 5V to the Display Node via the USB-C VBUS line on the inter-node cable.

---

## Build Instructions

### Prerequisites

- ESP-IDF v5.1 or later (v5.2+ recommended for latest TWAI fixes).
- Target: `esp32s3`.
- Python 3.8+ (for ESP-IDF tools).

### Configure and Build

```bash
cd CAN_Interface

# Set the target (only needed once)
idf.py set-target esp32s3

# Open menuconfig to review settings (optional)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor (adjust port as needed)
idf.py -p COMx flash monitor
```

### Project Structure

```
CAN_Interface/
  CMakeLists.txt
  main/
    CMakeLists.txt
    main.c              # app_main(), boot sequence, task creation
  components/
    devices/             # Board-specific pin definitions
    can_driver/          # CAN HAL (MCP2515 Phase 1, MCP2518FD Phase 7, TWAI legacy)
    pid_db/              # PID lookup table and decode
    obd2/                # OBD-II protocol stack
    diagnostics/         # DTC, UDS, vehicle info
    poll_engine/         # Smart polling scheduler
    comm_link/           # UART TX to display node
    wifi_ap/             # DEPRECATED - moved to Display Node
    system/              # Logging, NVS, timing, task utils
```

### Sdkconfig Defaults

Key `sdkconfig.defaults` entries to consider:

```
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_TASK_WDT_TIMEOUT_S=10
CONFIG_TWAI_ISR_IN_IRAM=y
CONFIG_SPI_MASTER_IN_IRAM=y
```
