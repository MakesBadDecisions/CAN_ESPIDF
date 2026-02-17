# CAN_ESPIDF - Project Roadmap & TODO

## Development Phases

### Phase 1: Foundation (CAN Interface Core)
Get a single CAN request/response working end-to-end using proven hardware.

- [ ] **System component** - ESP-IDF logging wrapper, timing utilities, NVS config storage
- [ ] **CAN Driver HAL** - Abstract interface with MCP2515 SPI backend (proven from Arduino project)
- [ ] **PID Database** - Migrate all 165 PIDs to pure C const table with formula-type decoding
- [ ] **OBD-II Protocol** - Single-frame request builder and response parser
- [ ] **OBD-II Protocol** - ISO-TP multi-frame assembly/disassembly
- [ ] **OBD-II Protocol** - Flow control handling
- [ ] **Integration test** - Send PID request via MCP2515, receive response, decode, print to console

### Phase 2: HAL Validation & Scanning
Second backend proves the HAL abstraction, plus full vehicle discovery.

- [ ] **CAN Driver HAL** - TWAI backend (validates HAL with 2 backends; also needed for 1-Wire GM)
- [ ] **Controller Scanner** - ECU detection (0x7E0-0x7E7 ping)
- [ ] **Controller Scanner** - Supported PID discovery (range scanning 0x00-0xC0)
- [ ] **Vehicle Info** - VIN request and ISO-TP reassembly
- [ ] **Vehicle Info** - ECU name, CVN retrieval
- [ ] **DTC Manager** - Mode 03/07 stored/pending DTC read
- [ ] **DTC Manager** - UDS Mode 0x19 extended DTC read
- [ ] **DTC Manager** - DTC clear (Mode 04)
- [ ] **Poll Engine** - Priority-based smart polling scheduler
- [ ] **Poll Engine** - Adaptive interval adjustment based on success rates
- [ ] **Integration test** - Full vehicle scan -> PID discovery -> continuous polling

### Phase 3: Inter-Node Communication
UART link over USB-C cable between CAN interface and display.

- [ ] **Shared protocol** - Define binary message format (comm_protocol) with sync, length, type, CRC
- [ ] **Shared types** - PID data types shared between nodes (pid_types)
- [ ] **CAN node comm_link** - UART TX with DMA and message framing
- [ ] **Display node comm_link** - UART RX with DMA and message parsing
- [ ] **Heartbeat** - Connection health monitoring with time synchronization
- [ ] **PID streaming** - Real-time PID data batching and transmission
- [ ] **Config commands** - Display -> CAN node configuration messages
- [ ] **Error/alert forwarding** - Priority-based error messages from CAN node to display
- [ ] **Integration test** - CAN node polls PIDs -> UART over USB-C -> display node prints values

### Phase 4: Display Node Core
Basic gauge display on Elecrow CrowPanel.

- [ ] **Display Driver HAL** - Abstract display interface
- [ ] **Display Driver** - Elecrow CrowPanel driver (RGB parallel LCD, capacitive touch)
- [ ] **Display Driver** - Panel support: 5" DIS07050 and 7" DIS08070H
- [ ] **Gauge Engine** - Basic gauge rendering (numeric, bar, sweep)
- [ ] **Gauge Engine** - Layout management (configurable gauge arrangements)
- [ ] **Gauge Engine** - Alert/warning thresholds with audio feedback (I2S)
- [ ] **Gauge Engine** - Unit conversion at display level (SI to imperial per user preference)
- [ ] **Integration test** - Live gauges showing real vehicle data on CrowPanel

### Phase 5: Data Logging
SD card logging with downloadable CSV.

- [ ] **Data Logger** - SD card initialization and management (CrowPanel built-in SD slot)
- [ ] **Data Logger** - CSV file creation with HPTuners-compatible headers
- [ ] **Data Logger** - Real-time write with buffering (no data loss)
- [ ] **Data Logger** - Session management (start/stop/auto-name)
- [ ] **Data Logger** - File rotation and storage management
- [ ] **Integration test** - Log session -> CSV file -> verify in HPTuners/Excel

### Phase 6: WiFi Configuration
Web-based configuration and log download (Display Node only).

- [ ] **WiFi AP** - Access point on Display Node for system config
- [ ] **WiFi AP** - Web server with configuration UI
- [ ] **WiFi AP** - PID selection and poll rate configuration (relayed to CAN node via UART)
- [ ] **WiFi AP** - Browse and download log files via browser
- [ ] **WiFi AP** - Gauge layout configuration via web UI

### Phase 7: MCP2518FD & CAN-FD Support
Waveshare 2-CH CAN FD HAT integration -- drop-in backend behind the same HAL.

- [ ] **MCP2518FD backend** - SPI driver for MCP2518FD (similar SPI command set to MCP2515)
- [ ] **Dual channel** - CH0 and CH1 via separate CS pins on shared SPI bus
- [ ] **CAN-FD** - Extended frame support in OBD-II stack (64-byte payload, flexible data rate)
- [ ] **Multi-bus** - Simultaneous operation of MCP2518FD + TWAI backends
- [ ] **Bus detection** - Auto-detect baud rate and protocol

### Phase 8: Legacy & Advanced CAN
Extended legacy network support.

- [ ] **1-Wire GM** - Legacy GM single-wire network interface via TWAI

### Phase 9: OTA & Polish
Final refinements for daily use.

- [ ] **OTA Updates** - CAN Interface Node firmware update via Display Node over UART
- [ ] **OTA Updates** - Display Node self-update via WiFi AP
- [ ] **NVS Persistence** - Save all user settings across power cycles
- [ ] **Error recovery** - Graceful handling of bus errors, disconnects, UART link loss
- [ ] **Power management** - Sleep modes, wake-on-CAN
- [ ] **Enclosure design** - Physical packaging considerations

## Open Questions

- [x] Exact ESP32-S3 devkit model for CAN Interface Node -> ESP32-S3-N16R8-DevKitC
- [x] Display hardware -> Elecrow CrowPanel 5" (DIS07050) and 7" (DIS08070H)
- [x] Primary CAN hardware -> Waveshare 2-CH CAN FD HAT (2x MCP2518FD) -- Phase 7
- [x] Phase 1 CAN hardware -> MCP2515 standalone (proven from Arduino project)
- [x] Inter-node communication -> UART over USB-C cable (2Mbps, DMA, full duplex)
- [x] WiFi AP location -> Display Node only (CAN Interface Node is headless)
- [x] Unit conversion -> At display level (CAN node always outputs native SI units)
- [x] Time sync -> Clock offset exchange via heartbeat messages
- [x] OTA -> Application-level OTA over UART (CAN node), WiFi self-update (Display node)
- [ ] CAN transceiver model for TWAI/1-Wire GM use (SN65HVD230 vs TJA1050)
- [ ] SD card interface preference on CrowPanel (SPI vs SDMMC 1-bit vs SDMMC 4-bit)
- [ ] Pin assignments for MCP2515 module (CS, INT, SPI bus -- verify when hardware arrives)
- [ ] Log file naming convention and max file size before rotation

## Notes

- Phase 1-2 can be developed and tested with just the CAN interface node, MCP2515 module, and a serial monitor
- Phase 3 requires both nodes connected via USB-C cable but no display rendering (print to console)
- Phase 4+ requires the Elecrow CrowPanel display hardware
- Phase 7 requires the Waveshare 2-CH CAN FD HAT (arriving in 3-4 weeks)
- Phases are sequential dependencies but components within a phase can be parallel
- CAN Interface Node is headless (no WiFi, no display); all user interaction is through the Display Node
- MCP2515 and MCP2518FD use similar SPI command patterns -- the HAL built in Phase 1 translates directly
