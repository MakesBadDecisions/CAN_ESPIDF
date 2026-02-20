# CAN_ESPIDF CAN Interface Node -- TODO

Tracks what needs to be built for each component. Items are roughly ordered by dependency (earlier items unblock later ones).

Status key: `[ ]` not started, `[-]` in progress, `[x]` done.

---

## sys_mgr/ -- System Manager

- [x] Define state machine (INIT → CONNECTING_CAN → CONNECTING_COMM → RUNNING ↔ ERROR → FATAL)
- [x] Implement sys_mgr_init() and sys_mgr_start()
- [x] Implement task registration and watchdog checking
- [x] Implement error reporting with automatic recovery triggers
- [x] Implement state transition validation
- [x] Implement statistics tracking (uptime, counters, heap)
- [x] Create monitor task (periodic health checks, status logging)
- [ ] Add CAN recovery callback integration (call can_driver_reinit)
- [ ] Add comm link recovery coordination
- [ ] Add NVS change notification system
- [ ] Integrate with ESP-IDF task watchdog (esp_task_wdt)

---

## can_driver/ -- CAN Hardware Abstraction Layer

- [x] Define unified CAN driver API in `can_driver.h` (open, close, send, receive, get_status)
- [x] Implement MCP2515 SPI backend (`mcp2515.c`) -- SPI bus init, register read/write, CAN frame TX/RX
- [x] Add MCP2515 interrupt-driven RX using GPIO ISR on INT pin
- [x] Fix MCP2515 ISR task watchdog starvation -- drain loop (read CANINTF until 0) + taskYIELD() every 8 iterations
- [x] Add configurable bit rate (125k, 250k, 500k, 1M)
- [x] Verify MCP2515 initialization on hardware (detected, configured @ 500kbps)
- [x] Add MCP2515 hardware filter configuration (2 masks, 6 filters) -- dynamic ECU-based filtering with physical addressing discovery
- [ ] Implement backend selection at runtime (primary MCP2518FD, fallback MCP2515/TWAI)
- [x] Add bus-off detection and automatic recovery logic (hard SPI reset + full reconfigure, ISR drain-loop protection, rate-limited logging)
- [x] Add CAN error counter reporting (TEC/REC) in get_status
- [ ] Implement TWAI backend (`twai_backend.c`) -- Phase 2, validates HAL with 2 backends; also for 1-Wire GM
- [x] Implement MCP2518FD SPI backend (`mcp2518fd.c`) -- full driver: SPI primitives, mode control, bitrate config, FIFO/filter setup, TX/RX, ISR task
- [x] Wire MCP2518FD into can_driver.c dispatch layer (all 11 API calls)
- [x] Update CAN_CONFIG_DEFAULT to MCP2518FD + 40MHz crystal
- [x] Revert CAN_CONFIG_DEFAULT to MCP2515 + 8MHz (current hardware still MCP2515; swap comment in can_driver.h when ready)
- [ ] Add MCP2518FD dual-channel support (CH0 and CH1 via separate CS pins on shared SPI bus)
- [ ] Add CAN-FD frame support (64-byte data field, flexible data rate)
- [ ] Unit test: loopback mode TX/RX round-trip on MCP2518FD

---

## MCP2518FD Transition Plan (MCP2515 → MCP2518FD)

**Status:** MCP2518FD driver implementation **COMPLETE**. All code compiles clean. Pending: physical hardware swap and vehicle validation.

### What Changed

| Layer | Change |
|-------|--------|
| `mcp2518fd_defs.h` | NEW -- Full register map (CiCON, CiNBTCFG, CiINT, FIFO, filter, OSC, message objects, bit timing tables) |
| `mcp2518fd.h` | NEW -- Config struct, full API surface (14 functions) |
| `mcp2518fd.c` | NEW -- ~600 lines: SPI primitives, mode control, FIFO/filter setup, TX/RX, ISR task, init/deinit |
| `can_driver.c` | UPDATED -- All 11 MCP2518FD dispatch cases wired (init, deinit, start, stop, send, receive, etc.) |
| `can_driver.h` | UPDATED -- Added 20/40MHz crystal enums, CAN_CONFIG_DEFAULT → MCP2518FD + 40MHz |
| `CMakeLists.txt` | UPDATED -- Added mcp2518fd.c to SRCS |
| `ESP32-S3-N16R8-DevKitC.h` | UPDATED -- Added MCP2518FD_CRYSTAL_HZ define |
| `obd2/` | None -- uses can_driver API unchanged |
| `poll_engine/` | None -- uses OBD2 API unchanged |
| `comm_link/` | None -- uses PID data unchanged |
| `pid_db/` | None -- decoding logic unchanged |

### What Stays the Same

- All code above `can_driver.c` is backend-agnostic
- SPI bus (SPI2/FSPI) already initialized -- same MOSI/MISO/SCK pins
- CS=GPIO10, INT=GPIO9 -- same pins, physical swap only
- CAN bitrate 500kbps -- MCP2518FD supports same rates
- OBD-II framing is CAN 2.0B -- MCP2518FD runs classic CAN mode

### Implementation Steps

1. [x] Create `mcp2518fd_defs.h` -- register definitions (CiCON, CiFIFOCON, CiINT, etc.)
2. [x] Create `mcp2518fd.h` -- config struct and API declarations
3. [x] Create `mcp2518fd.c` -- SPI command layer (read/write registers, read/write FIFO RAM)
4. [x] Implement `mcp2518fd_init()` -- reset, configure clock, set classic CAN mode @ 500kbps
5. [x] Implement `mcp2518fd_send()` -- write TX FIFO, trigger transmission
6. [x] Implement `mcp2518fd_receive()` -- read RX FIFO via interrupt or polling
7. [x] Implement interrupt handler -- GPIO ISR on INT pin, push frames to rx_queue
8. [x] Register as backend in `can_driver.c`, update default config
9. [ ] Test: loopback mode on bench (no vehicle needed)
10. [ ] Test: vehicle scan -- verify same 49 PIDs, VIN, same decode results
11. [ ] Test: 10Hz polling -- verify identical data to MCP2515 baseline
12. [ ] (Future) Add CAN-FD mode with 64-byte payloads and flexible data rate
13. [ ] (Future) Add Channel 1 support (CS=GPIO46, INT=GPIO3) for dual-bus

### Key MCP2518FD vs MCP2515 Differences

| Feature | MCP2515 | MCP2518FD |
|---------|---------|-----------|
| CAN standard | CAN 2.0B only | CAN 2.0B + CAN FD |
| Max data rate | 1 Mbps | 8 Mbps (FD data phase) |
| Payload size | 8 bytes | 8 (classic) / 64 (FD) bytes |
| SPI clock | 10 MHz | 20 MHz |
| TX FIFOs | 3 fixed buffers | Configurable TEF + up to 31 FIFOs |
| RX FIFOs | 2 fixed buffers | Configurable up to 31 FIFOs |
| RAM | None | 2 KB device RAM for FIFOs |
| SPI protocol | Byte-level commands | 4-byte SPI header + word-aligned data |
| Filters | 2 masks, 6 filters | Up to 32 filters with individual masks |
| Clocking | External crystal | External crystal or PLL |

---

## pid_db/ -- PID Database

- [x] Define `pid_entry_t` struct (pid, name, type, unit, formula, params, byte_count, service)
- [x] Define formula types enum (RAW, LINEAR, BITFIELD, RATIO, etc.)
- [x] Build unified pid_tables.c with compact format
- [x] Add OBD-II Mode 0x01 PIDs (0x00-0xA4) - 100+ entries
- [x] Add GM Mode 0x22 PIDs - 68 entries (UNVERIFIED - need vehicle testing)
- [x] Implement table access functions (pid_db_get_obd2_table, pid_db_get_gm_table)
- [x] Implement `pid_db_lookup()` -- find entry by PID number, return pointer or NULL
- [x] Implement `pid_db_decode()` -- take raw bytes + pid_entry_t, return float engineering value
- [x] Implement `pid_db_unit_str()` -- convert unit_t enum to display string (RPM, kPa, %, C, etc.)
- [x] Implement unit conversion helpers (C to F, kPa to PSI, km/h to mph) -- via shared/pid_types
- [ ] Build compile-time hash index or sorted array with binary search for fast lookup
- [ ] Add PID supported-bitmap helpers (decode Mode 01 PID 0x00/0x20/0x40/... responses)
- [ ] Validate GM PID addresses against actual vehicle responses
- [ ] Validate table: no duplicate PIDs, all formulas produce values within min/max

---

## obd2/ -- OBD-II Protocol Stack

- [x] Define OBD-II request struct (mode, pid, tx_id, timeout) and response struct (status, mode, pid, ecu_id, data, negative_code)
- [x] Implement request builder: Mode 01 (current data), Mode 02 (freeze frame), Mode 09 (vehicle info), Mode 22 (extended)
- [x] Implement single-frame response parser (match response mode = request mode + 0x40)
- [x] Implement ISO-TP multi-frame RX: first frame detection, flow control TX, consecutive frame reassembly with timeout
- [x] Implement flow control frame generation (block size, separation time from config)
- [x] Add request/response correlation (semaphore signaling, pending response capture)
- [x] Add response timeout handling (configurable, default 100 ms, retry support)
- [x] Add negative response (0x7F) parsing and retry logic (0x78 response pending)
- [x] Implement multi-ECU response handling (0x7DF broadcast → accept any 0x7E8-0x7EF)
- [x] Write `CMakeLists.txt`
- [x] Implement helper functions: obd2_read_vin, obd2_read_dtcs, obd2_clear_dtcs, obd2_get_supported_pids
- [ ] Implement ISO-TP multi-frame TX (for long requests, rare)
- [ ] Test: single-frame request/response for RPM (PID 0x0C)
- [ ] Test: multi-frame ISO-TP response (VIN retrieval, 17+ bytes)
- [x] Wire obd2_process_frame() into CAN RX task (via callback system)

---

## diagnostics/ -- DTC and UDS

- [x] Define DTC entry struct (code string, status byte, freeze frame flag)
- [x] Implement Mode 03 -- read stored DTCs, parse 2-byte DTC pairs into P/B/C/U codes
- [ ] Implement Mode 07 -- read pending DTCs
- [x] Implement Mode 04 -- clear DTCs (with confirmation/safety gate)
- [ ] Implement UDS service 0x19 (ReadDTCInformation) sub-functions: reportDTCByStatusMask, reportDTCSnapshotRecord
- [x] Implement VIN retrieval (Mode 09 PID 02) with ISO-TP multi-frame handling
- [ ] Implement CVN retrieval (Mode 09 PID 06)
- [ ] Implement ECU name retrieval (Mode 09 PID 0A)
- [ ] Add DTC storage manager -- store last-read DTCs in RAM, optionally persist to NVS
- [ ] Add UDS session control (0x10) and security access (0x27) stubs for future use
- [x] Write `CMakeLists.txt`
- [x] Implement ECU scan (discover responding ECUs 0x7E8-0x7EF)
- [x] Implement supported PID discovery and caching
- [x] Wire diagnostics to comm_link (send VIN/DTCs to display node)

---

## poll_engine/ -- Smart Polling Scheduler

**Design Decision:** Using priority levels 1-5 instead of poll groups:
- Priority 1: Slow (~5 second interval)
- Priority 2: Low (~1 second interval)
- Priority 3: Normal (~200ms interval)
- Priority 4: High (~100ms interval)
- Priority 5: Realtime (~100ms, 10 Hz)

PID priorities are stored in NVS and user-adjustable via display node.
No poll groups - each PID has individual priority setting.

- [x] Define poll job struct (pid, priority 1-5, base_interval_ms, next_run_tick, last_value, changed_flag)
- [x] Implement priority run queue (sorted by next_run_tick, tie-break by priority)
- [x] Implement NVS priority storage (key: PID number, value: priority 1-5, default: 3)
- [x] Implement scheduler tick: pop due jobs, submit OBD-II requests, update next_run_tick
- [x] Implement adaptive interval logic: shorten interval on consistent success, lengthen on failure/timeout
- [x] Add bandwidth limiter: max N outstanding requests, backpressure when TX queue is full
- [x] Add PID enable/disable at runtime (from config or command from display node)
- [x] Add priority change API (set_pid_priority, get_pid_priority) with NVS persistence
- [x] Implement "discovery sweep" on first connect: send PID 0x00/0x20/0x40/... to detect supported PIDs, then build poll list
- [x] Add statistics: average response time per PID, success rate, polls per second
- [x] Write `CMakeLists.txt`
- [x] Wire poll_engine to comm_link (forward PID values to display node via callback)

---

## comm_link/ -- UART TX to Display Node

- [x] Initialize UART1 (GPIO 17 TX, GPIO 18 RX, 2Mbps, DMA-backed)
- [x] Implement UART frame TX: start byte + header + payload + CRC16
- [x] Implement UART frame RX: scan for start byte, validate CRC, dispatch by message type
- [x] Implement PID data batching: collect changed PIDs since last TX, pack into UART frames
- [x] Implement heartbeat message (sent every 500ms, includes node_state, can_status, heap, uptime)
- [x] Heartbeat can_status reflects real CAN driver state (RUNNING/BUS_OFF/ERROR_PASSIVE)
- [x] Add link detection: monitor UART RX for heartbeat from display node
- [x] Add delivery tracking: maintain sequence numbers, log frame errors
- [x] Handle CONFIG_CMD messages received from display node (poll list changes, scan requests)
- [x] Implement `comm_link_send_pid_metadata()` -- send PID name/unit batches to Display Node
- [x] Write `CMakeLists.txt`

---

## wifi_ap/ -- REMOVED

WiFi AP functionality has been moved to the Display Node. The CAN Interface
Node is a headless backend. See `CAN_Display/components/wifi_manager/`.

---

## system/ -- System Utilities

- [x] Implement logging wrapper: SYS_LOG* macros wrapping esp_log
- [x] Implement NVS config module: `sys_config_get_*()` / `sys_config_set_*()` for typed key access
- [x] Implement timing utilities: `sys_time_us()`, `sys_time_ms()`, `sys_timeout_check()`
- [x] Implement device info printing (chip, flash, PSRAM, MAC, heap)
- [x] Implement heap helpers: `sys_get_free_heap()`, `sys_get_min_free_heap()`
- [ ] Define all config keys as an enum/table (CAN_BITRATE, POLL_PRESET, UART_BAUD, etc.)
- [ ] Add task stack high-water-mark helpers
- [ ] Write `CMakeLists.txt`

---

## devices/ -- Board Definitions

- [ ] Complete ESP32-S3-N16R8-DevKitC.h pin assignments (verify against actual hardware)
- [ ] Create device_test utility to verify flash size, PSRAM size, chip model, MAC address
- [ ] Add Waveshare 2-CH CAN FD HAT device header (pin mappings, SPI config)
- [ ] Add MCP2518FD standalone module device header (when hardware arrives)

---

## main/ -- Application Entry Point

- [x] Write `app_main()` implementing the boot sequence (see README)
- [x] Orchestrate component initialization in correct order
- [x] Create `init_components()` function for all component init
- [x] Create placeholder `create_tasks()` function
- [x] Integrate sys_mgr init and start
- [x] Extend scan handler -- send VEHICLE_INFO → PID_METADATA batches → SCAN_STATUS_COMPLETE
- [x] Scan guard -- check CAN driver state before scan, attempt recovery, send SCAN_STATUS_FAILED if CAN not running
- [ ] Implement actual task creation (CAN, Poll, Comm) when components ready
- [ ] Add graceful shutdown handling
- [ ] Create all FreeRTOS tasks with correct core pinning, priority, and stack sizes
- [ ] Add top-level error handling: if a critical init fails (CAN driver), log and halt or retry
- [ ] Add serial command handler on UART0 for debug (optional, low priority)
- [ ] Write `CMakeLists.txt` with component dependencies

---

## Project-Level

- [x] Write root `CMakeLists.txt` for the CAN_Interface project
- [x] Create `sdkconfig.defaults` with FreeRTOS tick rate, PSRAM, task WDT timeout
- [x] Create `partitions.csv` (NVS, app partitions)
- [x] Verify full build with PlatformIO (zero errors, zero warnings)
- [x] Flash to hardware and confirm MCP2515 initializes (detected, 500kbps configured)
- [x] Confirm UART link to display node sends/receives frames
- [x] Test CAN bus communication with actual vehicle (Ford F-150, VIN: 1FTLR4FEXBPA98994, 49 PIDs)
