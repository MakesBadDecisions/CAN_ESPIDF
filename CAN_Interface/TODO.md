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
- [x] Add configurable bit rate (125k, 250k, 500k, 1M)
- [x] Verify MCP2515 initialization on hardware (detected, configured @ 500kbps)
- [ ] Add MCP2515 hardware filter configuration (2 masks, 6 filters)
- [ ] Implement backend selection at runtime (primary MCP2515, fallback TWAI)
- [ ] Add bus-off detection and automatic recovery logic
- [ ] Add CAN error counter reporting (TEC/REC) in get_status
- [ ] Implement TWAI backend (`twai_backend.c`) -- Phase 2, validates HAL with 2 backends; also for 1-Wire GM
- [ ] Implement MCP2518FD SPI backend (`mcp2518fd_backend.c`) -- Phase 7, drop-in behind same HAL
- [ ] Add MCP2518FD dual-channel support (CH0 and CH1 via separate CS pins on shared SPI bus)
- [ ] Add CAN-FD frame support (64-byte data field, flexible data rate) -- Phase 7
- [ ] Unit test: loopback mode TX/RX round-trip on MCP2515

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
- [ ] Implement unit conversion helpers (C to F, kPa to PSI, km/h to mph, L/100km to MPG)
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
- [x] Implement heartbeat message (sent every 500ms, includes bus status, heap, uptime)
- [x] Add link detection: monitor UART RX for heartbeat from display node
- [x] Add delivery tracking: maintain sequence numbers, log frame errors
- [x] Handle CONFIG_CMD messages received from display node (poll list changes, scan requests)
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
- [ ] Confirm UART link to display node sends/receives frames
- [ ] Test CAN bus communication with actual vehicle
