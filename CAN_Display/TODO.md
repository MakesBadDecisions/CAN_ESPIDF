# Display Node - Task List

Status key: `[ ]` not started, `[-]` in progress, `[x]` done.

## User Workflow (Connect -> Configure -> Monitor)

The primary user interaction flow:

1. **Connect to Vehicle**
   - User presses "Connect" button on display
   - System sends scan request to CAN Interface via UART
   - CAN Interface queries vehicle for ECU IDs, VIN, Supported PIDs (Mode 01 PID 00/20/40...)
   - Results returned to Display over UART
   - Display saves vehicle info to NVS (for future sessions)
   - Display populates PID dropdown with supported PIDs

2. **Configure Gauges**
   - User selects PIDs from dropdown (up to 20 max)
   - User selects display units for each PID (metric/imperial)
   - Configuration saved to NVS
   - User presses "Connect" to start polling

3. **Active Monitoring**
   - Display sends poll list to CAN Interface via UART
   - CAN Interface polls selected PIDs continuously
   - PID values streamed to Display at configured rate
   - Gauges render live values
   - Data logged to SD card (automatic session start)

4. **Session Logging**
   - Each session creates new CSV file on SD card
   - File header includes: VIN, ECU IDs, timestamp, PID list
   - Vehicle info from NVS included in every log file
   - 20Hz logging rate for selected PIDs

## ui/ (SquareLine Studio Generated)

- [x] Add LVGL library to project
- [x] Create SquareLine Studio project targeting 4.3" (480x272)
- [x] Create Screen1 with Connect button
- [x] Add gauge panel container
- [x] Add PID dropdown (`ui_piddropdown1`)
- [x] Add unit dropdown (`ui_unnitdropdown2`)
- [x] Add gauge value label (`ui_gaugeText1`)
- [x] Add `connectCAN` event handler stub
- [x] Configure lv_conf.h for ESP32-S3 with PSRAM framebuffer
- [x] Implement `connectCAN()` -- trigger vehicle scan via comm_link
- [x] Populate PID dropdown from PID metadata received from CAN Interface
- [x] Populate unit dropdown based on selected PID
- [x] Handle PID selection change -- update unit dropdown and hot-swap active poll
- [x] Handle unit selection change -- convert and display in selected unit
- [x] Implement `pollCAN()` -- start/stop toggle, poll selected PID at 10Hz, update gauge via lv_timer
- [x] Update `ui_gaugeText1` from comm_link PID store (periodic refresh)
- [ ] Add status label for connection state (DISCONNECTED/CONNECTING/CONNECTED)
- [x] Add VIN display area for vehicle identification (ui_vehicleInfoLabel1)
- [x] Add multi-gauge layout screen with 4 configurable gauges (2x2 grid)
- [x] Table-driven gauge_widget_t[] maps slot index to LVGL widget pointers
- [x] Generic on_pid_changed / on_unit_changed callbacks (slot found by lv_event_get_target match)
- [x] LVGL timer (100ms) calls gauge_engine_update() then pushes value_str to all gauge labels
- [ ] Persist per-gauge PID and unit selections to NVS

## system/

- [x] Logging wrapper -- `SYS_LOGE/W/I/D/V` macros over `esp_log`
- [x] `system_init()` -- NVS flash init
- [x] `sys_print_device_info()` -- chip, flash, PSRAM, MAC
- [x] `sys_time_us()` / `sys_time_ms()` -- timing utilities
- [x] `sys_get_free_heap()` -- heap monitoring
- [ ] NVS config module -- load/save gauge layout, alert thresholds, WiFi credentials
- [ ] NVS config module -- define default config values for first boot
- [ ] Task watchdog registration for all tasks
- [ ] System health struct -- heap free, min heap, uptime, connection state

## comm_link/

- [x] UART initialization -- configure UART RX/TX pins (GPIO17/18), 2Mbps baud
- [x] RX task -- receive UART bytes, scan for frame start byte (0xAA)
- [x] Frame parser -- validate CRC16, extract header and payload
- [x] Message dispatcher -- route parsed messages by type using shared/comm_protocol definitions
- [x] PID data store -- fixed-size array (64 entries), holds latest value + timestamp
- [x] PID data store -- mutex protection for cross-core access
- [x] PID metadata store -- RAM-only array (96 entries) of pid/name/unit from CAN Interface
- [x] Handle MSG_PID_METADATA -- store PID names and unit strings received during scan
- [x] Fix bitmap off-by-one -- PID 1-based indexing in is_pid_supported and get_supported_pids
- [x] Heartbeat send -- TX heartbeat every 500ms
- [x] Heartbeat monitor -- track last RX timestamp, set disconnected flag after 2s timeout
- [x] Connection status API -- `comm_link_get_state()`, `comm_link_get_stats()`
- [x] TX path -- send frames to CAN Interface Node over UART
- [x] Metadata API -- get_pid_name, get_pid_unit_str, get_meta_pid_id, get_pid_meta_count
- [ ] PID data store -- stale detection (mark entries not updated within timeout)
- [x] Send vehicle scan request -- MSG_CONFIG_CMD to trigger ECU/VIN/PID scan
- [x] Handle scan response -- MSG_SCAN_STATUS with ECU IDs, VIN, supported PIDs + PID metadata
- [x] Send poll list update -- MSG_CONFIG_CMD with selected PID list
- [ ] Store vehicle info -- save VIN/ECU to NVS on successful scan
- [ ] Load vehicle info -- recall saved VIN/ECU from NVS on boot
- [ ] Callback registration -- notify UI when new PID data arrives

## display_driver/

- [x] Display HAL interface -- `display_init()`, `display_clear()`, `display_lock()`/`display_unlock()`
- [x] RGB parallel LCD driver -- ESP-IDF `esp_lcd_panel_rgb` with device header pin definitions
- [x] Double framebuffer in PSRAM (`num_fbs=2`) with LVGL `direct_mode`
- [x] Bounce buffers in SRAM (8 lines, 480x8 pixels)
- [x] VSYNC-synced buffer swap via binary semaphore + ISR callback
- [x] Dirty area synchronization between double buffers in flush callback
- [x] LVGL task on Core 1 (Priority 3, 16KB stack)
- [x] Backlight and panel enable GPIO control
- [x] Device selection via compile-time `DEVICE_DIS*` define
- [x] sdkconfig: 64B cache line, 80MHz PSRAM, ISR IRAM safe, SPIRAM fetch/rodata
- [x] CrowPanel 4.3" (DIS06043H) -- 480x272 RGB LCD
- [ ] CrowPanel 5" (DIS07050) -- 800x480 RGB LCD (untested)
- [ ] CrowPanel 7" (DIS08070H) -- 800x480 RGB LCD (untested)
- [ ] Backlight PWM control -- LEDC peripheral for brightness adjustment
- [ ] Boot splash screen -- render CAN_ESPIDF logo during initialization

## touch_driver/

- [x] XPT2046 SPI driver -- read X/Y/Z1 channels, 12-bit ADC
- [x] SPI bus init -- SPI2_HOST, DMA disabled, 1MHz clock
- [x] Touch polling task on Core 0 (Priority 2, 4KB stack)
- [x] Volatile cache struct -- Core 0 writes, Core 1 reads (zero SPI in LVGL callback)
- [x] LVGL input device registration (under display lock)
- [x] Coordinate mapping -- raw ADC to screen coordinates with calibration
- [x] 4-corner calibration screen -- crosshair targets, 5-reading average, auto inversion detect
- [x] NVS persistence -- load/save calibration data (namespace "touch_cal")
- [x] `touch_clear_calibration()` -- erase NVS, force recalibration
- [x] Fallback to header defaults when no NVS data
- [ ] Touch filtering -- debounce or median filter for noisy readings
- [ ] Long-press detection for calibration re-entry from running UI

## gauge_engine/

- [x] gauge_slot_t struct -- PID binding, base/display unit, raw/display value, formatted string
- [x] Slot array (GAUGE_MAX_SLOTS=20) with FreeRTOS mutex protection
- [x] gauge_engine_init() -- zero all slots, create mutex
- [x] gauge_engine_set_pid() -- assign PID from comm_link metadata index, set base unit
- [x] gauge_engine_set_unit() -- change display unit (0=base, 1+=alternates from pid_types)
- [x] gauge_engine_clear_slot() -- un-assign a gauge slot
- [x] gauge_engine_get_slot() -- read-only pointer to slot state
- [x] gauge_engine_get_unit_options() -- build unit dropdown string for slot's PID
- [x] gauge_engine_build_pid_options() -- build PID dropdown string from scan metadata
- [x] gauge_engine_start_polling() / stop_polling() -- aggregate PIDs into deduplicated poll list
- [x] gauge_engine_update() -- read latest values, apply unit conversion, format value_str
- [x] gauge_engine_rebuild_poll_list() -- re-aggregate after PID change during active polling
- [x] Wired into main.c boot sequence (Phase 7)
- [x] No LVGL dependency -- pure data manager
- [ ] NVS persistence -- save/load per-slot PID and unit assignments
- [ ] Alert thresholds -- warning/critical limits per slot
- [ ] Value smoothing -- low-pass filter or EMA for jittery readings
- [ ] Gauge type renderers -- sweep dials, bar graphs (currently numeric-only via LVGL labels)

## data_logger/

- [ ] SD card initialization -- SPI bus config using device header SD pins, mount FAT filesystem
- [ ] SD card detection -- card detect or mount/unmount retry logic
- [ ] Session management -- start session (create new CSV), stop session (flush and close)
- [ ] Session auto-naming -- `LOG_YYYYMMDD_HHMMSS_NNN.csv` format
- [ ] CSV header generation -- build header row from actively-polled PIDs
- [ ] CSV row writing -- snapshot PID data store, format values, write row with timestamp
- [ ] Write buffering -- RAM ring buffer, flush to SD every 50ms or on threshold
- [ ] File rotation -- close and start new at configurable size limit
- [ ] Storage monitoring -- track SD card free space, warn when low

## wifi_manager/

- [ ] WiFi AP initialization -- configure SSID, password, channel
- [ ] AP on-demand start -- enable AP via touch button or NVS setting
- [ ] HTTP server -- lightweight httpd for file serving and API endpoints
- [ ] Log file browser -- HTML page listing CSV files on SD card
- [ ] Log file download -- serve CSV files for direct download
- [ ] Gauge config UI -- HTML page for layout, PID assignments, alert thresholds
- [ ] CAN node config proxy -- relay settings to CAN node over UART
- [ ] System status page -- connection state, heap, SD usage

## devices/

- [x] Create DIS06043H (4.3") device header with pin assignments
- [x] Create device.h selector header for build-time device selection
- [x] Create custom board JSON for CrowPanel 4.3" (4MB Flash, 2MB Quad PSRAM)
- [ ] Verify DIS07050 (5") device header pin assignments against actual hardware
- [ ] Verify DIS08070H (7") device header pin assignments against actual hardware

## main/

- [x] `main.c` -- app_main entry point with boot sequence
- [x] `main.c` -- system_init -> display_init -> touch_init -> calibration -> ui_init -> comm_link -> gauge_engine_init
- [x] `main.c` -- error handling for init failures (continue with degraded functionality)
- [x] `main.c` -- gauge_engine_init() wired as Phase 7 after comm_link_start()
- [x] `CMakeLists.txt` -- register all components
- [ ] `partitions.csv` -- verify partition table for OTA support

## Integration and Testing

- [ ] Loopback test -- comm_link receives simulated UART messages, verify PID store updates
- [ ] Display test -- render all gauge types with known values, visual verification
- [ ] Touch test -- verify XPT2046 touch input and calibration accuracy
- [ ] Logger test -- run logging session, verify CSV output matches HPTuners import format
- [ ] WiFi test -- connect to AP, browse logs, download file, verify file integrity
- [ ] Alert test -- inject out-of-range values, verify warning and critical alert behavior
- [ ] Stress test -- sustained 20Hz PID updates across 30+ PIDs, verify no dropped data
- [ ] Endurance test -- 1-hour continuous logging, verify no file corruption or heap leak
- [ ] Cross-node integration -- connect to CAN Interface Node via USB-C, live vehicle data
