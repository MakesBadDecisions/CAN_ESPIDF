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
   - File header includes: VIN, creation time, channel info (PID numbers, names, units)
   - Vehicle info from NVS included in every log file
   - Logging rate matches poll rate (10Hz default)
   - Hardware addon sensor data logged alongside CAN PIDs (when sensors connected)

5. **Web Configuration Portal** (via WiFi AP)
   - Connect phone/laptop to display's WiFi AP
   - Full web UI for all system settings:
     - Gauge layout, units, alerts, backlight
     - CAN Interface settings (bitrate, controller, filters, poll rates)
     - Extended poll list (poll more than displayed), custom PIDs, math channels
     - Log file browsing and download
     - DTC read/clear, freeze frames, ECU info
     - Hardware addon setup (sensors, buzzers, RGB)
     - Vehicle profiles (auto-detect by VIN)

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
- [x] Add status label for connection state (DISCONNECTED/CONNECTING/CONNECTED)
- [x] Status label shows dual state: "UART: OK | CAN: OFF" with tri-color coding (green=both OK, yellow=partial, red=both down)
- [x] Add VIN display area for vehicle identification (ui_vehicleInfoLabel1)
- [x] Add multi-gauge layout screen with 4 configurable gauges (2x2 grid)
- [x] Table-driven gauge_widget_t[] maps slot index to LVGL widget pointers
- [x] Generic on_pid_changed / on_unit_changed callbacks (slot found by lv_event_get_target match)
- [x] LVGL timer (100ms) calls gauge_engine_update() then pushes value_str to all gauge labels
- [x] Persist per-gauge PID and unit selections to NVS
- [x] Restore saved gauge config after scan -- sync dropdown selections to NVS state
- [x] Add Screen2 (Settings) -- SquareLine export with brightness slider, back button, WiFi button, system panel, color wheel panel
- [x] Settings button callback -- registered as additional LV_EVENT_CLICKED in ui_events_post_init() (SquareLine-safe)
- [x] Deferred Screen2 init -- 200ms one-shot LVGL timer waits for _ui_screen_change() to complete before manipulating widgets
- [x] Brightness slider wired -- on_brightness_changed callback (LV_EVENT_VALUE_CHANGED) calls display_set_brightness()
- [x] Brightness label update -- slider value reflected in ui_brightnessLabel text
- [x] Color wheel -- created dynamically in ui_colorWhellPanel each Screen2 visit (not SquareLine-managed)
- [x] Color wheel hex label -- #RRGGBB centered inside wheel, updates on drag
- [x] Color wheel saturation guard -- saved colors with HSV S<20 (white/gray) replaced with default blue to prevent invisible arc
- [x] Theme color application -- ui_apply_theme_color() sets text color on all labels, border color on all buttons/panels/gauges/dropdowns
- [x] Theme color NVS persistence -- ui_save_theme_color()/ui_load_theme_color() in namespace "display" key "theme"
- [x] Theme color boot restore -- main.c Phase 8 calls ui_load_theme_color() + ui_apply_theme_color() after ui_events_post_init()
- [x] Theme color re-apply on Screen2 entry -- settings_screen_init_values() reapplies to freshly created Screen2 widgets

## system/

- [x] Logging wrapper -- `SYS_LOGE/W/I/D/V` macros over `esp_log`
- [x] `system_init()` -- NVS flash init
- [x] `sys_print_device_info()` -- chip, flash, PSRAM, MAC
- [x] `sys_time_us()` / `sys_time_ms()` -- timing utilities
- [x] `sys_get_free_heap()` -- heap monitoring
- [x] NVS config module -- load/save gauge layout, alert thresholds, WiFi credentials
- [ ] NVS config module -- define default config values for first boot
- [ ] Task watchdog registration for all tasks (implement when stability concern arises)
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
- [x] CAN status API -- `comm_link_get_can_status()` parses heartbeat can_status field
- [x] TX path -- send frames to CAN Interface Node over UART
- [x] Metadata API -- get_pid_name, get_pid_unit_str, get_meta_pid_id, get_pid_meta_count
- [x] PID data store -- stale detection (mark entries not updated within timeout)
- [x] Send vehicle scan request -- MSG_CONFIG_CMD to trigger ECU/VIN/PID scan
- [x] Handle scan response -- MSG_SCAN_STATUS with ECU IDs, VIN, supported PIDs + PID metadata
- [x] Send poll list update -- MSG_CONFIG_CMD with selected PID list
- [x] Store vehicle info -- save VIN/ECU to NVS on successful scan (namespace "vehicle")
- [x] Load vehicle info -- recall saved VIN/ECU from NVS on boot, show on display
- [x] Callback registration -- notify UI when new PID data arrives (gauge_engine callback + LVGL timer)
- [x] Remote system info API -- `comm_link_get_remote_heap_kb()`, `comm_link_get_remote_uptime_ms()`, `comm_link_get_remote_node_state()` from heartbeat
- [x] DTC storage -- parse `MSG_DTC_LIST` into `comm_dtc_entry_t` cache with mutex
- [x] DTC API -- `comm_link_request_dtcs()`, `comm_link_clear_dtcs()`, `comm_link_get_dtcs()`, `comm_link_get_dtc_count()`, `comm_link_has_dtc_data()`

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
- [x] Waveshare 2.1" (WS_TOUCH_LCD_21) -- 480x480 ST7701S SPI+RGB, PWM backlight via LEDC
- [x] Backlight PWM control -- LEDC peripheral for brightness adjustment (Waveshare)
- [x] Brightness API -- display_set_brightness(uint8_t percent) and display_get_brightness()
- [x] Brightness NVS persistence -- namespace "display" key "bl_pct", restored on boot
- [x] Waveshare backlight init -- gpio_config → LEDC timer → LEDC channel → ledc_fade_func_install(0), matching demo ST7701S.c
- [x] CrowPanel backlight -- GPIO on/off (percent > 0 = on, 0 = off)
- [ ] CrowPanel 5" (DIS07050) -- 800x480 RGB LCD (untested)
- [ ] CrowPanel 7" (DIS08070H) -- 800x480 RGB LCD (untested)
- [x] Boot splash screen -- BMP loader renders SD card image during initialization

## touch_driver/

- [x] XPT2046 SPI driver -- read X/Y/Z1 channels, 12-bit ADC
- [x] SPI bus init -- SPI2_HOST, DMA enabled (SPI_DMA_CH_AUTO), 1MHz clock, max_transfer_sz=4096 (shared with SD card)
- [x] Touch polling task on Core 0 (Priority 2, 4KB stack)
- [x] Volatile cache struct -- Core 0 writes, Core 1 reads (zero SPI in LVGL callback)
- [x] LVGL input device registration (under display lock)
- [x] Coordinate mapping -- raw ADC to screen coordinates with calibration
- [x] 4-corner calibration screen -- crosshair targets, 5-reading average, auto inversion detect
- [x] NVS persistence -- load/save calibration data (namespace "touch_cal")
- [x] `touch_clear_calibration()` -- erase NVS, force recalibration
- [x] Fallback to header defaults when no NVS data
- [x] CST820 I2C capacitive touch backend (Waveshare) -- polling task, LVGL indev, TCA9554 reset
- [x] Multi-controller HAL -- compile-time #if TOUCH_TYPE_XPT2046 / TOUCH_TYPE_CST820
- [ ] Touch filtering -- debounce or median filter for noisy readings (low priority)
- [ ] Long-press detection -- hidden menus, easter eggs, fun features (deferred)

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
- [x] NVS persistence -- save/load per-slot PID and unit assignments (namespace "gauge_cfg")
- [x] Auto-save on set_pid / set_unit / clear_slot with suppress-save guard during load
- [x] Virtual PID infrastructure -- VPID_BASE (0xFF00+), GAUGE_IS_VIRTUAL() macro, bypass comm_link for virtual channels
- [x] IMU as virtual PID -- "IMU" appears in PID dropdown (appended after CAN PIDs), VPID_IMU = 0xFF00
- [x] IMU unit dropdown -- "G-Load" / "Tilt" mode selector (display_unit stores mode index)
- [x] IMU bubble renders inside gauge slot panel -- replaces dedicated gyroPanel1, inner container avoids dropdowns
- [x] Virtual PID NVS persistence -- VPID_IMU stored/restored from NVS naturally, display mode preserved
- [x] Virtual PID poll list -- virtual PIDs excluded from CAN poll list, rebuild_poll_list handles IMU-only case
- [x] IMU uniqueness enforcement -- only one gauge slot can host IMU at a time (ui_events.c on_pid_changed)
- [x] Alert thresholds -- warning/critical limits per slot (warn/crit/max per PID, NVS blob, evaluate_alert with bidirectional detection)
- [x] Alert visual effects -- yellow→red gradient border (warn zone), flashing red border + red text (critical), theme color restore on clear
- [ ] Gauge type renderers -- sweep dials, bar graphs, symbols, arc gauges (big project, later -- see Project Vision below)

## data_logger/

- [x] SD card initialization -- SPI bus, FAT mount on shared SPI2_HOST with touch (DMA enabled)
- [x] SD card initialization -- SDMMC 1-bit mount (Waveshare, TCA9554 D3 enable)
- [x] Timestamp file naming -- MMDDHHMM.csv using RTC + timezone, 8.3 FAT compatible (replaces sequential counter, no NVS tracking)
- [x] ~~Sequential file naming -- log1.csv, log2.csv... with NVS-persisted counter~~ (replaced by timestamp naming)
- [x] HP Tuners CSV header -- bracketed sections, channel info (PID numbers, names, units)
- [x] Data row writing -- time offset + raw CAN values, empty fields for stale/missing
- [x] Write buffering -- 4KB buffer, flushed on full or session stop
- [x] Session auto-start/stop -- wired into pollCAN() start/stop in ui_events.c
- [x] Row logging from PID callback -- logger_log_row() called on each new_data in gauge_update_cb
- [x] Non-blocking mutex -- 5ms timeout, skip row rather than block LVGL
- [x] Wired into main.c boot sequence (Phase 8, non-fatal on failure)
- [x] Storage monitoring -- SD free space via FATFS f_getfree(), LOW! warning when < 50MB (status API, System Info panel, portal)
- [x] File listing API -- enumerate log files for WiFi download (log_handler.c)

## wifi_manager/ -- Web Configuration Portal

The WiFi Manager is a full-featured configuration portal served over a WiFi AP.
It is the primary settings hub for the entire system (both Display and CAN Interface).
See `components/wifi_manager/README.md` for full architecture, API spec, and QR code design.

### Phase 1: WiFi AP + QR Code + System Status
- [x] Enable `LV_USE_QRCODE 1` in `lv_conf.h`
- [x] WiFi AP init -- APSTA mode, SSID from MAC suffix (CAN_<4hex>), random 8-char password
- [x] Random password generator -- `esp_random()`, unambiguous charset (no 0/O/o, 1/l/I)
- [x] WiFi event handler -- track client connect/disconnect, update client count
- [x] QR screen (LVGL) -- WiFi QR code centered on screen, SSID and password as text below
- [x] QR stage 2 -- swap to URL QR (http://192.168.4.1/) when first client connects
- [x] QR screen close button -- stop AP, return to gauge UI
- [x] HTTP server start -- `httpd_start()`, register URI handlers
- [x] `GET /api/status` -- system info JSON (heap, uptime, link_state, sd_mounted)
- [x] `GET /` -- embedded HTML single-page app (portal_handler.c)
- [x] NVS settings -- load/save ssid, pass, channel, ap_on (namespace "wifi")
- [x] NVS password behavior -- empty = random each start, non-empty = persistent

### Phase 2: Log File Browser + Download
- [x] `GET /api/logs` -- list CSV files on SD card (name, size, date)
- [x] `GET /api/logs/:filename` -- chunked file download (4KB chunks via httpd_resp_send_chunk)
- [x] `DELETE /api/logs/:filename` -- delete log file
- [x] `GET /api/sd` -- SD card status (total_bytes, free_bytes, mounted)
- [x] ~~Log session control~~ -- not needed; logging auto-starts/stops with polling

### Phase 3: Display Configuration API
- [x] `GET /api/gauges` -- read all 20 gauge slots (pid_id, units, value)
- [x] `POST /api/gauges` -- set PID and unit for a slot
- [x] `DELETE /api/gauges` -- clear a slot
- [x] `GET /api/gauges/pids` -- PID dropdown options from gauge_engine
- [x] `GET /api/gauges/units` -- unit dropdown options per slot
- [x] `GET/POST /api/config/splash` -- splash duration read/write (boot_splash API)
- [x] `GET/POST /api/config/backlight` -- backlight brightness (display_driver API)
- [x] `GET/POST /api/config/wifi` -- WiFi AP SSID, password, channel settings
- [x] `GET/POST /api/config/autopoll` -- auto-poll on reconnect toggle
- [x] Alert thresholds -- warning/critical limits per gauge slot (POST /api/pids/alerts endpoint, PID table columns for warn/crit/max, saveAlerts() JS)
- [ ] Touch calibration -- trigger recalibration from web UI

### Phase 4: HTML Frontend (Embedded SPA)
- [x] Status page -- system status, link state, SD usage, uptime, time sync
- [x] Logs page -- file list table, download/delete buttons
- [x] PIDs page -- PID browser with poll toggle per PID
- [x] Workshop page -- gauge slot grid, PID dropdown, unit dropdown per slot
- [x] Settings page -- splash duration, backlight slider, WiFi SSID/pass, auto-poll toggle
- [x] Embed as const char[] -- PORTAL_HTML in portal_handler.c (no SPIFFS needed)
- [x] Mobile-friendly layout -- responsive CSS, tabs, cards, toast notifications
- [x] Auto time sync -- JS grabs client epoch + timezone on page load, POSTs to /api/settime

### Phase 5: CAN Interface Proxy (over UART)
- [x] `GET /api/pids` -- PID list with names, units, supported status from CAN Interface
- [x] `GET /api/pids/poll` -- current poll list state
- [x] `POST /api/pids/poll` -- set poll list (PIDs, rate) via comm_link
- [x] `POST /api/can/scan` -- trigger vehicle scan via comm_link (scan_handler.c)
- [x] `GET /api/can/vehicle` -- VIN, protocol, ECU count, PID count, DTC count, scan status, ECU name, CalID, CVN
- [x] Scan button on web portal -- Vehicle card with Scan Vehicle button on Status page
- [x] Display scan button renamed "Connect" → "Scan" (existing Screen1 button, no layout change)

### Phase 6: Vehicle Tab + DTC
- [x] Vehicle tab in portal -- dedicated page with VIN, ECU count, protocol info, ECU name, CalID, CVN, scan button
- [x] DTC storage on Display comm_link -- `comm_dtc_list_t` parsed from Interface, cached with mutex
- [x] DTC HTTP API -- `GET /api/can/dtcs`, `POST /api/can/dtcs/read`, `POST /api/can/dtcs/clear`
- [x] DTC viewer on Vehicle tab -- table with code, system, type; Read/Clear buttons
- [x] DTC comm_link API -- `comm_link_request_dtcs()`, `comm_link_clear_dtcs()`, `comm_link_get_dtcs()`
- [x] CAN_Interface CMD_READ_DTCS handler -- routes to diagnostics_read_dtcs via scan_task
- [x] CAN_Interface CMD_CLEAR_DTCS handler -- routes to diagnostics_clear_dtcs via scan_task
- [x] Fix wire format -- comm_link_send_dtcs uses `comm_dtc_list_t` with `comm_dtc_entry_t` entries (was ad-hoc)
- [x] DTC type bitmask -- type field is OR'd bitmask (stored|pending|permanent), scan_handler renders combined strings
- [x] MIL status on Vehicle tab -- 3-state indicator (green OFF, yellow OFF with emission DTCs, red ON)
- [x] Mode 03 + 07 + 0A combined query -- stored, pending, permanent DTCs with dedup-merge
- [x] PID 0x01 monitor status -- MIL lamp + emission DTC count decoded during scan, sent in vehicle info
- [ ] Tests and bi-directional controls -- future scope on Vehicle tab

### Phase 7: IMU Setup Tab
- [ ] IMU Setup tab in portal -- dedicated configuration and calibration UI
- [ ] Configurable G range -- adjustable ±G mapping via IMU Setup tab
- [ ] Direction-of-travel calibration -- drive straight for ~5s, IMU determines forward vector (yaw offset)
- [ ] IMU recalibration trigger -- recalibrate gyro bias from web UI

### Phase 8+ (Future / Deferred)
- [ ] Custom PID editor -- user-defined address, mode, name, formula (big project, later)
- [ ] Custom math channels -- virtual PIDs derived from real PIDs (big project, later)
- [ ] Freeze frame viewer -- snapshot data for stored DTCs
- [ ] CAN bitrate change -- new CONFIG_CMD + CAN Interface handler
- [ ] Vehicle profiles -- per-VIN gauge configs with standard defaults, auto-detect by VIN (later)
- [x] Display themes -- basic theme coloring via colorwheel (text + border colors, NVS-persisted). Full LVGL style system is future scope.
- [x] Time sync -- POST /api/settime + GET /api/time endpoints (time_handler.c), JS auto-sync on portal load
- [x] PCF85063 RTC -- I2C driver, UTC storage, NVS timezone offset, boot-time system clock restore
- [ ] WebSocket live data -- real-time push alternative to polling
- [ ] OTA update via AP -- display updates through WiFi AP, CAN Interface updates via UART command from display (map full workflow before implementation)
- [x] Buzzer integration -- TCA9554 EXIO8, triggers on ALERT_MAX level, silences when value drops below MAX

## devices/

- [x] Create DIS06043H (4.3") device header with pin assignments
- [x] Create device.h selector header for build-time device selection
- [x] Create custom board JSON for CrowPanel 4.3" (4MB Flash, 2MB Quad PSRAM)
- [x] Create Waveshare ESP32-S3-Touch-LCD-2.1 device header (ws_touch_lcd_2_1.h)
- [x] Create custom board JSON for Waveshare 2.1" (16MB Flash, 8MB Octal PSRAM)
- [x] Create partitions_16mb.csv for 16MB flash boards
- [x] Create sdkconfig.waveshare_2_1 (Octal PSRAM, 16MB flash)
- [ ] Verify DIS07050 (5") device header pin assignments against actual hardware
- [ ] Verify DIS08070H (7") device header pin assignments against actual hardware

## i2c_bus/

- [x] Shared I2C bus manager -- single init, port accessor, init guard
- [x] Device-conditional compilation -- stub for boards without I2C peripherals
- [x] Verified on Waveshare hardware (CST820, TCA9554, QMI8658 all respond)

## tca9554/

- [x] TCA9554 I2C GPIO expander driver -- init, set_pin, set_all, shadow register
- [x] Pin mapping: EXIO1=LCD reset, EXIO2=touch reset, EXIO3=LCD SPI CS, EXIO4=SD D3, EXIO8=buzzer
- [x] Stub implementation for boards without HAS_GPIO_EXPANDER
- [x] Verified on Waveshare hardware (LCD/touch reset, SPI CS sequencing)

## qmi8658/ -- 6-Axis IMU

- [x] QMI8658 I2C driver -- init, chip ID verify, accel/gyro config
- [x] Burst read API -- 14-byte read (temp + accel XYZ + gyro XYZ)
- [x] Sensitivity tables -- auto-convert raw to G / DPS based on configured range
- [x] Low-pass filter config -- accel LPF mode 0, gyro LPF mode 3
- [x] Power management -- power_down / wake API
- [x] Stub implementation for boards without HAS_IMU
- [x] Background task -- 50Hz polling, complementary filter (α=0.98), volatile cache
- [x] Boot calibration -- 2s gyro bias + Rodrigues' rotation matrix (arbitrary mount)
- [x] NVS persistence -- save/load calibration to `imu_cal` namespace
- [x] qmi8658_calibrate() -- force live recalibration (stop/restart task)
- [x] qmi8658_clear_calibration() -- erase NVS calibration data
- [x] Gyro range 512 DPS (was 64, caused clipping)
- [x] Axis mapping -- X=forward, Y=right, Z=up in calibrated reference frame
- [x] IMU data integration into data logger -- Lateral G + Longitudinal G columns appended to CSV
- [x] IMU data as virtual PID channels in gauge_engine -- VPID_IMU, gauge_engine_update reads qmi8658
- [ ] Orientation detection -- portrait/landscape auto-rotate (low priority, round display)

## pcf85063/ -- Real-Time Clock

- [x] PCF85063A I2C driver -- init, set_time, get_time, set_epoch, get_epoch
- [x] BCD encode/decode helpers for 7-register burst read/write
- [x] UTC storage design -- RTC stores UTC, timezone offset in NVS
- [x] `pcf85063_sync_to_system()` -- read RTC → `settimeofday()` on boot
- [x] `pcf85063_set_epoch()` -- set system clock + write RTC in one call
- [x] NVS timezone persistence -- namespace \"rtc\" key \"tz_off\" (minutes from UTC)
- [x] HAS_RTC guard -- stub implementations for boards without RTC
- [x] Boot integration -- Phase 0.7 in main.c (after IMU, before display)
- [x] HTTP time sync -- POST /api/settime + GET /api/time in time_handler.c
- [x] JS auto-sync -- portal grabs client epoch + timezone offset on page load
- [ ] Battery backup -- coin cell holds time across power cycles (hardware ordered)

## System Info Panel (Screen2)

- [x] System panel populated -- Display heap/min/uptime, SD/logger status, CAN Interface remote stats, UART stats
- [x] 1-second LVGL timer refresh
- [x] Remote CAN Interface data from heartbeat (heap, uptime, node state)

## imu_display/ -- IMU Visualization

- [x] Bubble dot -- maps ±1.5G lateral/longitudinal to panel edges, color-coded by G-force
- [x] Crosshair lines -- subtle center reference
- [x] G-force labels -- lateral G, longitudinal G at axis endpoints
- [x] 10Hz LVGL timer -- reads qmi8658_get_orientation() volatile cache
- [x] Separate component -- lives outside ui/ directory (SquareLine-safe)
- [x] HAS_IMU guard -- hides panel on boards without IMU
- [x] G-load mode -- bubble driven by accel not tilt (correct for on-road use)
- [x] Refactor to render inside gauge slot -- attach/detach API, inner container for dropdown avoidance
- [x] Tilt mode option -- unit dropdown toggles G-load vs tilt via imu_display_set_mode()
- [x] Dual mode rendering -- G-Load (±1.5G accel) and Tilt (±30° pitch/roll) in timer callback
- [ ] Configurable G range -- adjustable ±G mapping via IMU Setup tab (see wifi_manager Phase 7)
- [ ] Direction-of-travel calibration -- forward vector determination from sustained straight-line driving

## data_logger/ -- IMU Integration

- [x] Add Lateral G / Longitudinal G columns to CSV output (auto-detected via HAS_IMU)
- [x] IMU channels in log header (channel info section: PID 65280/65281, names, G units)

## Boot Splash

- [x] Load splash image from SD card /images/splash.bmp
- [x] Render splash on display during boot sequence (visible during comm_link/gauge init)
- [x] BMP decoder -- 16/24/32 bpp, bottom-up and top-down, row-by-row to PSRAM RGB565
- [x] Auto-cleanup -- boot_splash_hide() frees PSRAM after ui_init() takes over

## main/

- [x] `main.c` -- app_main entry point with boot sequence
- [x] `main.c` -- system_init -> display_init -> touch_init -> logger_init -> boot_splash -> comm_link -> gauge_engine -> ui_init -> boot_splash_hide -> ui_events_post_init -> theme_color_restore
- [x] `main.c` -- error handling for init failures (continue with degraded functionality)
- [x] `main.c` -- gauge_engine_init() wired as Phase 7 after comm_link_start()
- [x] `CMakeLists.txt` -- register all components
- [ ] `partitions.csv` -- configure for OTA support (needed when OTA is implemented)

## Integration and Testing

- [ ] Loopback test -- comm_link receives simulated UART messages, verify PID store updates
- [ ] Display test -- render all gauge types with known values, visual verification
- [ ] Touch test -- verify XPT2046 touch input and calibration accuracy
- [x] Logger test -- run logging session, verify CSV output matches HP Tuners import format
- [ ] WiFi test -- connect to AP, browse logs, download file, verify file integrity
- [ ] REST API test -- verify all endpoints return correct JSON for config read/write
- [ ] Custom PID test -- create custom PID via web UI, verify polling and logging
- [ ] Math channel test -- create virtual channel, verify formula output in gauges and log
- [x] Alert test -- verified on hardware: gradient warn border, flashing red critical border + red text, theme restore on clear
- [ ] Sensor test -- connect BME280, verify readings appear in data store and CSV
- [ ] Buzzer test -- trigger alert condition, verify tone output
- [x] Stress test -- sustained polling across 20 PIDs in round-robin, ~1.1 Hz per PID, verified zero dropped data, zero failures
- [x] Endurance test -- 10-hour overnight run: active CAN polling, UART streaming, LVGL rendering, SD logging. Zero heap delta, zero crashes, UI responsive throughout. (Feb 21, 2026)
- [x] Cross-node integration -- connect to CAN Interface Node via USB-C, live vehicle data

---

## Project Vision

This project will evolve into a **full driver display platform** for:
- **Retro-mods** -- modern digital gauges in classic cars
- **Race cars** -- lap data, G-forces, telemetry, alerts
- **Off-road rigs** -- inclinometer, tilt angles, trail data
- **Primary gauge sets** -- not just addon gauges, but complete instrument clusters

### Gauge Renderer System (WiFi Manager scope)

The current numeric-only gauge rendering will be replaced by a modular renderer
system supporting multiple visualization types per gauge slot:

- Sweep dials (analog-style arc gauges)
- Bar graphs (horizontal/vertical fill)
- Numeric readouts (current, with formatting options)
- IMU bubble / inclinometer
- Symbol indicators (CEL, turn signals, warning lights)
- True IMU 3D visualization
- Driver-focused composite displays

Renderers will be selectable per gauge slot and configurable via the web UI.
This requires the WiFi Manager infrastructure to be in place first.

### Virtual Channels & Hardware Integration

Beyond CAN PIDs, the gauge system will support:
- IMU data as selectable virtual PIDs (G-load, tilt, yaw)
- Environmental sensors (BME/BMP temp, humidity, pressure)
- Custom math channels (HP from torque+RPM, AFR from lambda, etc.)
- Output controls based on user-defined if/and/else logic
- External inputs from the CAN Interface Node hardware addons

### Design Principles

> Clean, modular code with an overkill of documentation is absolutely critical.

Every component must be:
- Self-contained with clear API boundaries
- Documented with README, inline comments, and type definitions
- Testable in isolation
- Prepared for display-agnostic rendering (480x272, 480x480, 800x480+)
