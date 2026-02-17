# Display Node - Task List

Status key: `[ ]` not started, `[-]` in progress, `[x]` done.

## User Workflow (Connect → Configure → Monitor)

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
- [ ] Implement `connectCAN()` -- trigger vehicle scan via comm_link
- [ ] Populate PID dropdown from supported PIDs list
- [ ] Populate unit dropdown based on selected PID
- [ ] Handle PID selection change -- bind gauge to PID
- [ ] Handle unit selection change -- convert and display in selected unit
- [ ] Update `ui_gaugeText1` from comm_link PID store (periodic refresh)
- [ ] Add status label for connection state (DISCONNECTED/CONNECTING/CONNECTED)
- [ ] Add VIN display area for vehicle identification
- [ ] Add multi-gauge layout screen with 2-6 configurable gauges
- [ ] Configure lv_conf.h for ESP32-S3 with PSRAM framebuffer

## system/

- [ ] Logging wrapper -- thin layer over `esp_log` with project-wide tag conventions
- [ ] Timing utilities -- `millis()` / `micros()` equivalents using `esp_timer`
- [ ] NVS config module -- load/save gauge layout, alert thresholds, WiFi credentials
- [ ] NVS config module -- define default config values for first boot
- [ ] Task management helpers -- task creation wrapper with core affinity, priority, stack size
- [ ] Task watchdog registration for all tasks
- [ ] System health struct -- heap free, min heap, uptime, connection state
- [ ] Kconfig entries for stack sizes, task priorities, timer periods
- [ ] `sdkconfig.defaults` -- target esp32s3, partition table, flash size, PSRAM

## comm_link/

- [x] UART initialization -- configure UART RX/TX pins (GPIO17/18), 2Mbps baud
- [x] RX task -- receive UART bytes, scan for frame start byte (0xAA)
- [x] Frame parser -- validate CRC16, extract header and payload
- [x] Message dispatcher -- route parsed messages by type using shared/comm_protocol definitions
- [x] PID data store -- fixed-size array (64 entries), holds latest value + timestamp
- [x] PID data store -- mutex protection for cross-core access
- [ ] PID data store -- stale detection (mark entries not updated within timeout)
- [x] Heartbeat send -- TX heartbeat every 500ms
- [x] Heartbeat monitor -- track last RX timestamp, set disconnected flag after 2s timeout
- [x] Connection status API -- `comm_link_get_state()`, `comm_link_get_stats()`
- [x] TX path -- send frames to CAN Interface Node over UART
- [ ] Send vehicle scan request -- MSG_CONFIG_CMD to trigger ECU/VIN/PID scan
- [ ] Handle scan response -- MSG_SCAN_STATUS with ECU IDs, VIN, supported PIDs
- [ ] Send poll list update -- MSG_CONFIG_CMD with selected PID list
- [ ] Store vehicle info -- save VIN/ECU to NVS on successful scan
- [ ] Load vehicle info -- recall saved VIN/ECU from NVS on boot
- [ ] Callback registration -- notify UI when new PID data arrives

## display_driver/

- [ ] Display HAL interface -- `display_init()`, `display_flush()`, `display_set_backlight()`
- [ ] HAL interface -- `display_get_width()`, `display_get_height()`, `display_get_buffer()`
- [ ] Framebuffer allocation -- double buffer in PSRAM (800x480 RGB565 = ~750KB per buffer)
- [ ] RGB parallel LCD driver -- configure ESP-IDF RGB panel interface using device header pin definitions
- [ ] Elecrow CrowPanel 5" driver (DIS07050) -- init with correct timing parameters from device header
- [ ] Elecrow CrowPanel 7" driver (DIS08070H) -- init with correct timing parameters from device header
- [ ] GT911 touch controller driver -- I2C init, read touch points, report to gauge engine
- [ ] Backlight PWM control -- LEDC peripheral for brightness adjustment
- [ ] Device selection -- compile-time include of correct device header
- [ ] Color format abstraction -- RGB565 assumed, but keep format configurable
- [ ] Boot splash screen -- render CAN_ESPIDF logo / text during initialization

## gauge_engine/

- [ ] Gauge type definitions -- numeric, horizontal bar, vertical bar, sweep (radial dial)
- [ ] Gauge struct -- PID binding, position, size, min/max range, label, unit string
- [ ] Layout struct -- array of gauge definitions, screen arrangement, background color
- [ ] Layout storage -- save/load layouts to/from NVS
- [ ] Numeric gauge renderer -- large value text, label, unit, optional min/max indicators
- [ ] Bar gauge renderer -- filled rectangle proportional to value, tick marks, color gradient
- [ ] Sweep gauge renderer -- arc/dial with needle, tick marks, value label
- [ ] Value smoothing -- low-pass filter or exponential moving average for jitter reduction
- [ ] Alert threshold evaluation -- compare current value against WARNING and CRITICAL limits
- [ ] Alert rendering -- color changes (normal/yellow/red), border flash, full-screen overlay
- [ ] Alert audio feedback -- I2S audio or GPIO buzzer on CRITICAL alerts
- [ ] Layout manager -- switch between layouts (touch input or auto based on driving mode)
- [ ] Render pipeline -- clear framebuffer, draw all gauges in layout, push to display
- [ ] Font rendering -- embedded bitmap font or anti-aliased font for value/label text
- [ ] Stale value indication -- dim or gray out gauges whose PID data has gone stale

## data_logger/

- [ ] SD card initialization -- SPI bus config using device header SD pins, mount FAT filesystem
- [ ] SD card detection -- card detect GPIO or mount/unmount retry logic
- [ ] Session management -- start session (create new CSV), stop session (flush and close)
- [ ] Session auto-naming -- `LOG_YYYYMMDD_HHMMSS_NNN.csv` format (RTC or uptime-based)
- [ ] CSV header generation -- build header row from list of actively-polled PIDs
- [ ] CSV row writing -- snapshot PID data store, format values, write row with timestamp
- [ ] Write buffering -- RAM ring buffer, flush to SD every 50ms or on threshold
- [ ] Double buffering -- fill one buffer while writing the other to avoid blocking
- [ ] File rotation -- close current file and start new one at configurable size limit
- [ ] Storage monitoring -- track SD card free space, warn when low, stop logging when full
- [ ] Graceful SD removal -- detect unmount, buffer remaining data, resume on reinsert
- [ ] Session metadata -- write vehicle info (VIN, ECU) at top of file or in companion file
- [ ] Log trigger modes -- manual start/stop, auto-start on first PID data, ignition-based

## wifi_manager/

- [ ] WiFi AP initialization -- configure SSID, password, channel, max connections
- [ ] AP on-demand start -- enable AP via touch button or NVS setting, not always-on at boot
- [ ] HTTP server -- lightweight httpd for file serving and API endpoints
- [ ] Log file browser -- HTML page listing all CSV files on SD card with size and date
- [ ] Log file download -- serve CSV files for direct download via browser
- [ ] Log file deletion -- delete individual files or clear all logs via web UI
- [ ] Gauge config UI -- HTML page to select layout, assign PIDs to gauges, set ranges
- [ ] Alert config UI -- HTML page to set warning/critical thresholds per PID
- [ ] CAN node config proxy -- relay PID poll list, baud rate, scan commands to CAN node over UART
- [ ] CAN node status page -- show CAN bus state, error counts, supported PIDs (proxied over UART)
- [ ] Config save endpoint -- POST handler to save gauge/alert config to NVS
- [ ] System status page -- show connection state, heap, uptime, SD usage, active PIDs
- [ ] mDNS -- advertise `can-espidf.local` for easy browser access
- [ ] OTA update endpoint -- firmware upload via web browser (future)
- [ ] WiFi AP auto-disable -- shut down AP after configurable idle timeout to save power

## devices/

- [x] Create DIS06043H (4.3") device header with pin assignments (UART1 on GPIO17/18)
- [x] Create device.h selector header for build-time device selection
- [x] Create custom board JSON for CrowPanel 4.3" (4MB Flash, 2MB Quad PSRAM)
- [ ] Verify DIS07050 (5") device header pin assignments against actual hardware
- [ ] Verify DIS08070H (7") device header pin assignments against actual hardware
- [ ] Create device_test utility for display panel verification

## main/

- [ ] `main.c` -- app_main entry point, call init functions in boot sequence order
- [ ] `main.c` -- create all FreeRTOS tasks with correct core, priority, stack size
- [ ] `main.c` -- error handling for init failures (continue with degraded functionality)
- [ ] `CMakeLists.txt` -- register all components, link shared components from `../shared/`
- [ ] `partitions.csv` -- partition table (app, NVS, optional OTA partitions)

## Integration and Testing

- [ ] Loopback test -- comm_link receives simulated UART messages, verify PID store updates
- [ ] Display test -- render all gauge types with known values on CrowPanel, visual verification
- [ ] Touch test -- verify GT911 touch input registers correctly on both panel sizes
- [ ] Logger test -- run logging session, verify CSV output matches HPTuners import format
- [ ] WiFi test -- connect to AP, browse logs, download file, verify file integrity
- [ ] Alert test -- inject out-of-range values, verify warning and critical alert behavior
- [ ] Stress test -- sustained 20Hz PID updates across 30+ PIDs, verify no dropped data
- [ ] Endurance test -- 1-hour continuous logging session, verify no file corruption or heap leak
- [ ] Cross-node integration -- connect to CAN Interface Node via USB-C, live vehicle data end-to-end
