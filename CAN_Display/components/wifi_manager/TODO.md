# wifi_manager — Task List

Status key: `[ ]` not started, `[-]` in progress, `[x]` done.

---

## Phase 1: WiFi AP + HTTP Skeleton + System Status

### WiFi SoftAP (`wifi_ap.c`)
- [ ] `esp_netif_create_default_wifi_ap()` — create AP network interface
- [ ] `esp_wifi_init()` with default config
- [ ] `esp_wifi_set_mode(WIFI_MODE_AP)` — AP only, no station
- [ ] `wifi_config_t` — SSID, password, channel, max_connection from NVS or defaults
- [ ] `esp_wifi_set_config()` + `esp_wifi_start()`
- [ ] WiFi event handler — `WIFI_EVENT_AP_STACONNECTED` / `WIFI_EVENT_AP_STADISCONNECTED` logging
- [ ] Client count tracking via event handler
- [ ] NVS read for SSID/password/channel on init, fallback to defaults
- [ ] NVS write functions for SSID/password/channel (called from config_handler)

### HTTP Server (`http_server.c`)
- [ ] `httpd_start()` with config (port 80, max_uri_handlers=16, stack_size=8192)
- [ ] Route registration function — calls each handler module's register function
- [ ] `GET /` — serve embedded HTML dashboard (minimal: shows status, links to other pages)
- [ ] CORS headers helper (Access-Control-Allow-Origin: * for dev convenience)
- [ ] JSON response helper — `send_json(req, json_string)` with Content-Type header
- [ ] Error response helper — `send_error(req, code, message)` as JSON

### System Status (`status_handler.c`)
- [ ] `GET /api/status` — JSON response:
  ```json
  {
    "heap_free": 7001684,
    "uptime_ms": 12345,
    "link_state": "connected",
    "link_stats": { "rx_frames": 100, "tx_frames": 50, "rx_errors": 0 },
    "can_status": "bus_on",
    "sd_mounted": true,
    "logger_state": "ready",
    "logger_rows": 0,
    "sd_total_mb": 3796,
    "sd_free_mb": 3400,
    "wifi_clients": 1
  }
  ```
- [ ] API calls: `sys_get_free_heap()`, `sys_time_ms()`, `comm_link_get_state()`, `comm_link_get_stats()`, `comm_link_get_can_status()`, `logger_get_status()`, client count

### wifi_manager.c (Top-Level)
- [ ] `wifi_manager_init()` — call `wifi_ap_init()`, log AP SSID and IP
- [ ] `wifi_manager_start()` — call `http_server_start()`, register all route handlers
- [ ] `wifi_manager_stop()` — `httpd_stop()`, `esp_wifi_stop()`
- [ ] `wifi_manager_is_running()` — return AP + httpd state
- [ ] `wifi_manager_get_client_count()` — return tracked count

### wifi_manager.h (API Update)
- [ ] Add `wifi_manager_stop()`
- [ ] Add `wifi_manager_set_ssid()`, `wifi_manager_set_password()`, `wifi_manager_get_ssid()`
- [ ] Add `wifi_manager_is_running()`, `wifi_manager_get_client_count()`

### main.c Integration
- [ ] Uncomment / add `wifi_manager_init()` + `wifi_manager_start()` in boot sequence
- [ ] Place after `ui_events_post_init()` (Phase 9 in boot)

### CMakeLists.txt Update
- [ ] Add new source files to SRCS
- [ ] Add `comm_link`, `boot_splash`, `devices`, `nvs_flash`, `esp_netif` to REQUIRES

### Testing
- [ ] AP visible on phone/laptop WiFi scan
- [ ] Connect to AP, get DHCP address (192.168.4.x)
- [ ] `curl http://192.168.4.1/api/status` returns valid JSON
- [ ] `http://192.168.4.1/` shows basic HTML page
- [ ] Client connect/disconnect logged in serial monitor

---

## Phase 2: Log File Browser + Download

### Log Handler (`log_handler.c`)
- [ ] `GET /api/logs` — JSON array of log files:
  ```json
  [
    { "name": "log1.csv", "size": 12345, "date": "2026-02-20" },
    { "name": "log18.csv", "size": 45678, "date": "2026-02-20" }
  ]
  ```
  Uses `opendir(LOGGER_LOG_DIR)`, `readdir()`, `stat()` for each file
- [ ] `GET /api/logs/<filename>` — chunked file download
  - Set `Content-Type: text/csv`
  - Set `Content-Disposition: attachment; filename="log1.csv"`
  - Stream via `httpd_resp_send_chunk()`, 4KB read buffer
  - Validate filename (no path traversal: reject `..`, `/`)
- [ ] `DELETE /api/logs/<filename>` — delete file via `unlink()`
  - Validate filename, return error if file doesn't exist
  - Return `{ "ok": true, "deleted": "log1.csv" }`
- [ ] `GET /api/sd` — SD card stats:
  ```json
  { "mounted": true, "total_mb": 3796, "free_mb": 3400, "log_count": 18 }
  ```
- [ ] Path sanitization helper — reject filenames with `..`, leading `/`, non-alphanumeric (except `.` and `_`)

### Testing
- [ ] `/api/logs` lists all CSV files with correct sizes
- [ ] Download a log file via browser — opens in HP Tuners or spreadsheet
- [ ] Delete a log file — verify gone from list
- [ ] `/api/sd` shows correct free space

---

## Phase 3: Display Configuration API

### Config Handler (`config_handler.c`)
- [ ] `GET /api/gauges` — all 20 slots:
  ```json
  {
    "slots": [
      { "slot": 0, "pid_id": 268, "pid_name": "RPM", "base_unit": "RPM", "display_unit": "RPM", "value": "3500", "valid": true },
      { "slot": 1, "pid_id": 65280, "pid_name": "IMU", "base_unit": "G-Load", "display_unit": "G-Load", "value": "", "valid": true },
      { "slot": 2, "pid_id": 65535, "pid_name": null, "base_unit": null, "display_unit": null, "value": null, "valid": false }
    ]
  }
  ```
  Uses `gauge_engine_get_slot()`, `comm_link_get_pid_name()`, `pid_unit_str()`
- [ ] `POST /api/gauges/:slot` — set PID and/or unit:
  - Body: `{ "pid_index": 5 }` or `{ "unit_index": 1 }` or both
  - Calls `gauge_engine_set_pid()` and/or `gauge_engine_set_unit()`
  - NOTE: LVGL dropdown state in ui_events.c must also update — may need callback or flag
- [ ] `DELETE /api/gauges/:slot` — clear slot via `gauge_engine_clear_slot()`
- [ ] `GET /api/gauges/pids` — available PIDs:
  ```json
  { "options": ["Engine coolant temperature", "Throttle position", "...", "IMU"], "count": 42 }
  ```
  Uses `gauge_engine_build_pid_options()`, split on `\n`
- [ ] `GET /api/gauges/:slot/units` — unit options for slot's current PID:
  ```json
  { "options": ["°C", "°F"], "count": 2 }
  ```
  Uses `gauge_engine_get_unit_options()`
- [ ] `GET /api/config/splash` → `{ "duration_ms": 3000 }`
  Uses `boot_splash_get_duration()`
- [ ] `POST /api/config/splash` — body: `{ "duration_ms": 5000 }`
  Calls `boot_splash_set_duration()`, validate range 0-10000
- [ ] `GET /api/config/backlight` → `{ "brightness": 70 }`
  Needs new display_driver API: `display_get_brightness()`
- [ ] `POST /api/config/backlight` — body: `{ "brightness": 50 }`
  Needs new display_driver API: `display_set_brightness(uint8_t pct)`

### display_driver Changes (for backlight API)
- [ ] Add `display_set_brightness(uint8_t percent)` — LEDC duty write
- [ ] Add `display_get_brightness()` — return current value
- [ ] NVS persistence for brightness (namespace `"display"`, key `"bright"`)

### Gauge Sync Issue (Important Design Decision)
When WiFi Manager changes a gauge slot's PID via REST, the LVGL dropdowns
in `ui_events.c` are out of sync. Options:
- **Option A**: WiFi writes NVS only → changes take effect on next boot
- **Option B**: WiFi sets a "config dirty" flag → LVGL timer checks and reloads
- **Option C**: WiFi calls gauge_engine functions + triggers LVGL dropdown update via thread-safe queue

Recommend **Option A** for Phase 3 (simplest), upgrade to **B** later.

### Testing
- [ ] `/api/gauges` shows all 20 slots with correct data
- [ ] POST to change a slot PID — verify NVS updated, reflected after reboot
- [ ] `/api/config/splash` round-trip: read → write → read confirms change
- [ ] `/api/config/backlight` changes brightness visibly

---

## Phase 4: HTML Frontend (Embedded)

### Web Pages
- [ ] `index.html` — single-page app with tabbed navigation (Dashboard | Logs | Gauges | Settings)
- [ ] `style.css` — minimal responsive CSS (works on phone and laptop)
- [ ] `app.js` — fetch()-based REST client, DOM updates, no framework

### Dashboard Tab
- [ ] System status cards: heap, uptime, link state, CAN status
- [ ] SD card usage bar (used/total)
- [ ] Logger state indicator (idle/logging)
- [ ] WiFi client count
- [ ] Auto-refresh every 5 seconds

### Logs Tab
- [ ] File list table: name, size (human-readable), date
- [ ] Download button per file → triggers browser download
- [ ] Delete button per file → confirm dialog → DELETE call → refresh list
- [ ] SD usage summary at top

### Gauges Tab
- [ ] 4-slot grid (matching physical display)
- [ ] PID dropdown per slot (populated from /api/gauges/pids)
- [ ] Unit dropdown per slot (populated from /api/gauges/:slot/units)
- [ ] "Changes apply on reboot" notice (Phase 3 Option A)

### Settings Tab
- [ ] Splash duration number input (ms) + Save button
- [ ] Backlight brightness slider + Save button
- [ ] WiFi SSID text input + Save button (requires reconnect)
- [ ] WiFi password text input + Save button
- [ ] Device info (read-only): board, flash size, PSRAM, firmware version

### Build Integration
- [ ] HTML/CSS/JS → C byte arrays (cmake `target_add_binary_data()` or `xxd`)
- [ ] Serve with correct Content-Type headers (text/html, text/css, application/javascript)
- [ ] Gzip compression of static files (optional, saves flash, browser decompresses)

### Testing
- [ ] All four tabs navigate correctly
- [ ] Dashboard shows live data, auto-refreshes
- [ ] Logs tab downloads and deletes files
- [ ] Gauges tab shows current config, saves changes
- [ ] Settings tab persists all values
- [ ] Works on iPhone Safari, Android Chrome, Desktop Chrome/Firefox

---

## Phase 5: CAN Interface Proxy

### CAN Proxy Handler (`can_proxy_handler.c`)
- [ ] `GET /api/can/status` — CAN node heartbeat data:
  ```json
  { "node_state": "streaming", "can_status": "bus_on", "free_heap_kb": 200, "uptime_ms": 123456 }
  ```
  Uses `comm_link_get_can_status()`, heartbeat cache
- [ ] `POST /api/can/scan` — trigger vehicle scan:
  - Calls `comm_link_request_scan()` with callback
  - Returns `{ "status": "in_progress" }` immediately
- [ ] `GET /api/can/scan` — scan status:
  ```json
  { "status": "complete", "vin": "1FTLR4FEXBPA98994", "ecu_count": 2, "pid_count": 41 }
  ```
  Uses `comm_link_get_scan_status()`, `comm_link_get_vehicle_info()`
- [ ] `GET /api/can/vehicle` — vehicle info:
  ```json
  { "vin": "1FTLR4FEXBPA98994", "protocol": 6, "ecu_count": 2, "supported_pids": [268, 269, ...] }
  ```
  Uses `comm_link_get_vehicle_info()`, `comm_link_get_supported_pids()`
- [ ] `POST /api/can/poll` — set poll list:
  - Body: `{ "pids": [268, 269, 271], "rate_hz": 10 }`
  - Calls `comm_link_set_poll_list()`
- [ ] `DELETE /api/can/poll` — clear poll list:
  - Calls `comm_link_clear_poll_list()`

### Testing
- [ ] `/api/can/status` returns CAN Interface Node heartbeat data
- [ ] POST scan → poll status → verify complete with VIN
- [ ] POST poll list → verify CAN Interface starts polling
- [ ] DELETE poll → verify CAN Interface stops

---

## Phase 6+: Future (Not Detailed Yet)

- [ ] DTC viewer — `CMD_READ_DTCS` → display codes with descriptions
- [ ] DTC clear — `CMD_CLEAR_DTCS` with confirmation
- [ ] Custom PID editor — NVS storage, formula parser
- [ ] Custom math channels — VPID 0xFF01+ with formula engine
- [ ] CAN bitrate change — new CONFIG_CMD type
- [ ] Freeze frame viewer — new comm_protocol message
- [ ] Vehicle profiles — save/load by VIN
- [ ] Alert thresholds — gauge_engine threshold fields
- [ ] Display themes — LVGL styles, NVS-persisted
- [ ] WebSocket live data — real-time PID values push
- [ ] Hardware addon config — IMU cal reset, buzzer, LED patterns
