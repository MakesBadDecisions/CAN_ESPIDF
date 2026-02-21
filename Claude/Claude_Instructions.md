# Claude AI Agent Instructions 
You are an embedded firmware engineer. That Always produces clean, modular, and maintainable code.
Basic Rules:
✅ Add enough debug logging to aid in proper diagnotisics of issues
✅ Dont allow files to get too large.
✅ Remove dead/unnecessary code as needed.
✅ Refactor for clarity and long-term maintainability.
✅ Follow existing project style unless change is justified and discussed properly.
✅ Create and maintain a main project README.md and TODO.md with all significant changes.
✅ Create and maintain README.md and TODO.md for all sub module folders, these show the what/how/where for each module as they are built and/or used/modified.
✅ The main README and TODO are for project scope docmentation while the sub module README and TODO are for keeping detailed documentation for each module for reuse or modification.
Workflow:
You dont build, upload or monitor. I do that. We dont waste your context by you watching those processes. 
Understand context first. If not 95% sure, ask clarifying questions.
Complete the task cleanly and efficiently using best practices.
Document changes in README.md (if relevant) and update TODO.md.
Goal:
Deliver robust, production-quality code with clear reasoning and minimal clutter.

---

# Session Handoff — IMU Integration (ALL COMPLETE)

## Completed Work

### 1. Virtual PID Infrastructure in gauge_engine (DONE)
- `VPID_BASE (0xFF00)`, `VPID_IMU (0xFF00)`, `GAUGE_IS_VIRTUAL()` macro, `imu_display_mode_t` enum
- 7 functions modified: set_pid, set_unit, build_pid_options, get_unit_options, update, rebuild_poll_list, load_config
- NVS persistence, uniqueness enforcement, active count skips virtual PIDs

### 2. imu_display — Gauge Slot Rendering (DONE)
- Attach/detach API, inner container (center 60%), dual mode (G-Load ±1.5G / Tilt ±30°)
- Clean crosshair layout: axis-end labels only (lateral G right of H-axis, longitudinal G top of V-axis)
- 10Hz LVGL timer reads qmi8658_get_orientation() volatile cache

### 3. ui_events.c — IMU Wiring (DONE)
- `gauge_widget_t` includes `gauge_panel` pointer
- on_pid_changed: IMU attach/detach with uniqueness enforcement
- on_unit_changed: mode switching via imu_display_set_mode()
- NVS restore handles VPID_IMU slots
- gauge_update_cb skips virtual PIDs (imu_display has own timer)

### 4. Data Logger — IMU G-load Columns (DONE)
- `data_logger.c`: Auto-detects IMU via `HAS_IMU` compile flag → `s_ctx.log_imu = true`
- `write_header()`: Appends 2 IMU columns to HP Tuners CSV channel info rows
  - PID numbers: 65280 (Lateral G), 65281 (Longitudinal G)
  - Names: "Lateral G", "Longitudinal G" — Units: "G", "G"
- `logger_log_row()`: Appends `orient.accel_lat` and `orient.accel_lon` after CAN PID values
- CMakeLists: Added `qmi8658` to REQUIRES
- No API change — logger_start() still takes CAN PIDs only, IMU columns auto-appended

## Architecture Notes
- Virtual PID IDs: 0xFF00+ reserved. VPID_IMU = 0xFF00. Future virtual channels (GPS, etc.) get 0xFF01+
- Display mode stored in `display_unit` field as cast int (0=G-Load, 1=Tilt) for VPID_IMU only
- Only ONE gauge slot can host IMU (single sensor). UI enforces uniqueness.
- Data logger IMU columns are independent of gauge slot IMU — always appended when HAS_IMU defined
- CAN channel count in UI log message reflects CAN PIDs only (IMU channels are silent additions)

## SquareLine Studio Notes
- `ui_gyroPanel1` deleted from SquareLine — no longer referenced in code
- After any SquareLine re-export: fix `ui_events.h` (re-add `#include "lvgl.h"` and `void ui_events_post_init(void);`)

## What's Next (from TODO.md)
1. **WiFi Manager** — Phase 1: WiFi AP + QR code + system status (see `wifi_manager/README.md`)
2. **Custom gauge types** — sweep dials, bar graphs, arc gauges
3. **File rotation** — close + new log file at configurable size limit

## WiFi Manager Design (DOCUMENTED — NOT YET IMPLEMENTED)

### QR Code System (adapted from ToolTruck OTA)
- Two-stage QR flow: WiFi QR on LCD → URL QR after client connects
- `LV_USE_QRCODE` must be enabled in `lv_conf.h` (line 660, currently `0`)
- WiFi QR format: `WIFI:T:WPA;S:<ssid>;P:<password>;;` — phones auto-connect
- URL QR format: `http://192.168.4.1/` — shown after `WIFI_EVENT_AP_STACONNECTED`
- Random 8-char password via `esp_random()`, unambiguous charset (no 0/O/o, 1/l/I)
- SSID: `CAN_<MAC4>` by default (last 2 bytes of MAC), overridable via NVS
- APSTA mode for future STA coexistence
- Reference source: `C:\Users\TEOLA\Documents\Projects\ToolTruckESPIDF\main\ota\ota.c`

### New Files for Phase 1
- `wifi_ap.c` — SoftAP init, DHCP, WiFi event handler, random password
- `qr_screen.c` — LVGL QR code screen (two stages), close button
- `http_server.c` — httpd route registration
- `status_handler.c` — GET /api/status JSON endpoint

### Full API (wifi_manager.h — updated)
- Lifecycle: `init()`, `start()`, `stop()`, `is_running()`
- Credentials: `get_ssid()`, `get_password()`, `get_ip()`
- QR data: `get_wifi_qr_data(buf, size)`, `get_url_qr_data(buf, size)`
- Settings: `set_ssid()`, `set_password()`, `set_channel()`, `set_auto_start()`
- Client tracking: `get_client_count()`

## Recent: Boot Splash from SD Card (DONE)

### boot_splash Component
- `boot_splash.h/c`: BMP decoder + LVGL screen, loads `/sdcard/images/splash.bmp`
- Supports 16-bit (RGB565), 24-bit (BGR888), 32-bit (BGRA8888) uncompressed BMP
- Row-by-row decode into PSRAM RGB565 buffer, centered on black LVGL screen
- `boot_splash_show()`: Read+decode+display — non-fatal on missing file (ESP_ERR_NOT_FOUND)
- `boot_splash_hide()`: Delete LVGL screen + free PSRAM buffer

### Boot Sequence Reorganized (main.c)
- **Moved logger_init() up** to Phase 3 (after touch_init, before UI) — SD must be mounted for splash
- Phase 3.5: `boot_splash_show()` — splash visible during phases 4-6
- Phase 7: `ui_init()` loads main UI, replacing splash screen
- Phase 7.5: `boot_splash_hide()` frees splash memory
- Touch calibration (Phase 4) appears over splash on first boot