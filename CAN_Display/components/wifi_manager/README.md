# wifi_manager — WiFi AP, Config Hub & Log Download Server

## Purpose

The **only WiFi access point** in the CAN_ESPIDF system. Users connect from
a phone or laptop to browse logs, download CSV files, configure gauges,
adjust display settings, and manage CAN Interface Node settings (proxied
over the UART link). All settings are persisted to NVS.

When WiFi is activated, the LCD displays a **QR code** that the user scans
with their phone to auto-connect to the AP — no manual SSID/password typing.
After the client connects, a second QR code appears with the portal URL.
This two-stage QR flow is modeled on the proven ToolTruck OTA implementation
(`C:\Users\TEOLA\Documents\Projects\ToolTruckESPIDF\main\ota\`).

## Status: STUB — Implementation Phases Below

Only `wifi_manager_init()` and `wifi_manager_start()` exist as stubs.

---

## Architecture

```
 Phone / Laptop                          LCD Screen
      │                                 ┌─────────────────┐
      │                                 │  QR Code Panel   │
      │  1. Scan WiFi QR on LCD         │  ┌───────────┐  │
      │     "WIFI:T:WPA;S:..;P:..;;"    │  │ █▄▀█▀▄█▀█ │  │
      │  2. Phone auto-connects         │  │ ▄▀▄█▄▀█▄▀ │  │
      │  3. LCD shows URL QR            │  │ █▄▀█▀▄█▀█ │  │
      │  4. Scan URL QR → browser       │  └───────────┘  │
      │                                 │  SSID: CAN_abcd │
      │  WiFi (AP mode)                 │  Pass: Hk7mNp3Q │
      │                                 └─────────────────┘
      ▼
┌─────────────────────────────────────────────────────────────┐
│                     wifi_manager                            │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────┐    │
│  │  wifi_ap.c   │  │ http_server  │  │  REST API      │    │
│  │  AP init     │  │ route reg    │  │  handlers      │    │
│  │  DHCP        │  │ static HTML  │  │  JSON in/out   │    │
│  │  random pwd  │  └──────┬───────┘  └───────┬────────┘    │
│  │  client evts │         │                   │            │
│  └──────┬───────┘         │                   │            │
│         │    ┌────────────┼───────────────────┤            │
│         │    ▼            ▼                   ▼            │
│  ┌──────▼──────┐  ┌────────────┐  ┌────────────────────┐  │
│  │  qr_screen  │  │log_handler │  │  status_handler    │  │
│  │  WiFi QR    │  │ file list  │  │  system info       │  │
│  │  URL QR     │  │ download   │  │  comm_link stats   │  │
│  │  LVGL panel │  │ delete     │  │  CAN bus status    │  │
│  └─────────────┘  └─────┬──────┘  └────────┬───────────┘  │
│                          │                  │              │
│  ┌────────────┐          │                  │              │
│  │config_hdlr │          │                  │              │
│  │ gauge cfg  │          │                  │              │
│  │ display    │          │                  │              │
│  │ splash     │          │                  │              │
│  └─────┬──────┘          │                  │              │
│        │                 │                  │              │
└────────┼─────────────────┼──────────────────┼──────────────┘
         │                 │                  │
    ┌────▼────┐   ┌────────▼──────┐   ┌───────▼──────────┐
    │gauge_   │   │data_logger    │   │comm_link         │
    │engine   │   │(SD card)      │   │(UART to CAN      │
    │boot_    │   │               │   │ Interface Node)  │
    │splash   │   │               │   │                  │
    └─────────┘   └───────────────┘   └──────────────────┘
```

---

## QR Code System

### Overview

The QR code system eliminates manual SSID/password entry. LVGL's built-in
QR code widget (`LV_USE_QRCODE`) renders scannable codes directly on the LCD.

**Prerequisite**: Set `LV_USE_QRCODE 1` in `lv_conf.h` (currently disabled at line 660).

### Two-Stage QR Flow

Modeled on the ToolTruck OTA pattern (`ota.c` lines 700-847):

**Stage 1 — WiFi QR (shown immediately on AP start)**:
```
WIFI:T:WPA;S:<ssid>;P:<password>;;
```
This is the standard WiFi QR format recognized by all modern phones.
Scanning auto-joins the AP without any manual typing.

**Stage 2 — URL QR (shown after a client connects)**:
```
http://192.168.4.1/
```
After the phone connects to the AP, the LCD swaps to the URL QR code.
The user scans it and their browser opens the config portal.

### Random Password Generation

Each time the AP starts, a **fresh 8-character random password** is generated
using `esp_random()` (hardware RNG). The charset excludes visually ambiguous
characters (no `0/O/o`, `1/l/I`):

```c
static const char charset[] = "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghjkmnpqrstuvwxyz23456789";
```

This matches the ToolTruck OTA pattern exactly. The password is displayed
as plain text beneath the QR code so the user can type it manually if
QR scanning fails.

### QR Screen Layout (LVGL)

The QR screen is a dedicated LVGL screen (not a SquareLine screen) that
overlays the normal gauge UI when WiFi is active.

```
┌──────────────────────────────────┐
│          WiFi Setup              │  ← Title bar
│  ┌────────────────────────────┐  │
│  │                            │  │
│  │       ██ ▄▀█ ██ ▄▀        │  │
│  │       ▀▄ ██ ▀▄ ██         │  │  ← QR code (centered, ~200x200 px)
│  │       ██ ▄▀█ ██ ▄▀        │  │
│  │                            │  │
│  └────────────────────────────┘  │
│                                  │
│   Network: CAN_a7b3              │  ← SSID (with random suffix)
│   Password: Hk7mNp3Q            │  ← Password (plain text fallback)
│                                  │
│   Scan QR to connect             │  ← Instruction text
│           [Close]                │  ← Close button → stop AP, return to gauges
└──────────────────────────────────┘
```

After a client connects (`WIFI_EVENT_AP_STACONNECTED`), the screen updates:

```
┌──────────────────────────────────┐
│        Config Portal             │  ← Title changes
│  ┌────────────────────────────┐  │
│  │       ██ ▄▀█ ██ ▄▀        │  │
│  │       ▀▄ ██ ▀▄ ██         │  │  ← URL QR code
│  │       ██ ▄▀█ ██ ▄▀        │  │
│  └────────────────────────────┘  │
│                                  │
│   Connected: 1 client            │  ← Client count
│   http://192.168.4.1/            │  ← URL (plain text fallback)
│                                  │
│   Scan QR to open portal         │  ← Updated instruction
│           [Close]                │  ← Close button
└──────────────────────────────────┘
```

### WiFi Event Handling

The WiFi event handler tracks client connections exactly like ToolTruck OTA:

```c
// In wifi_ap.c event handler:
case WIFI_EVENT_AP_STACONNECTED:
    s_client_count++;
    // Trigger QR screen update: WiFi QR → URL QR
    break;

case WIFI_EVENT_AP_STADISCONNECTED:
    if (s_client_count > 0) s_client_count--;
    // If count drops to 0, revert to WiFi QR
    break;
```

### SSID Format

The AP SSID includes a short random suffix to avoid collisions when
multiple CAN_ESPIDF units are nearby:

```
CAN_<4-hex-chars>        e.g. "CAN_a7b3"
```

The suffix is derived from the ESP32 MAC address (last 2 bytes), so it
is consistent across reboots but unique per device. The SSID can be
overridden via NVS if the user sets a custom one through the web UI.

### Activation Flow (from main.c)

WiFi is **not started automatically on boot** by default. The user
activates it from the touch UI (a "WiFi" button or menu item):

1. User taps WiFi button on gauge screen
2. `wifi_manager_start()` called
3. AP starts with random password
4. QR screen rendered on LCD (replaces gauge screen)
5. User scans WiFi QR → phone connects
6. LCD swaps to URL QR
7. User scans URL QR → browser opens portal
8. User taps "Close" on LCD → `wifi_manager_stop()`, returns to gauges

If `ap_on` NVS key is set, the AP starts automatically in `wifi_manager_init()`
during boot (but the QR screen is not shown — the user must tap the WiFi
button to see it).

---

## Implementation Phases

### Phase 1: WiFi AP + QR Code + System Status
Get the AP running with QR code display and a basic JSON system status endpoint.

**Files**: `wifi_ap.c`, `qr_screen.c`, `http_server.c`, `status_handler.c`

| Task | API Used | Notes |
|------|----------|-------|
| WiFi AP init (APSTA mode) | `esp_wifi_set_mode(WIFI_MODE_APSTA)` | APSTA allows future STA coexistence |
| AP config | `esp_wifi_set_config(WIFI_IF_AP, ...)` | SSID from MAC suffix, random password |
| Random password | `esp_random()` modulo charset | 8 chars, unambiguous charset |
| DHCP server on 192.168.4.x | `esp_netif_create_default_wifi_ap()` | Automatic with SoftAP |
| WiFi event handler | `esp_event_handler_register(WIFI_EVENT, ...)` | Track client connect/disconnect |
| Enable LV_USE_QRCODE | Set `LV_USE_QRCODE 1` in `lv_conf.h` | One-time config change |
| QR screen (WiFi stage) | `lv_qrcode_create()`, `lv_qrcode_update()` | WIFI: URI format |
| QR screen (URL stage) | `lv_qrcode_update()` on client connect event | http://192.168.4.1/ |
| QR screen SSID/pass labels | `lv_label_create()` | Plain text fallback |
| Close button | `lv_btn_create()` + event handler | Calls `wifi_manager_stop()` |
| HTTP server start | `httpd_start()`, `httpd_register_uri_handler()` | ESP-IDF built-in |
| `GET /api/status` | system, comm_link, logger APIs | JSON response |
| `GET /` | — | Minimal HTML with status fetch |

**API endpoints (Phase 1)**:
```
GET  /              → HTML dashboard (embedded in firmware)
GET  /api/status    → { heap, uptime, link_state, link_stats, sd_mounted, logger_state }
```

**NVS keys** (namespace `"wifi"`):
| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `ssid` | string | `""` (empty = use MAC-derived) | AP SSID override |
| `pass` | string | `""` (empty = random each start) | AP password override |
| `channel` | u8 | `1` | WiFi channel |
| `ap_on` | u8 | `0` | Auto-start AP on boot (0=manual, 1=auto) |

**Password behavior**:
- If NVS `pass` is empty (default): generate random 8-char password each AP start
- If NVS `pass` is set by user via web UI: use that persistent password
- Password always displayed beneath QR code on LCD for manual entry fallback

---

### Phase 2: Log File Browser + Download
Browse and download CSV log files from the SD card.

**Files**: `log_handler.c`

| Task | API Used | Notes |
|------|----------|-------|
| List log files | `opendir()`, `readdir()`, `stat()` on `LOGGER_LOG_DIR` | Returns name, size, date |
| Download file | `fopen()` + chunked HTTP response | Stream large files without buffering entire file in RAM |
| Delete file | `unlink()` | With confirmation in frontend |
| SD stats | `logger_get_status()` → `sd_total_bytes`, `sd_free_bytes` | Show on dashboard |

**API endpoints (Phase 2)**:
```
GET  /api/logs              → [ { name, size, date }, ... ]
GET  /api/logs/:filename    → file download (Content-Disposition: attachment)
DELETE /api/logs/:filename  → delete file → { ok: true }
GET  /api/sd                → { total_bytes, free_bytes, mounted }
```

**Chunked download** — Required because log files can be multi-MB.
Read 4KB at a time via `httpd_resp_send_chunk()`, final empty chunk to close.

---

### Phase 3: Display Configuration API
Read/write gauge config, splash duration, backlight via REST.

**Files**: `config_handler.c`

| Task | API Used | Notes |
|------|----------|-------|
| Read gauge slots | `gauge_engine_get_slot(i)` for i=0..19 | Returns pid_id, units, value |
| Write gauge slot | `gauge_engine_set_pid()`, `gauge_engine_set_unit()` | Triggers NVS save |
| Clear gauge slot | `gauge_engine_clear_slot()` | |
| PID dropdown list | `gauge_engine_build_pid_options()` | Newline-separated string |
| Unit options | `gauge_engine_get_unit_options()` | Per-slot |
| Splash duration | `boot_splash_get_duration()` / `boot_splash_set_duration()` | ms value |
| Backlight | Direct LEDC duty write or new API | Needs display_driver addition |

**API endpoints (Phase 3)**:
```
GET  /api/gauges            → { slots: [ { pid_id, base_unit, display_unit, ... } ] }
POST /api/gauges/:slot      → { pid_index: N, unit_index: M } → set PID + unit
DELETE /api/gauges/:slot    → clear slot
GET  /api/gauges/pids       → { options: "RPM\nSpeed\n...\nIMU" }
GET  /api/gauges/:slot/units → { options: "km/h\nmph" }
GET  /api/config/splash     → { duration_ms: 3000 }
POST /api/config/splash     → { duration_ms: 5000 }
GET  /api/config/backlight  → { brightness: 70 }
POST /api/config/backlight  → { brightness: 50 }
```

---

### Phase 4: HTML Frontend (Embedded)
Single-page webapp served from firmware flash. No SPIFFS needed.

**Files**: `www/` directory with HTML/CSS/JS → compiled as C byte arrays.

| Page | Features |
|------|----------|
| Dashboard | System status, link state, SD usage, uptime |
| Logs | File list table, download/delete buttons |
| Gauges | Slot grid, PID dropdown, unit dropdown per slot |
| Settings | Splash duration slider, backlight slider, WiFi SSID/pass |

**Approach**: Embed as `const char index_html[] = "..."` in C source or use
cmake `target_add_binary_data()`. Frontend uses `fetch()` API for all REST
calls — no page reloads.

**Size estimate**: ~15-30KB compressed for a minimal but functional UI.

---

### Phase 5: CAN Interface Proxy
Relay commands to the CAN Interface Node over UART via comm_link.

**Files**: `can_proxy_handler.c`

| Task | API Used | Notes |
|------|----------|-------|
| Get CAN status | `comm_link_get_can_status()`, heartbeat data | CAN bus on/off/error |
| Trigger scan | `comm_link_request_scan()` | Async, poll scan_status |
| Get vehicle info | `comm_link_get_vehicle_info()` | VIN, ECU count, supported PIDs |
| Set poll list | `comm_link_set_poll_list()` | From web UI |
| Read DTCs | CONFIG_CMD `CMD_READ_DTCS` via comm_link | Future |
| Clear DTCs | CONFIG_CMD `CMD_CLEAR_DTCS` via comm_link | Future |

**API endpoints (Phase 5)**:
```
GET  /api/can/status        → { can_status, node_state, uptime, free_heap }
POST /api/can/scan          → trigger scan → { status: "in_progress" }
GET  /api/can/scan          → { status, vin, ecu_count, pid_count }
GET  /api/can/vehicle       → { vin, protocol, supported_pids: [...] }
POST /api/can/poll          → { pids: [0x010C, 0x010D], rate_hz: 10 }
DELETE /api/can/poll        → clear poll list
```

---

### Phase 6+ (Future / Deferred)

| Feature | Phase | Dependency |
|---------|-------|------------|
| DTC viewer / clear | 6 | `CMD_READ_DTCS`/`CMD_CLEAR_DTCS` in comm_protocol |
| Custom PID editor | 6 | New PID storage in NVS, formula engine |
| Custom math channels | 6 | Virtual PID extension (VPID 0xFF01+) |
| Freeze frame viewer | 6 | New comm_protocol message type |
| CAN bitrate change | 6 | New CONFIG_CMD + CAN Interface handler |
| Vehicle profiles | 7 | NVS profile storage keyed by VIN |
| Alert thresholds | 7 | gauge_engine threshold fields + UI |
| Display themes | 8 | LVGL style system, NVS-persisted |
| WebSocket live data | 8 | Alternative to polling /api/status |
| Hardware addon config | 8 | IMU cal, sensor offsets, buzzer/LED |

---

## File Structure

```
wifi_manager/
├── README.md               # This file
├── CMakeLists.txt          # Component build config
├── wifi_manager.h          # Public API (init, start, stop, QR data, credentials)
├── wifi_manager.c          # Top-level init, NVS load, state machine
├── wifi_ap.c               # WiFi SoftAP setup, DHCP, client event handlers
├── qr_screen.c             # LVGL QR code screen (WiFi stage + URL stage)
├── http_server.c           # httpd_start, route registration, static files
├── status_handler.c        # GET /api/status — system info JSON
├── log_handler.c           # GET/DELETE /api/logs — file list, download, delete
├── config_handler.c        # GET/POST /api/config/* — gauges, splash, backlight
├── can_proxy_handler.c     # GET/POST /api/can/* — CAN node proxy over UART
└── www/                    # HTML/CSS/JS source (compiled to C arrays)
    ├── index.html
    ├── style.css
    └── app.js
```

## Public API

```c
#include "wifi_manager.h"

// ──────────────────────────────────────────────────────────
// Lifecycle
// ──────────────────────────────────────────────────────────

/// Initialize WiFi subsystem, load NVS settings. Does NOT start AP
/// unless NVS "ap_on" is set.
esp_err_t wifi_manager_init(void);

/// Start WiFi AP with random password, start HTTP server,
/// show QR screen on LCD.
esp_err_t wifi_manager_start(void);

/// Stop HTTP server, stop WiFi AP, free QR screen, return to gauge UI.
esp_err_t wifi_manager_stop(void);

/// Check if AP is currently running.
bool wifi_manager_is_running(void);

// ──────────────────────────────────────────────────────────
// Client tracking (updated by WiFi event handler)
// ──────────────────────────────────────────────────────────

/// Number of clients currently connected to the AP.
int wifi_manager_get_client_count(void);

// ──────────────────────────────────────────────────────────
// Credentials (valid only while AP is running)
// ──────────────────────────────────────────────────────────

/// Current AP SSID (MAC-derived default or NVS override).
const char *wifi_manager_get_ssid(void);

/// Current AP password (random or NVS override).
const char *wifi_manager_get_password(void);

/// AP IP address string (always "192.168.4.1").
const char *wifi_manager_get_ip(void);

// ──────────────────────────────────────────────────────────
// QR code data (for display on LCD via qr_screen.c)
// ──────────────────────────────────────────────────────────

/// Fill buffer with WiFi QR URI: "WIFI:T:WPA;S:<ssid>;P:<pass>;;"
/// Returns ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer too small.
esp_err_t wifi_manager_get_wifi_qr_data(char *buffer, size_t buffer_size);

/// Fill buffer with URL QR data: "http://192.168.4.1/"
/// Returns ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer too small.
esp_err_t wifi_manager_get_url_qr_data(char *buffer, size_t buffer_size);

// ──────────────────────────────────────────────────────────
// Settings (NVS-backed, persist across reboots)
// ──────────────────────────────────────────────────────────

/// Override the default MAC-derived SSID. Empty string = revert to default.
esp_err_t wifi_manager_set_ssid(const char *ssid);

/// Override random password. Empty string = generate random each start.
esp_err_t wifi_manager_set_password(const char *pass);

/// Set WiFi channel (1-13).
esp_err_t wifi_manager_set_channel(uint8_t channel);

/// Enable/disable auto-start AP on boot.
esp_err_t wifi_manager_set_auto_start(bool enable);
```

## Dependencies

| Component | Purpose |
|-----------|---------|
| `esp_wifi` | SoftAP mode (APSTA) |
| `esp_http_server` | HTTP server, URI handlers |
| `esp_netif` | Network interface for AP |
| `esp_event` | WiFi event handler (client connect/disconnect) |
| `lvgl` | QR code widget (`lv_qrcode_create`) |
| `data_logger` | SD card file access, logger status |
| `gauge_engine` | Slot read/write, PID list, unit options |
| `comm_link` | UART link stats, vehicle info, scan, poll list |
| `boot_splash` | Splash duration get/set |
| `system` | Heap, uptime, NVS access |
| `devices` | `HAS_*` capability flags |
| `nvs_flash` | WiFi settings persistence |

## NVS Namespaces Used

| Namespace | Keys | Owner |
|-----------|------|-------|
| `wifi` | `ssid`, `pass`, `channel`, `ap_on` | wifi_manager |
| `display` | `splash_ms` | boot_splash (read by config_handler) |
| `gauge_cfg` | slot blob | gauge_engine (read/write by config_handler) |
| `logger` | `filenum` | data_logger (read by log_handler) |

## Memory Budget

| Resource | Budget | Notes |
|----------|--------|-------|
| WiFi stack | ~40KB | ESP-IDF WiFi driver |
| HTTP server | ~4KB + 1KB/connection | Max 4 concurrent connections |
| Embedded HTML | ~15-30KB | Compressed in flash, served directly |
| QR code widget | ~2KB | LVGL QR code canvas |
| JSON response buffers | ~2KB | Stack-allocated, largest is gauge config |
| **Total** | ~60-80KB RAM | Fits comfortably with 8MB PSRAM |

## Configuration Defaults

| Setting | Default | Range | Notes |
|---------|---------|-------|-------|
| SSID | `CAN_<MAC4>` | 1-32 chars | MAC-derived, overridable via NVS |
| Password | Random 8-char | 8-63 chars | Regenerated each AP start unless NVS override |
| Channel | 1 | 1-13 | |
| Auto-start | No | On/off | Off by default, user taps WiFi button |
| Max clients | 4 | 1-8 | |
| HTTP port | 80 | — | |
| AP IP | 192.168.4.1 | — | ESP-IDF default for SoftAP |

## Reference Implementation

The QR code and WiFi AP patterns are adapted from the ToolTruck ESP-IDF project:
- **Source**: `C:\Users\TEOLA\Documents\Projects\ToolTruckESPIDF\main\ota\ota.c`
- **Key functions**: `generate_random_password()`, `ota_get_wifi_qr_data()`, `ota_get_url_qr_data()`
- **Patterns adopted**: Random password charset, WIFI: URI format, two-stage QR, APSTA mode, client tracking via WiFi events, embedded HTML template
