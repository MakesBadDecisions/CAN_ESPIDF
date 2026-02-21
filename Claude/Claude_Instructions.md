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

# Session Handoff — Settings Screen, Brightness, & Theme Color (ALL COMPLETE)

## Completed Work (This Session)

### 1. Settings Screen (Screen2) from SquareLine Studio (DONE)
- SquareLine Studio Screen2 exported with: brightness slider, back button, WiFi settings button, system panel, color wheel panel, settings button on Screen1
- **SquareLine export damage fixed**: `ui_events.h` re-added `#include "lvgl.h"`, `settingsButton()`, `ui_events_post_init()` declarations; `ui/CMakeLists.txt` restored to `idf_component_register()` format with `ui_Screen2.c` added

### 2. Brightness Slider + display_set_brightness API (DONE)
- `display_driver.h`: Added `display_set_brightness(uint8_t percent)` and `display_get_brightness()` API
- `display_driver.c` (Waveshare): GPIO config → LEDC timer → LEDC channel → `ledc_fade_func_install(0)` → NVS load → `display_set_brightness()`. Matches Waveshare demo `ST7701S.c` exactly.
- `display_driver.c` (CrowPanel): GPIO on/off (percent > 0 = on, 0 = off)
- Duty formula: `duty = 8191 - 81 * (100 - percent)`, special case: `percent == 0 → duty = 0`
- NVS persistence: namespace `"display"`, key `"bl_pct"`, restored on boot
- `ui_events.c`: `on_brightness_changed()` reads slider value → `display_set_brightness()` → updates label

### 3. SquareLine Screen Transition Fix (DONE)
- **Root cause**: `ui_event_settingsButton` (SquareLine-generated) calls `_ui_screen_change()` but never calls `settingsButton()`. Screen change destroys/recreates Screen2 widgets.
- **Fix 1**: Register `settingsButton` as additional `LV_EVENT_CLICKED` callback on `ui_settingsButton` in `ui_events_post_init()` — SquareLine-safe (never edits generated files)
- **Fix 2**: `settingsButton()` uses deferred init via `lv_timer_create(settings_screen_deferred_init, 200, NULL)` + `lv_timer_set_repeat_count(t, 1)` — runs AFTER screen transition completes
- Pattern documented for future: SquareLine event handlers run BEFORE `_ui_screen_change()` destroys/recreates the target screen

### 4. Color Wheel + Theme Color System (DONE)
- Color wheel created dynamically inside `ui_colorWhellPanel` in `settings_screen_init_values()` (not in SquareLine — no re-export damage risk)
- `lv_colorwheel_create(ui_colorWhellPanel, true)` with `lv_colorwheel_set_mode_fixed(true)`, auto-sized to fit panel
- Hex label (#RRGGBB) centered inside wheel, updates on every drag
- `on_colorwheel_changed()`: reads RGB → updates label → `ui_apply_theme_color()` → `ui_save_theme_color()`
- **Saturation guard**: Saved colors with S < 20 (white/gray) are replaced with default blue — prevents invisible all-white arc
- Default theme color: `0x34DB` (blue, ~#3498DB in RGB888)

### 5. Theme Color Application (DONE)
- `ui_apply_theme_color(uint16_t color_raw)`: Sets text color on all labels, border color on all buttons/panels/gauges/dropdowns across both screens
- **Screen 1 themed**: gaugeText1-4, Label1-3, vehicleInfoLabel1, statusLabel1 (text); connectCAN, pollCAN1, settingsButton (borders); gauge1-4 (borders); all 8 dropdowns (text + borders)
- **Screen 2 themed**: brightnessLabel, Label4, Label5 (text); backButton, wifiAPstartButton1 (borders); brightnessSlider (border); systemPanel, colorWhellPanel (borders)
- Boot-time restore in `main.c` Phase 8: `ui_load_theme_color()` → `ui_apply_theme_color()` after `ui_events_post_init()`
- Screen2 re-entry: `settings_screen_init_values()` reapplies theme to freshly created Screen2 widgets

### 6. Theme Color NVS Persistence (DONE)
- `ui_save_theme_color(uint16_t color_raw)`: `nvs_set_u16()` to namespace `"display"`, key `"theme"`
- `ui_load_theme_color()`: `nvs_get_u16()`, defaults to `THEME_COLOR_DEFAULT` (0x34DB)
- Saved on every colorwheel drag (VALUE_CHANGED event)
- Restored at boot (main.c Phase 8) and on every Screen2 entry

## Previously Completed Work

### IMU Integration (ALL COMPLETE — previous session)
- Virtual PID infrastructure: `VPID_BASE (0xFF00)`, `VPID_IMU`, `GAUGE_IS_VIRTUAL()` macro
- imu_display: attach/detach API, dual mode (G-Load ±1.5G / Tilt ±30°), 10Hz timer
- Data Logger: Lateral G + Longitudinal G columns auto-appended when HAS_IMU defined
- ui_events.c: IMU wiring, uniqueness enforcement, NVS restore

### Boot Splash from SD Card (DONE — previous session)
- BMP decoder: 16/24/32 bpp, bottom-up/top-down, PSRAM RGB565
- Configurable splash duration via NVS: namespace `"display"`, key `"splash_ms"`, default 3000ms
- `boot_splash_show()` → `boot_splash_wait()` → `boot_splash_hide()`

### WiFi Manager Design (DOCUMENTED — NOT YET IMPLEMENTED)
- Full architecture in `wifi_manager/README.md`
- QR code two-stage flow, APSTA mode, random password
- API declared in `wifi_manager.h` (14 functions)
- Reference: ToolTruck OTA implementation

## Architecture Notes

### NVS Namespaces
| Namespace | Keys | Component |
|-----------|------|-----------|
| `imu_cal` | bias, R_matrix, valid | qmi8658 |
| `gauge_cfg` | slot0..slot19 (pid_id + unit) | gauge_engine |
| `vehicle` | vin, ecu_count | comm_link |
| `touch_cal` | x_min, x_max, y_min, y_max, x_inv, y_inv | touch_driver |
| `logger` | file_counter | data_logger |
| `display` | `splash_ms`, `bl_pct`, `theme` | boot_splash, display_driver, ui_events |

### SquareLine Studio Cautions
After ANY SquareLine re-export, these files get overwritten and must be fixed:
1. **`ui_events.h`**: Re-add `#include "lvgl.h"`, `void settingsButton(lv_event_t *e);`, `void ui_events_post_init(void);`, theme color function declarations
2. **`ui/CMakeLists.txt`**: Restore `idf_component_register()` format (SquareLine replaces with raw CMake `add_library()`)
3. **`ui_Screen1.c`**: SquareLine-generated `ui_event_settingsButton` never calls `settingsButton()` — our workaround in `ui_events_post_init()` registers it as a separate callback, so no edit needed here

### Screen Transition Pattern
- `_ui_screen_change(&ui_Screen2, ...)` destroys old Screen2 and calls `ui_Screen2_screen_init()` to recreate
- Event handlers registered by SquareLine run BEFORE `_ui_screen_change()` in the same callback
- **Solution**: Defer any widget manipulation with `lv_timer_create(callback, 200, NULL)` + `lv_timer_set_repeat_count(t, 1)`
- Color wheel and brightness slider callbacks are re-registered every Screen2 visit (widgets are new objects)

### Waveshare Backlight (from Waveshare demo ST7701S.c)
- GPIO6 configured as output, then LEDC takes over (13-bit, 5kHz, LEDC_LOW_SPEED_MODE)
- `ledc_fade_func_install(0)` required for reliable duty updates
- Duty formula: `duty = 8191 - 81 * (100 - percent)`, `percent == 0 → duty = 0`
- Active-high: higher duty = brighter

## What's Next (from TODO.md)
1. **WiFi Manager** — Phase 1: WiFi AP + QR code + system status
2. **Custom gauge types** — sweep dials, bar graphs, arc gauges
3. **File rotation** — close + new log file at configurable size limit