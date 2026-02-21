# imu_display - IMU Bubble Visualization

## Purpose

Renders an IMU G-force / tilt bubble with acceleration labels inside any
gauge slot panel. Appears as the "IMU" entry in the PID dropdown — the user
selects it like any other PID, and the bubble replaces the numeric value
in that gauge slot. Separated into its own component to avoid being
overwritten by SquareLine re-exports.

## Visual Layout

```
┌──────────────────────────┐
│  [PID dropdown]          │  ← managed by SquareLine / ui_events
│ ┌──────────────────────┐ │
│ │       0.2G            │ │  ← lon G / pitch at top of V-axis
│ │         |             │ │
│ │  ───────●────── 0.1G  │ │  ← lat G / roll at right of H-axis
│ │         |             │ │
│ │         |             │ │
│ └──────────────────────┘ │
│  [Unit dropdown]         │  ← "G-Load" / "Tilt" mode selector
└──────────────────────────┘
```

Clean crosshair with bubble dot — values shown at axis endpoints.
The inner container occupies the center 60% of the gauge panel (top/bottom
20% reserved for dropdowns).

## Display Modes

| Mode (unit dropdown) | Horizontal axis | Vertical axis | Range |
|---|---|---|---|
| **G-Load** | Lateral G (cornering) | Longitudinal G (accel/brake) | ±1.5G |
| **Tilt** | Roll angle | Pitch angle | ±30° |

- **Bubble color**: Green (<0.3G dynamic) → Yellow (0.3–0.8G) → Red (>0.8G)
- **Axis labels**: Lateral G / roll at right of horizontal crosshair, longitudinal G / pitch at top of vertical crosshair

## Data Source

Reads `qmi8658_get_orientation()` at 10 Hz via an LVGL timer. Lock-free
volatile cache written by the IMU task on Core 0, safe to read from LVGL
context on Core 1.

## API

```c
#include "imu_display.h"

// Attach bubble to a gauge panel (e.g., ui_gauge1)
imu_display_attach(lv_obj_t *parent);

// Detach from current panel, delete all widgets
imu_display_detach();

// Check state
bool imu_display_is_attached(void);
lv_obj_t *imu_display_get_parent(void);

// Switch G-Load / Tilt mode
imu_display_set_mode(IMU_MODE_G_LOAD);  // or IMU_MODE_TILT
```

Attach/detach called from `on_pid_changed()` in `ui_events.c` when the
user selects/deselects "IMU" in a PID dropdown. Only one instance active
at a time (single physical sensor).

## Virtual PID Integration

IMU appears as a virtual PID (`VPID_IMU = 0xFF00`) in gauge_engine:
- `gauge_engine_build_pid_options()` appends "IMU" after all CAN PIDs
- `gauge_engine_set_pid()` handles VPID_IMU without comm_link lookup
- `gauge_engine_update()` reads qmi8658 for VPID_IMU slots, not comm_link
- `gauge_engine_get_unit_options()` returns "G-Load\nTilt" for IMU slots
- NVS persistence works naturally (VPID_IMU = 0xFF00 stored as pid_id)

## Files

```
imu_display/
├── README.md          # This file
├── CMakeLists.txt     # REQUIRES: lvgl, qmi8658, devices, gauge_engine, log
├── imu_display.h      # Public API (attach/detach/set_mode)
└── imu_display.c      # Implementation
```

## Dependencies

- `lvgl` — LVGL widget creation and timer
- `qmi8658` — IMU orientation data via `qmi8658_get_orientation()`
- `devices` — `HAS_IMU` compile-time flag
- `gauge_engine` — `imu_display_mode_t` enum, `VPID_IMU` defines

## Notes

This component lives in `components/imu_display/` (not inside `components/ui/`)
because SquareLine Studio re-exports overwrite all files in the `ui/` directory.
