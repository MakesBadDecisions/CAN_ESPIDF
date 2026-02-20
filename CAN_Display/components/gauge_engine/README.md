# gauge_engine - Gauge Data Manager

## Purpose

Owns per-gauge state (PID assignment, unit selection, live converted values) and aggregates active gauges into a single deduplicated poll list sent to the CAN Interface Node. This component has **no LVGL dependency** -- the UI layer (`ui_events.c`) reads formatted value strings from here and pushes them to widgets.

Designed to support up to **20 simultaneous gauge slots** across multiple screens.

## Architecture

```
┌──────────────────────────────────────────────────────┐
│  UI Layer (ui_events.c)                              │
│  - Table-driven: gauge_widget_t[] maps slots→widgets │
│  - on_pid_changed / on_unit_changed callbacks        │
│  - LVGL timer reads value_str, pushes to labels      │
└────────────────┬─────────────────────────────────────┘
                 │  gauge_engine API
┌────────────────▼─────────────────────────────────────┐
│  gauge_engine (this component)                       │
│  - s_slots[GAUGE_MAX_SLOTS] (mutex-protected)        │
│  - PID assignment via comm_link metadata index       │
│  - Unit conversion via pid_types                     │
│  - Poll list aggregation with PID deduplication      │
│  - Formatted value_str output per slot               │
└────────────────┬─────────────────────────────────────┘
                 │  comm_link / pid_types APIs
┌────────────────▼─────────────────────────────────────┐
│  comm_link          │  shared/pid_types              │
│  - PID data store   │  - pid_unit_convert()          │
│  - PID metadata     │  - pid_unit_get_alts()         │
│  - set_poll_list()  │  - pid_unit_to_str()           │
└─────────────────────┴────────────────────────────────┘
```

## Gauge Slot

Each slot holds PID assignment, unit selection, and live data:

```c
typedef struct {
    uint16_t    pid_id;                         // Selected PID (0xFFFF = none)
    pid_unit_t  base_unit;                      // PID's native unit
    pid_unit_t  display_unit;                   // Currently selected display unit
    float       raw_value;                      // Last value in base units
    float       display_value;                  // Converted to display_unit
    char        value_str[GAUGE_VALUE_STR_LEN]; // Formatted string "123.4"
    bool        value_valid;                    // Got at least one reading
    uint32_t    last_update_tick;               // Tick of last update
} gauge_slot_t;
```

## API Summary

| Function | Purpose |
|----------|---------|
| `gauge_engine_init()` | Zero all slots, create mutex |
| `gauge_engine_set_pid(slot, pid_index)` | Assign PID from comm_link metadata index |
| `gauge_engine_set_unit(slot, unit_index)` | Change display unit (0 = base, 1+ = alternates) |
| `gauge_engine_clear_slot(slot)` | Un-assign a gauge slot |
| `gauge_engine_get_slot(slot)` | Read-only pointer to slot state |
| `gauge_engine_get_unit_options(slot, buf, len)` | Build unit dropdown string for slot's PID |
| `gauge_engine_build_pid_options(buf, len)` | Build PID dropdown string from scan metadata |
| `gauge_engine_start_polling(rate_hz)` | Aggregate assigned PIDs, send poll list |
| `gauge_engine_stop_polling()` | Stop polling, clear poll list |
| `gauge_engine_is_polling()` | Check poll state |
| `gauge_engine_update()` | Read latest values, convert, format strings |
| `gauge_engine_rebuild_poll_list()` | Re-aggregate after PID assignment change |
| `gauge_engine_get_active_pid_count()` | Count unique PIDs in poll list |

## Thread Safety

All slot access is protected by a FreeRTOS mutex. The UI layer calls set/get functions from the LVGL task (Core 1), while `gauge_engine_update()` reads from `comm_link` which is updated by the UART RX task (Core 0).

## Adding More Gauges

1. **SquareLine Studio**: Add gauge container with `piddropdownN`, `unitdropdownN`, `gaugeTextN`.
2. **ui_events.c**: Add an entry to `s_gauge_widgets[]` mapping the new slot index to the new widget pointers.
3. No changes needed in `gauge_engine` -- slots are pre-allocated up to `GAUGE_MAX_SLOTS` (20).

## Files

```
gauge_engine/
├── README.md           # This file
├── CMakeLists.txt      # REQUIRES: comm_link pid_types
├── gauge_engine.h      # Public API and gauge_slot_t struct
└── gauge_engine.c      # Implementation (~280 lines)
```

## Dependencies

- `comm_link` (PID data store, metadata, poll list control)
- `shared/pid_types` (unit conversion, unit string formatting)

## Future

- [ ] Alert thresholds -- warning/critical limits with color changes
- [ ] Value smoothing -- low-pass filter for jittery readings
- [ ] Gauge type rendering -- sweep dials, bar graphs (currently numeric only)
