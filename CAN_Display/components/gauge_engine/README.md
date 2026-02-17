# gauge_engine - Gauge Rendering & Layout

## Purpose

Renders automotive gauges on the display using data from the PID value store. Supports multiple gauge types, configurable layouts, and alert/warning thresholds.

## Gauge Types

| Type | Description | Best For |
|------|-------------|----------|
| **Numeric** | Large number with label and units | RPM, speed, temp |
| **Bar** | Horizontal or vertical fill bar | Throttle, fuel level, load |
| **Sweep** | Arc/dial gauge with needle | RPM tach, speedometer, boost |
| **Min/Max** | Number with min/max history | Knock retard, fuel trim |
| **Status** | On/off or enum text | MIL, DFCO, PE active |
| **Graph** | Scrolling time-series line | Trend monitoring |

## Layout System

Gauges are arranged in configurable layouts. Each layout defines a grid of gauge slots:

```c
typedef struct {
    uint8_t     row;
    uint8_t     col;
    uint8_t     row_span;   // 1 = single cell, 2 = double height
    uint8_t     col_span;   // 1 = single cell, 2 = double width
    gauge_type_t type;
    uint16_t    pid;        // Which PID to display
    unit_t      display_unit; // Desired display unit (may differ from native)
    float       min_value;  // Scale minimum
    float       max_value;  // Scale maximum
    float       warn_low;   // Yellow warning below this
    float       warn_high;  // Yellow warning above this
    float       crit_low;   // Red critical below this
    float       crit_high;  // Red critical above this
} gauge_slot_t;

typedef struct {
    const char   *name;         // "Street", "Track", "Diagnostics"
    uint8_t       rows;
    uint8_t       cols;
    gauge_slot_t *slots;
    uint8_t       slot_count;
} gauge_layout_t;
```

Multiple layouts can be stored and switched at runtime (e.g., "Street" with 4 gauges, "Track" with 8 gauges, "Diag" with all values).

## Alert System

Each gauge slot can define warning and critical thresholds:

- **Normal:** Standard color rendering
- **Warning:** Yellow/amber highlight, optional audible beep
- **Critical:** Red flash, persistent alert until acknowledged

Alerts are evaluated every render cycle against the current PID value.

## Rendering Pipeline

```
1. Read PID values from pid_store (comm_link provides these)
2. For each gauge slot in active layout:
   a. Get current value for slot's PID
   b. Convert to display unit if needed
   c. Check alert thresholds
   d. Render gauge to framebuffer (only if value changed)
3. Render status bar (connection state, log status, alerts)
4. Flush dirty regions to display hardware
```

## Configuration

Gauge layouts are stored in NVS and configurable via WiFi web UI. Default layouts are compiled in for out-of-box use.

## Files

```
gauge_engine/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── gauge_engine.h       # Public API: init, render_tick, set_layout
│   ├── gauge_types.h        # Gauge type definitions, layout structs
│   └── gauge_alerts.h       # Alert threshold checking
└── src/
    ├── gauge_engine.c        # Layout management, render coordinator
    ├── gauge_numeric.c       # Numeric gauge renderer
    ├── gauge_bar.c           # Bar gauge renderer
    ├── gauge_sweep.c         # Sweep/dial gauge renderer
    ├── gauge_status.c        # Status indicator renderer
    ├── gauge_graph.c         # Time-series graph renderer
    └── gauge_alerts.c        # Alert evaluation and visual effects
```

## Dependencies

- `display_driver` (framebuffer drawing primitives)
- `comm_link/pid_store` (current PID values)
- `shared/pid_types` (unit conversions for display)
- `system` (logging, NVS for layout storage)
