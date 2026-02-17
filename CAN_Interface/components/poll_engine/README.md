# poll_engine - Smart Polling Scheduler

## Purpose

Schedules and manages OBD-II PID requests using priority-based polling with adaptive intervals. Ensures high-priority data (RPM, throttle) updates frequently while low-priority data (barometric pressure, misfires) polls less often, keeping CAN bus utilization efficient.

## How It Works

### Priority Tiers

| Priority | Typical Interval | Example PIDs |
|----------|-----------------|-------------|
| 1 (Critical) | 50-200ms | RPM, MAF, torque, knock retard, injector PW |
| 2 (High) | 200-600ms | Throttle, fuel rate, boost pressure, spark advance |
| 3 (Normal) | 600-1500ms | Coolant temp, EGR, intake temp, DPF pressure |
| 4 (Low) | 1500-3000ms | Fuel level, oil temp, O2 sensors, hybrid battery |
| 5 (Background) | 3000-12000ms | Barometric pressure, misfires, max airflow |

### Adaptive Intervals

The poll engine tracks success/failure for each PID:

```c
typedef struct {
    uint16_t pid;
    uint32_t interval_ms;       // Current polling interval
    uint32_t base_interval_ms;  // Default interval (from pid_db)
    int64_t  last_poll_us;      // Last request timestamp
    uint8_t  priority;          // 1-5
    uint16_t success_count;
    uint16_t failure_count;
    uint16_t total_requests;
} poll_slot_t;
```

Every 5 seconds, intervals are recalculated:
- **High success rate (>90%):** Interval stays at base or decreases slightly
- **Medium success rate (50-90%):** Interval increases by 1.5x
- **Low success rate (<50%):** Interval increases by 3x (back off)
- **Zero responses:** PID is suspended (marked unsupported)

### Poll Budget

Each poll cycle has a time budget to prevent bus flooding:
- **Max requests per cycle:** Configurable (default 4)
- **Min spacing between requests:** 10ms (configurable)
- **Cycle period:** 10ms task period, actual sends governed by intervals

### Poll List Management

The active poll list is built from:
1. **Supported PIDs** discovered during controller scan
2. **User-selected PIDs** configured via display node (relayed over UART)
3. **Default poll configs** from pid_db with interval/priority

Only PIDs that are both supported by the vehicle AND requested by the user get polled.

## API

```c
// Initialize poll engine with discovered supported PIDs
esp_err_t poll_engine_init(const uint16_t *supported_pids, size_t count);

// Set the active poll list (from user config or display node request)
esp_err_t poll_engine_set_poll_list(const uint16_t *pids, size_t count);

// Called by the poll engine task every cycle
void poll_engine_tick(void);

// Record a successful or failed response
void poll_engine_record_response(uint16_t pid, bool success);

// Get current poll statistics
void poll_engine_get_stats(poll_engine_stats_t *stats);

// Pause/resume polling
void poll_engine_pause(void);
void poll_engine_resume(void);
```

## Files

```
poll_engine/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   └── poll_engine.h        # Public API
└── src/
    ├── poll_engine.c         # Scheduler core, tick logic
    └── poll_adaptive.c       # Adaptive interval adjustment
```

## Dependencies

- `obd2` (sending PID requests)
- `pid_db` (default poll configs, PID validation)
- `system` (logging, timing)
