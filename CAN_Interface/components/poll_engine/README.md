# poll_engine - Smart Polling Scheduler

## Purpose

Schedules and manages OBD-II PID requests using priority-based polling with round-robin fairness. Ensures high-priority data (RPM, throttle) updates frequently while low-priority data (barometric pressure, misfires) polls less often, keeping CAN bus utilization efficient.

## How It Works

### Priority Tiers

| Priority | Typical Interval | Example PIDs |
|----------|-----------------|-------------|
| 1 (Critical) | 50-200ms | RPM, MAF, torque, knock retard, injector PW |
| 2 (High) | 200-600ms | Throttle, fuel rate, boost pressure, spark advance |
| 3 (Normal) | 600-1500ms | Coolant temp, EGR, intake temp, DPF pressure |
| 4 (Low) | 1500-3000ms | Fuel level, oil temp, O2 sensors, hybrid battery |
| 5 (Background) | 3000-12000ms | Barometric pressure, misfires, max airflow |

### Round-Robin Scheduling

The scheduler uses a round-robin index to guarantee every PID gets serviced, even when many jobs share the same priority. Each tick, it scans for the highest-priority due job starting from *one position past* the last-polled index, wrapping around the full job array. This prevents early-index jobs from starving later ones.

**Key implementation details:**
- `last_poll_index` tracks where the previous scan selected a job
- Next scan starts at `(last_poll_index + 1) % count` and wraps
- Tie-breaking within equal priority is handled by scan order (round-robin), not array position
- Reset to `-1` on init and `poll_engine_clear_all()`

### Measured Performance (20 PIDs, MCP2515 @ 500kbps)

| Metric | Value |
|--------|-------|
| Per-PID poll time | ~45ms (OBD2 request is blocking ~25-35ms) |
| Full rotation (20 PIDs) | ~900ms |
| Per-PID update rate | ~1.1 Hz |
| Inter-poll delay | 1 tick (~1ms, CPU yield only) |
| Timeouts / failures | 0 across 3+ rotations |

The natural spacing from the blocking `obd2_request_pid()` call (~25-35ms) provides sufficient inter-request delay for the MCP2515's 2-buffer RX. No additional artificial delay is needed.

### Adaptive Intervals (Disabled by Default)

The poll engine includes adaptive interval logic that adjusts polling frequency based on success/failure rates. **This is disabled by default** (`adaptive_interval = false` in `POLL_ENGINE_CONFIG_DEFAULT`) because it can cause scheduler starvation — PIDs that experience transient failures get backed off aggressively, resulting in only a subset of PIDs being actively polled.

When enabled, intervals are recalculated every 5 seconds:
- **High success rate (>90%):** Interval stays at base or decreases slightly
- **Medium success rate (50-90%):** Interval increases by 1.5x
- **Low success rate (<50%):** Interval increases by 3x (back off)
- **Zero responses:** PID is suspended (marked unsupported)

### Poll Budget

Each poll cycle has a time budget to prevent bus flooding:
- **Max requests per cycle:** Configurable (default 4)
- **Cycle period:** 10ms task period, actual sends governed by intervals

### Poll List Management

The active poll list is built from:
1. **Supported PIDs** discovered during controller scan
2. **User-selected PIDs** configured via display node (relayed over UART)
3. **Default poll configs** from pid_db with interval/priority

Only PIDs that are both supported by the vehicle AND requested by the user get polled. If the `pid_store` component has an active user selection, `gauge_engine` defers to it rather than overwriting.

## API

```c
// Initialize poll engine with config
esp_err_t poll_engine_init_with_config(const poll_engine_config_t *cfg);

// Set the active poll list (from user config or display node request)
esp_err_t poll_engine_set_poll_list(const uint16_t *pids, size_t count);

// Clear all poll jobs and reset round-robin index
void poll_engine_clear_all(void);

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
├── poll_engine.h            # Public API, config struct, defaults
└── poll_engine.c            # Scheduler core, tick logic, round-robin
```

## Dependencies

- `obd2` (sending PID requests)
- `pid_db` (default poll configs, PID validation)
- `system` (logging, timing)

## Bug Fix History

### Scheduler Starvation (Fixed)

**Symptom:** Only 3 of 20 selected PIDs were actively polled. The first few PIDs in the job array monopolized the cycle.

**Root Cause:** The job selection loop always scanned from array index 0, using a strict `>` comparison for priority. When all jobs had equal priority (3), the first due job found (lowest array index) always won. With 200ms intervals and ~30ms per request, the first 3 PIDs kept becoming due before later PIDs ever got a turn.

**Fix:** Added `last_poll_index` tracking and modified the scan to start from `(last_poll_index + 1) % count`. This ensures all equal-priority PIDs are serviced in round-robin order. Verified on vehicle with 20 PIDs across 3+ complete rotations with zero failures.
