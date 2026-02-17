# sys_mgr - System Manager

## Purpose

Coordinates runtime operation of the CAN Interface Node. Manages task supervision, state machine, health monitoring, and error recovery.

## State Machine

```
INIT → CONNECTING_CAN → CONNECTING_COMM → RUNNING ↔ ERROR
                                              ↓
                                           FATAL
```

| State | Description |
|-------|-------------|
| INIT | Components initializing, tasks not yet started |
| CONNECTING_CAN | Waiting for CAN bus response / ECU discovery |
| CONNECTING_COMM | Waiting for comm link handshake with Display Node |
| RUNNING | Normal operation, streaming PID data |
| ERROR | Recoverable error, attempting recovery |
| FATAL | Unrecoverable, requires system reset |

## Responsibilities

- **Task Supervision**: Monitor registered tasks for watchdog timeouts
- **Health Monitoring**: Track heap usage, error counters, uptime
- **State Management**: Validate and execute state transitions
- **Recovery Coordination**: Trigger CAN bus-off recovery, comm link reconnect
- **Statistics**: Aggregate system-wide counters for debugging

## API

### Initialization
```c
esp_err_t sys_mgr_init(void);   // Call during init sequence
esp_err_t sys_mgr_start(void);  // Call after all tasks created
```

### Task Registration
```c
esp_err_t sys_mgr_register_task(const char *name, TaskHandle_t handle, uint32_t wdt_timeout_ms);
void sys_mgr_task_checkin(TaskHandle_t handle);  // Reset task watchdog
```

### State & Errors
```c
sys_state_t sys_mgr_get_state(void);
const char* sys_mgr_state_name(sys_state_t state);
esp_err_t sys_mgr_request_state(sys_state_t new_state);
void sys_mgr_report_error(sys_error_t error, const char *details);
```

### Statistics
```c
void sys_mgr_get_stats(sys_stats_t *stats);
SYS_STAT_CAN_RX(n);   // Increment CAN RX counter
SYS_STAT_CAN_TX(n);   // Increment CAN TX counter
SYS_STAT_OBD_RESP(n); // Increment OBD response counter
SYS_STAT_COMM_TX(n);  // Increment comm TX counter
```

## Usage Pattern

**In main.c:**
```c
// 1. Init sys_mgr first
sys_mgr_init();

// 2. Init all components
can_driver_init();
obd2_init();
// ...

// 3. Create all tasks, register with sys_mgr
xTaskCreatePinnedToCore(can_task, "can", ...);
sys_mgr_register_task("can", can_task_handle, 5000);  // 5s watchdog

// 4. Start sys_mgr (transitions to CONNECTING_CAN)
sys_mgr_start();
```

**In task functions:**
```c
void can_task(void *arg) {
    while (1) {
        sys_mgr_task_checkin(NULL);  // Reset watchdog
        // ... do work ...
    }
}
```

## Dependencies

- `system` - Logging, timing utilities
- `freertos` - Task management, semaphores
