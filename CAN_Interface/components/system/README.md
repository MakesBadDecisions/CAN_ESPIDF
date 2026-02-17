# system - Core System Infrastructure

## Purpose

Provides foundational services used by all other components: structured logging, NVS configuration storage, timing utilities, and task management helpers.

## Modules

### Logging (`sys_log`)

Wraps ESP-IDF's `esp_log` with project-specific tag management and optional UART forwarding. Each component registers a tag at init.

```c
// Usage
#define TAG "obd2"
SYS_LOGI(TAG, "PID 0x%02X response: %.1f %s", pid, value, unit);
SYS_LOGD(TAG, "ISO-TP frame %d/%d received", seq, total);
SYS_LOGE(TAG, "CAN bus error: %s", can_status_str(status));

// Log levels configurable per-tag at runtime via NVS or WiFi AP
```

### NVS Config (`sys_config`)

Typed key-value configuration stored in ESP-IDF's Non-Volatile Storage. Survives power cycles.

```c
// Store/retrieve settings
sys_config_set_u32("can_baud", 500000);
sys_config_set_str("display_mac", "AA:BB:CC:DD:EE:FF");
sys_config_set_blob("poll_list", pid_array, sizeof(pid_array));

uint32_t baud = sys_config_get_u32("can_baud", 500000);  // default if not set
```

### Timing (`sys_time`)

Overflow-safe timing utilities built on `esp_timer_get_time()`. Migrated from the Arduino project's timing module.

```c
int64_t now = sys_time_us();           // Microseconds since boot (64-bit)
uint32_t now_ms = sys_time_ms();       // Milliseconds since boot (32-bit)
bool elapsed = sys_timeout_check(start_us, timeout_us);  // Overflow-safe
void sys_delay_ms(uint32_t ms);        // FreeRTOS vTaskDelay wrapper
```

### Task Helpers (`sys_task`)

Wrappers for FreeRTOS task creation with standardized stack sizes, priorities, and core pinning.

```c
sys_task_create("can_proc", can_processing_task,
                CORE_1, PRIORITY_5, STACK_4K, &task_handle);
```

## Files

```
system/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── sys_log.h            # Logging macros and config
│   ├── sys_config.h         # NVS config API
│   ├── sys_time.h           # Timing utilities
│   └── sys_task.h           # Task creation helpers
└── src/
    ├── sys_log.c
    ├── sys_config.c
    ├── sys_time.c
    └── sys_task.c
```

## Dependencies

- `esp_log` (ESP-IDF logging)
- `nvs_flash` (non-volatile storage)
- `esp_timer` (microsecond timer)
- `freertos` (task management)
