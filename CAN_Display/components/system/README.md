# system - Core System Infrastructure (Display Node)

## Purpose

Same role as the CAN interface node's system component: logging, NVS config, timing, and task helpers. Shared API, separate instance per node.

This is intentionally a near-duplicate of the CAN node's system component. Both nodes need the same foundational services. In the future, this could be extracted to `shared/` if the implementations are identical.

## Modules

- **sys_log** - Structured logging via `esp_log` with per-tag level control
- **sys_config** - NVS key-value storage for persistent settings
- **sys_time** - Overflow-safe timing utilities
- **sys_task** - FreeRTOS task creation helpers

## Files

```
system/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── sys_log.h
│   ├── sys_config.h
│   ├── sys_time.h
│   └── sys_task.h
└── src/
    ├── sys_log.c
    ├── sys_config.c
    ├── sys_time.c
    └── sys_task.c
```

## Dependencies

- `esp_log`, `nvs_flash`, `esp_timer`, `freertos`
