# system - Core System Infrastructure (Display Node)

## Purpose

Core system services for the display node: NVS initialization, logging macros,
timing utilities, and device info reporting.

## API

```c
#include "system.h"

esp_err_t system_init(void);         // Init NVS flash
void      sys_print_device_info(void); // Print chip, flash, PSRAM, MAC
int64_t   sys_time_us(void);         // Microsecond timestamp
uint32_t  sys_time_ms(void);         // Millisecond timestamp
uint32_t  sys_get_free_heap(void);   // Free heap bytes
```

### Log Macros

```c
SYS_LOGE("error: %s", msg);   // ESP_LOGE with "sys" tag
SYS_LOGW("warning: %s", msg);
SYS_LOGI("info: %s", msg);
SYS_LOGD("debug: %s", msg);
```

## Files

```
system/
├── README.md
├── CMakeLists.txt
├── system.h        # Public API and log macros
└── system.c        # Implementation
```

## Dependencies

- `esp_log`, `nvs_flash`, `esp_timer`, `esp_system`
