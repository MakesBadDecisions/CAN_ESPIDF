# i2c_bus - Shared I2C Bus Manager

## Purpose

Initializes and manages the shared I2C bus used by multiple onboard
peripherals. Provides a single init point so all consumers (touch, GPIO
expander, IMU, RTC) get the same bus configuration without conflicting
initializations.

Present on the **Waveshare ESP32-S3-Touch-LCD-2.1** board. Devices without
I2C peripherals get stub implementations.

## I2C Bus Devices (Waveshare 2.1")

| Address | Device | Purpose |
|---------|--------|---------|
| 0x15 | CST820 | Capacitive touch controller |
| 0x20 | TCA9554 | GPIO expander (LCD/touch reset, SD D3, buzzer) |
| 0x51 | PCF85063 | Real-time clock |
| 0x6B | QMI8658 | 6-axis IMU (accelerometer + gyroscope) |

## Configuration

Bus parameters come from the device header (`ws_touch_lcd_2_1.h`):

| Define | Value | Purpose |
|--------|-------|---------|
| `I2C_BUS_SDA` | GPIO 15 | I2C data line |
| `I2C_BUS_SCL` | GPIO 7 | I2C clock line |
| `I2C_BUS_FREQ` | 400000 | Bus speed (400 kHz) |
| `I2C_BUS_PORT` | I2C_NUM_0 | ESP-IDF I2C port |

## API

```c
#include "i2c_bus.h"

esp_err_t    i2c_bus_init(void);           // Configure and install I2C driver
bool         i2c_bus_is_initialized(void); // Check init state
i2c_port_t   i2c_bus_get_port(void);       // Get port for I2C operations
```

After calling `i2c_bus_init()`, all consumers use the legacy I2C master API
(`i2c_master_write_to_device`, `i2c_master_write_read_device`) with the port
returned by `i2c_bus_get_port()`.

## Files

```
i2c_bus/
├── README.md        # This file
├── CMakeLists.txt   # REQUIRES: driver, devices
├── i2c_bus.h        # Public API
└── i2c_bus.c        # Implementation (~60 lines)
```

## Dependencies

- `driver` — I2C master driver
- `devices` — Pin definitions (`I2C_BUS_SDA`, `I2C_BUS_SCL`, etc.)
