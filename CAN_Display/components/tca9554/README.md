# tca9554 - I2C GPIO Expander Driver

## Purpose

Driver for the TCA9554 8-bit I2C GPIO expander used on the Waveshare
ESP32-S3-Touch-LCD-2.1 board. Controls LCD reset, touch reset, LCD SPI CS,
SD card D3, and buzzer through the shared I2C bus.

Devices without `HAS_GPIO_EXPANDER` defined get stub implementations.

## Pin Mapping (Waveshare 2.1")

| Pin | Function | Init State | Usage |
|-----|----------|------------|-------|
| EXIO1 | LCD reset | HIGH | Pulse LOW to reset ST7701S |
| EXIO2 | Touch reset | HIGH | Pulse LOW to reset CST820 |
| EXIO3 | LCD SPI CS | HIGH | Active LOW during ST7701S SPI init, then HIGH |
| EXIO4 | SD card D3 | HIGH | Must be HIGH for SDMMC 1-bit mode |
| EXIO5 | Unused | LOW | — |
| EXIO6 | Unused | LOW | — |
| EXIO7 | Unused | LOW | — |
| EXIO8 | Buzzer | LOW | Set HIGH to activate |

## API

```c
#include "tca9554.h"

esp_err_t tca9554_init(void);                          // Configure all pins
esp_err_t tca9554_set_pin(tca9554_pin_t pin, bool high); // Set individual pin
esp_err_t tca9554_set_all(uint8_t value);              // Set all 8 pins at once
uint8_t   tca9554_get_state(void);                     // Read shadow register
bool      tca9554_is_initialized(void);                // Check init state
```

## Shadow Register

All pin states are tracked in a software shadow register. `tca9554_set_pin()`
reads the shadow, modifies the target bit, and writes the full byte over I2C.
This avoids read-modify-write I2C transactions.

## Files

```
tca9554/
├── README.md        # This file
├── CMakeLists.txt   # REQUIRES: driver, devices, i2c_bus
├── tca9554.h        # Public API, pin enum
└── tca9554.c        # Implementation (~120 lines)
```

## Dependencies

- `i2c_bus` — Shared I2C bus manager
- `devices` — `TCA9554_I2C_ADDR`, `HAS_GPIO_EXPANDER`
- `driver` — I2C master API
