# PCF85063A RTC Driver

## Purpose

I2C real-time clock driver for the PCF85063A on the Waveshare ESP32-S3-Touch-LCD-2.1. Provides accurate timekeeping across power cycles via the onboard coin-cell-backed RTC.

## Design

- **UTC storage** — The RTC always stores UTC time. Timezone offset from the client browser is saved to NVS and applied at display/logging time. This avoids DST ambiguity.
- **Time sync** — When a user connects to the WiFi AP and loads the captive portal, JavaScript automatically sends the phone/laptop's epoch time to `/api/settime`. The ESP32 sets both the system clock (`settimeofday`) and the RTC.
- **Boot restore** — On boot, `pcf85063_sync_to_system()` reads the RTC and sets the ESP32 system clock, so timestamps are accurate immediately without WiFi or internet.

## I2C Details

| Parameter | Value |
|-----------|-------|
| Address   | 0x51 |
| Bus       | I2C_NUM_0 (shared with CST820, TCA9554, QMI8658) |
| Speed     | 400 kHz |
| Registers | 0x04-0x0A (seconds through years, BCD) |

## API

```c
esp_err_t pcf85063_init(void);              // Init + detect + clear OS flag
esp_err_t pcf85063_set_epoch(time_t epoch); // Set system clock + RTC from UTC epoch
esp_err_t pcf85063_get_epoch(time_t *out);  // Read UTC epoch from RTC
esp_err_t pcf85063_sync_to_system(void);    // Boot: RTC → system clock
esp_err_t pcf85063_save_tz_offset(int16_t); // Save timezone to NVS
int16_t   pcf85063_get_tz_offset(void);     // Get saved timezone offset (minutes)
```

## Files

```
pcf85063/
├── README.md
├── CMakeLists.txt
├── pcf85063.h      # Public API
└── pcf85063.c      # Driver + BCD conversion + NVS tz storage
```

## Dependencies

- `i2c_bus` (shared I2C bus)
- `devices` (pin/address defines, HAS_RTC guard)
- `nvs_flash` (timezone offset persistence)
