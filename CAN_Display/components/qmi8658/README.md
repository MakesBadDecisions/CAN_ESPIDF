# qmi8658 - 6-Axis IMU Driver (Accelerometer + Gyroscope)

## Purpose

Driver for the QMI8658 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
on the shared I2C bus. Provides initialization, configuration, and burst data
reading with automatic conversion to physical units (G and DPS).

Present on the **Waveshare ESP32-S3-Touch-LCD-2.1** board. Devices without
`HAS_IMU` defined get stub implementations that return `ESP_ERR_NOT_SUPPORTED`.

## Hardware

- **Chip**: QMI8658 (QST Corporation)
- **Interface**: I2C at address `0x6B`
- **Bus**: Shared I2C_NUM_0 with CST820 touch (0x15), TCA9554 (0x20), PCF85063 RTC (0x51)
- **Pins**: SDA=GPIO15, SCL=GPIO7 (via `i2c_bus` component)
- **No interrupt pin** connected to ESP32 — uses polling

## Capabilities

| Sensor | Range Options | Default | ODR Options | Default |
|--------|--------------|---------|-------------|---------|
| Accelerometer | ±2G, ±4G, ±8G, ±16G | ±4G | 31Hz – 8kHz | 8kHz |
| Gyroscope | ±16 – ±2048 DPS | ±64 DPS | 31Hz – 8kHz | 8kHz |
| Temperature | — | — | (read with sensor data) | — |

## Initialization

Requires `i2c_bus_init()` before calling `qmi8658_init()`. The init sequence:

1. Read WHO_AM_I register — verify chip ID is `0x05`
2. Enable 2MHz oscillator and auto-address-increment (CTRL1)
3. Enable accelerometer + gyroscope with high-speed clock (CTRL7 = 0x43)
4. Configure accel range + ODR from device header defines (CTRL2)
5. Configure gyro range + ODR from device header defines (CTRL3)
6. Enable low-pass filters — accel LPF mode 0 (2.66%), gyro LPF mode 3 (13.37%)

## API

```c
#include "qmi8658.h"

// Initialize (after i2c_bus_init)
esp_err_t qmi8658_init(void);
bool      qmi8658_is_initialized(void);

// Full reading (temp + accel + gyro in one 14-byte burst)
esp_err_t qmi8658_read(qmi8658_reading_t *reading);

// Individual reads
esp_err_t qmi8658_read_accel_raw(qmi8658_raw_t *raw);
esp_err_t qmi8658_read_gyro_raw(qmi8658_raw_t *raw);
esp_err_t qmi8658_read_temperature(float *temp_c);

// Runtime range adjustment
esp_err_t qmi8658_set_acc_range(qmi8658_acc_range_t range);
esp_err_t qmi8658_set_gyro_range(qmi8658_gyro_range_t range);

// Power management
esp_err_t qmi8658_power_down(void);
esp_err_t qmi8658_wake(void);
```

## Data Types

```c
typedef struct {
    qmi8658_data_t accel;       // X/Y/Z in G
    qmi8658_data_t gyro;        // X/Y/Z in degrees/sec
    float          temperature; // Die temperature in °C
    uint32_t       timestamp;   // ms since boot
} qmi8658_reading_t;
```

## Usage Example

```c
qmi8658_reading_t imu;
if (qmi8658_read(&imu) == ESP_OK) {
    printf("Accel: %.3fG, %.3fG, %.3fG\n", imu.accel.x, imu.accel.y, imu.accel.z);
    printf("Gyro:  %.1f, %.1f, %.1f DPS\n", imu.gyro.x, imu.gyro.y, imu.gyro.z);
    printf("Temp:  %.1f°C\n", imu.temperature);
}
```

## Files

```
qmi8658/
├── README.md        # This file
├── CMakeLists.txt   # REQUIRES: driver, esp_timer, devices, i2c_bus
├── qmi8658.h        # Public API, data types, enums
└── qmi8658.c        # Implementation (~330 lines)
```

## Dependencies

- `i2c_bus` — Shared I2C bus manager
- `devices` — `QMI8658_I2C_ADDR`, `HAS_IMU`, range/ODR defaults
- `driver` — I2C master API
- `esp_timer` — Timestamp for readings
