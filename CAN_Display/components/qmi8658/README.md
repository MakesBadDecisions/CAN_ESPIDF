# qmi8658 - 6-Axis IMU Driver (Accelerometer + Gyroscope)

## Purpose

Driver for the QMI8658 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
on the shared I2C bus. Provides initialization, configuration, burst data
reading, background orientation fusion, and NVS-persistent calibration.

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
| Gyroscope | ±16 – ±2048 DPS | ±512 DPS | 31Hz – 8kHz | 8kHz |
| Temperature | — | — | (read with sensor data) | — |

## Boot Calibration

On first boot (or when no saved calibration exists), the IMU task performs a
2-second calibration phase (100 samples at 50Hz). The device must be held still
during this period.

Calibration computes:
- **Gyro bias**: 3-axis average subtracted from all future readings
- **Rotation matrix**: Rodrigues' formula maps measured gravity → Z-up reference frame,
  allowing the device to be mounted in any orientation

Calibration data is automatically saved to NVS (namespace `imu_cal`) and restored
on subsequent boots, skipping the 2-second wait. Use `qmi8658_calibrate()` to
force a fresh calibration, or `qmi8658_clear_calibration()` to erase stored data.

## Orientation Fusion

A background FreeRTOS task (Core 0, 50Hz) reads the IMU and runs a complementary
filter (α=0.98) combining:
- Short-term: gyroscope integration (pitch, roll)
- Long-term: accelerometer tilt correction
- Yaw rate: calibrated gyro Z-axis (rotation about vertical)

The fused result is written to a volatile cache, safe for lock-free cross-core reads.

### Reference Frame (after calibration)

| Axis | Direction | Usage |
|------|-----------|-------|
| X | Forward | Longitudinal acceleration, pitch |
| Y | Right | Lateral acceleration, roll |
| Z | Up | Vertical acceleration (~1G at rest) |

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

// Background task + orientation
esp_err_t qmi8658_start_task(void);          // Start 50Hz polling + fusion
void      qmi8658_stop_task(void);           // Stop background task
esp_err_t qmi8658_get_orientation(qmi8658_orientation_t *orient);  // Lock-free read

// Calibration management
esp_err_t qmi8658_calibrate(void);           // Force live recalibration (2s)
esp_err_t qmi8658_clear_calibration(void);   // Erase NVS calibration data
```

## Data Types

```c
typedef struct {
    qmi8658_data_t accel;       // X/Y/Z in G
    qmi8658_data_t gyro;        // X/Y/Z in degrees/sec
    float          temperature; // Die temperature in °C
    uint32_t       timestamp;   // ms since boot
} qmi8658_reading_t;

typedef struct {
    float pitch;            // Pitch angle in degrees (-90 to +90)
    float roll;             // Roll angle in degrees (-180 to +180)
    float yaw_rate;         // Yaw rotation rate in °/s
    float accel_lat;        // Lateral acceleration in G (right = +)
    float accel_lon;        // Longitudinal acceleration in G (forward = +)
    float accel_vert;       // Vertical acceleration in G (up = +, ~1.0 at rest)
    float g_total;          // Total G-force magnitude
    float temperature;      // Die temperature in °C
    uint32_t timestamp;     // Last update timestamp (ms since boot)
    bool   valid;           // True after first successful read
} qmi8658_orientation_t;
```

## NVS Storage

| Namespace | Key | Type | Content |
|-----------|-----|------|---------|
| `imu_cal` | `cal` | blob (49 bytes) | Version byte + gyro bias[3] + rotation matrix[3][3] |

## Files

```
qmi8658/
├── README.md        # This file
├── CMakeLists.txt   # REQUIRES: driver, esp_timer, devices, i2c_bus, nvs_flash
├── qmi8658.h        # Public API, data types, enums
└── qmi8658.c        # Implementation (~815 lines)
```

## Dependencies

- `i2c_bus` — Shared I2C bus manager
- `devices` — `QMI8658_I2C_ADDR`, `HAS_IMU`, range/ODR defaults
- `nvs_flash` — Calibration persistence
- `driver` — I2C master API
- `esp_timer` — Timestamp for readings
