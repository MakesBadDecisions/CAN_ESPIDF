/**
 * @file qmi8658.h
 * @brief QMI8658 6-Axis IMU Driver (3-axis accelerometer + 3-axis gyroscope)
 *
 * Provides initialization, configuration, and data reading for the QMI8658
 * IMU on the shared I2C bus. Uses i2c_bus component for bus access.
 *
 * The QMI8658 is present on the Waveshare ESP32-S3-Touch-LCD-2.1 board at
 * I2C address 0x6B, sharing the bus with CST820 touch (0x15), TCA9554 GPIO
 * expander (0x20), and PCF85063 RTC (0x51).
 *
 * Devices without HAS_IMU defined get stub implementations that return
 * ESP_ERR_NOT_SUPPORTED.
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Data Types
// ============================================================================

/** Raw 3-axis sensor data (signed 16-bit) */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} qmi8658_raw_t;

/** Converted 3-axis sensor data (float, physical units) */
typedef struct {
    float x;
    float y;
    float z;
} qmi8658_data_t;

/** Complete IMU reading */
typedef struct {
    qmi8658_data_t accel;       /**< Accelerometer in G */
    qmi8658_data_t gyro;        /**< Gyroscope in degrees/sec */
    float          temperature; /**< Die temperature in °C */
    uint32_t       timestamp;   /**< Reading timestamp (ms since boot) */
} qmi8658_reading_t;

/** Accelerometer full-scale range */
typedef enum {
    QMI8658_ACC_RANGE_2G  = 0,
    QMI8658_ACC_RANGE_4G  = 1,
    QMI8658_ACC_RANGE_8G  = 2,
    QMI8658_ACC_RANGE_16G = 3,
} qmi8658_acc_range_t;

/** Gyroscope full-scale range */
typedef enum {
    QMI8658_GYRO_RANGE_16DPS   = 0,
    QMI8658_GYRO_RANGE_32DPS   = 1,
    QMI8658_GYRO_RANGE_64DPS   = 2,
    QMI8658_GYRO_RANGE_128DPS  = 3,
    QMI8658_GYRO_RANGE_256DPS  = 4,
    QMI8658_GYRO_RANGE_512DPS  = 5,
    QMI8658_GYRO_RANGE_1024DPS = 6,
    QMI8658_GYRO_RANGE_2048DPS = 7,
} qmi8658_gyro_range_t;

/** Output data rate */
typedef enum {
    QMI8658_ODR_8000HZ = 0,
    QMI8658_ODR_4000HZ = 1,
    QMI8658_ODR_2000HZ = 2,
    QMI8658_ODR_1000HZ = 3,
    QMI8658_ODR_500HZ  = 4,
    QMI8658_ODR_250HZ  = 5,
    QMI8658_ODR_125HZ  = 6,
    QMI8658_ODR_62HZ   = 7,
    QMI8658_ODR_31HZ   = 8,
} qmi8658_odr_t;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize the QMI8658 IMU
 *
 * Verifies chip ID, configures accelerometer and gyroscope with device
 * header defaults (range, ODR, LPF), and enables both sensors.
 * Requires i2c_bus_init() to have been called first.
 *
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if chip ID mismatch,
 *         ESP_ERR_NOT_SUPPORTED if HAS_IMU is not defined
 */
esp_err_t qmi8658_init(void);

/**
 * @brief Check if QMI8658 is initialized and running
 */
bool qmi8658_is_initialized(void);

/**
 * @brief Read accelerometer, gyroscope, and temperature in one burst
 *
 * Reads 14 bytes (temp + accel XYZ + gyro XYZ) via auto-increment.
 * Converts raw values to physical units using configured ranges.
 *
 * @param[out] reading  Pointer to reading struct to fill
 * @return ESP_OK on success
 */
esp_err_t qmi8658_read(qmi8658_reading_t *reading);

/**
 * @brief Read raw accelerometer data only
 *
 * @param[out] raw  Pointer to raw struct to fill
 * @return ESP_OK on success
 */
esp_err_t qmi8658_read_accel_raw(qmi8658_raw_t *raw);

/**
 * @brief Read raw gyroscope data only
 *
 * @param[out] raw  Pointer to raw struct to fill
 * @return ESP_OK on success
 */
esp_err_t qmi8658_read_gyro_raw(qmi8658_raw_t *raw);

/**
 * @brief Read die temperature
 *
 * @param[out] temp_c  Temperature in degrees Celsius
 * @return ESP_OK on success
 */
esp_err_t qmi8658_read_temperature(float *temp_c);

/**
 * @brief Set accelerometer full-scale range
 *
 * @param range  Desired range (2G, 4G, 8G, or 16G)
 * @return ESP_OK on success
 */
esp_err_t qmi8658_set_acc_range(qmi8658_acc_range_t range);

/**
 * @brief Set gyroscope full-scale range
 *
 * @param range  Desired range (16 to 2048 DPS)
 * @return ESP_OK on success
 */
esp_err_t qmi8658_set_gyro_range(qmi8658_gyro_range_t range);

/**
 * @brief Power down both sensors (low-power sleep)
 *
 * @return ESP_OK on success
 */
esp_err_t qmi8658_power_down(void);

/**
 * @brief Wake from power down (re-enable both sensors)
 *
 * @return ESP_OK on success
 */
esp_err_t qmi8658_wake(void);

#ifdef __cplusplus
}
#endif
