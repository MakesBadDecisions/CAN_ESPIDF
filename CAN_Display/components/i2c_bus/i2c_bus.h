/**
 * @file i2c_bus.h
 * @brief Shared I2C Bus Manager
 *
 * Provides a single I2C bus instance used by all onboard peripherals.
 * On Waveshare ESP32-S3-Touch-LCD-2.1: CST820, TCA9554, QMI8658, PCF85063.
 * On CrowPanel 5"/7": GT911 touch.
 *
 * Call i2c_bus_init() once during boot, before any peripheral that uses I2C.
 * All peripherals share the bus — thread-safety is handled by the ESP-IDF
 * I2C driver internally (i2c_master_cmd_begin blocks on a per-port mutex).
 */

#pragma once

#include "esp_err.h"
#include "driver/i2c.h"

/**
 * @brief Initialize the shared I2C bus
 *
 * Uses I2C_BUS_SDA, I2C_BUS_SCL, I2C_BUS_FREQ from the active device header.
 * Safe to call multiple times — subsequent calls return ESP_OK immediately.
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bus_init(void);

/**
 * @brief Check if the I2C bus has been initialized
 * @return true if initialized
 */
bool i2c_bus_is_initialized(void);

/**
 * @brief Get the I2C port number used by the bus
 * @return I2C port number (I2C_NUM_0 typically)
 */
i2c_port_t i2c_bus_get_port(void);
