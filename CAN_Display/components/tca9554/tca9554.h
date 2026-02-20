/**
 * @file tca9554.h
 * @brief TCA9554 I2C GPIO Expander Driver
 *
 * Provides pin-level control of the TCA9554PWR 8-bit I/O expander.
 * On Waveshare ESP32-S3-Touch-LCD-2.1, the TCA9554 controls:
 *   EXIO1 — LCD Reset
 *   EXIO2 — Touch Reset
 *   EXIO3 — LCD SPI CS
 *   EXIO4 — SD Card D3
 *   EXIO8 — Buzzer
 *
 * All pins are configured as outputs on init.
 * Thread-safe — uses the I2C bus which has internal mutex.
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// TCA9554 EXIO pin numbers (1-indexed to match Waveshare convention)
typedef enum {
    EXIO_PIN_1 = 0,     // Bit 0
    EXIO_PIN_2 = 1,     // Bit 1
    EXIO_PIN_3 = 2,     // Bit 2
    EXIO_PIN_4 = 3,     // Bit 3
    EXIO_PIN_5 = 4,     // Bit 4
    EXIO_PIN_6 = 5,     // Bit 5
    EXIO_PIN_7 = 6,     // Bit 6
    EXIO_PIN_8 = 7,     // Bit 7
} tca9554_pin_t;

/**
 * @brief Initialize TCA9554 GPIO expander
 *
 * Requires i2c_bus_init() to have been called first.
 * Configures all 8 pins as outputs, all LOW initially.
 *
 * @return ESP_OK on success
 */
esp_err_t tca9554_init(void);

/**
 * @brief Set a single EXIO pin HIGH or LOW
 *
 * @param pin   EXIO pin number (EXIO_PIN_1 through EXIO_PIN_8)
 * @param level true = HIGH, false = LOW
 * @return ESP_OK on success
 */
esp_err_t tca9554_set_pin(tca9554_pin_t pin, bool level);

/**
 * @brief Read the current output register state
 *
 * @param[out] state  8-bit output register value
 * @return ESP_OK on success
 */
esp_err_t tca9554_get_state(uint8_t *state);

/**
 * @brief Set all 8 output pins at once
 *
 * @param state  8-bit value (bit 0 = EXIO1, bit 7 = EXIO8)
 * @return ESP_OK on success
 */
esp_err_t tca9554_set_all(uint8_t state);

/**
 * @brief Check if TCA9554 has been initialized
 * @return true if initialized
 */
bool tca9554_is_initialized(void);
