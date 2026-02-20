/**
 * @file tca9554.c
 * @brief TCA9554 I2C GPIO Expander Driver Implementation
 */

#include "tca9554.h"
#include "device.h"
#include "i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "tca9554";

// TCA9554 register addresses
#define REG_INPUT       0x00    // Input port register (read-only)
#define REG_OUTPUT      0x01    // Output port register
#define REG_POLARITY    0x02    // Polarity inversion register
#define REG_CONFIG      0x03    // Configuration register (0=output, 1=input)

#if defined(HAS_GPIO_EXPANDER) && HAS_GPIO_EXPANDER

static bool s_initialized = false;
static uint8_t s_output_state = 0x00;   // Shadow register for output port

// ============================================================================
// I2C Helpers
// ============================================================================

static esp_err_t tca9554_write_reg(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9554_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_get_port(), cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t tca9554_read_reg(uint8_t reg, uint8_t *value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9554_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9554_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_bus_get_port(), cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t tca9554_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (!i2c_bus_is_initialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized — call i2c_bus_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    // Configure all pins as outputs (0x00 = all output)
    esp_err_t ret = tca9554_write_reg(REG_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure TCA9554: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set all outputs LOW initially
    s_output_state = 0x00;
    ret = tca9554_write_reg(REG_OUTPUT, s_output_state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set outputs: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "TCA9554 initialized (addr=0x%02X, all outputs LOW)", TCA9554_I2C_ADDR);
    return ESP_OK;
}

esp_err_t tca9554_set_pin(tca9554_pin_t pin, bool level)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (pin > EXIO_PIN_8) {
        return ESP_ERR_INVALID_ARG;
    }

    if (level) {
        s_output_state |= (1 << pin);
    } else {
        s_output_state &= ~(1 << pin);
    }

    return tca9554_write_reg(REG_OUTPUT, s_output_state);
}

esp_err_t tca9554_get_state(uint8_t *state)
{
    if (!state) return ESP_ERR_INVALID_ARG;
    *state = s_output_state;
    return ESP_OK;
}

esp_err_t tca9554_set_all(uint8_t state)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_output_state = state;
    return tca9554_write_reg(REG_OUTPUT, s_output_state);
}

bool tca9554_is_initialized(void)
{
    return s_initialized;
}

#else

// ============================================================================
// Stub implementation for devices without TCA9554
// ============================================================================

esp_err_t tca9554_init(void) { return ESP_OK; }
esp_err_t tca9554_set_pin(tca9554_pin_t pin, bool level) { return ESP_OK; }
esp_err_t tca9554_get_state(uint8_t *state) { if (state) *state = 0; return ESP_OK; }
esp_err_t tca9554_set_all(uint8_t state) { return ESP_OK; }
bool tca9554_is_initialized(void) { return false; }

#endif // HAS_GPIO_EXPANDER
