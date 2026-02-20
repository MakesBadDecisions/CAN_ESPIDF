/**
 * @file i2c_bus.c
 * @brief Shared I2C Bus Manager Implementation
 */

#include "i2c_bus.h"
#include "device.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";
static bool s_initialized = false;

esp_err_t i2c_bus_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

#if defined(I2C_BUS_SDA) && defined(I2C_BUS_SCL)
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_BUS_SDA,
        .scl_io_num = I2C_BUS_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_BUS_FREQ,
    };

    esp_err_t ret = i2c_param_config(I2C_BUS_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_BUS_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "I2C bus initialized (SDA=%d, SCL=%d, %lu Hz)",
             I2C_BUS_SDA, I2C_BUS_SCL, (unsigned long)I2C_BUS_FREQ);
    return ESP_OK;
#else
    ESP_LOGI(TAG, "No I2C bus defined for this device, skipping init");
    return ESP_OK;
#endif
}

bool i2c_bus_is_initialized(void)
{
    return s_initialized;
}

i2c_port_t i2c_bus_get_port(void)
{
#if defined(I2C_BUS_PORT)
    return I2C_BUS_PORT;
#else
    return I2C_NUM_0;
#endif
}
