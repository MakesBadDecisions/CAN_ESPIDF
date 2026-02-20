# touch_driver - Touch Input HAL
# Supports: XPT2046 (SPI, resistive), CST820 (I2C, capacitive)

idf_component_register(
    SRCS "touch_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer devices lvgl nvs_flash display_driver i2c_bus tca9554
)
