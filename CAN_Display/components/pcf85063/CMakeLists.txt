# pcf85063 - PCF85063A RTC Driver

idf_component_register(
    SRCS "pcf85063.c"
    INCLUDE_DIRS "."
    REQUIRES driver devices i2c_bus nvs_flash log
)
