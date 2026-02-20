# tca9554 - TCA9554 I2C GPIO Expander Driver

idf_component_register(
    SRCS "tca9554.c"
    INCLUDE_DIRS "."
    REQUIRES driver devices i2c_bus log
)
