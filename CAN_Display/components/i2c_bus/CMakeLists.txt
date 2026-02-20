# i2c_bus - Shared I2C Bus Manager

idf_component_register(
    SRCS "i2c_bus.c"
    INCLUDE_DIRS "."
    REQUIRES driver devices log
)
