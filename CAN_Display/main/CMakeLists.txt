idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        comm_link
        display_driver
        touch_driver
        gauge_engine
        data_logger
        wifi_manager
        system
        lvgl
        ui
        devices
        i2c_bus
        tca9554
        qmi8658
)
