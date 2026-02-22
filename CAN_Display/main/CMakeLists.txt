idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        comm_link
        display_driver
        touch_driver
        gauge_engine
        pid_store
        data_logger
        boot_splash
        wifi_manager
        system
        lvgl
        ui
        devices
        i2c_bus
        tca9554
        qmi8658
        pcf85063
)
