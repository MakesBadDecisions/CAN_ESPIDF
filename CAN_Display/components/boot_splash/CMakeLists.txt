idf_component_register(
    SRCS "boot_splash.c"
    INCLUDE_DIRS "."
    REQUIRES
        lvgl
        display_driver
        devices
        nvs_flash
)
