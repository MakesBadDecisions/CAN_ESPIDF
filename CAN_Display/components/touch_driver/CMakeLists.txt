idf_component_register(
    SRCS "touch_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer devices lvgl nvs_flash display_driver
)
