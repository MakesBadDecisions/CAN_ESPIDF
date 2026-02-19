# display_driver - Display Hardware Abstraction Layer
# RGB parallel LCD, framebuffer, touch

idf_component_register(
    SRCS "display_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd esp_timer devices lvgl log
)
