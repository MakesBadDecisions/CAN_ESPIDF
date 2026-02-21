# display_driver - Display Hardware Abstraction Layer
# RGB parallel LCD, framebuffer, LVGL integration
# Supports: CrowPanel (direct GPIO), Waveshare 2.1" (ST7701S SPI + TCA9554)

idf_component_register(
    SRCS "display_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd esp_timer devices lvgl log tca9554 nvs_flash
)
