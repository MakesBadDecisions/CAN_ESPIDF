# system - Core System Infrastructure
# Logging, NVS config, timing, task helpers

idf_component_register(
    SRCS "system.c"
    INCLUDE_DIRS "."
    REQUIRES log nvs_flash esp_timer freertos esp_system spi_flash esp_psram
)
