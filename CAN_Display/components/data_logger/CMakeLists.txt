# data_logger - SD Card CSV Logging
# Sequential log files with HP Tuners CSV format
# Supports: SPI SD (CrowPanel), SDMMC 1-bit (Waveshare)

idf_component_register(
    SRCS "data_logger.c"
    INCLUDE_DIRS "."
    REQUIRES fatfs sdmmc comm_link system devices driver nvs_flash tca9554
)
