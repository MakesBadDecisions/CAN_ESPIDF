# data_logger - SD Card CSV Logging
# HPTuners-compatible CSV format

idf_component_register(
    SRCS "data_logger.c"
    INCLUDE_DIRS "."
    REQUIRES fatfs sdmmc comm_link system
)
