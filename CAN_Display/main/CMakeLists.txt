idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        comm_link
        display_driver
        gauge_engine
        data_logger
        wifi_manager
        system
)
