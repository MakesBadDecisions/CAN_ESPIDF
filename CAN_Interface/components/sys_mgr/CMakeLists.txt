# sys_mgr - System Manager
# Runtime coordination, state machine, health monitoring

idf_component_register(
    SRCS "sys_mgr.c"
    INCLUDE_DIRS "."
    REQUIRES freertos system esp_timer can_driver
)
