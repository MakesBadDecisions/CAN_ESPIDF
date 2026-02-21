# UI Component - SquareLine Studio generated UI with event handlers
# Depends on lvgl for rendering, gauge_engine for PID data management

idf_component_register(
    SRCS
        "ui.c"
        "ui_helpers.c"
        "ui_events.c"
        "ui_Screen1.c"
        "ui_Screen2.c"
        "ui_comp_hook.c"
    INCLUDE_DIRS
        "."
    REQUIRES 
        lvgl
        comm_link
        gauge_engine
        data_logger
        display_driver
        imu_display
        log
)
