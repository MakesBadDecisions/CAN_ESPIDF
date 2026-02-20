# UI Component - SquareLine Studio generated UI with event handlers
# Depends on lvgl for rendering, gauge_engine for PID data management

idf_component_register(
    SRCS 
        "ui.c"
        "ui_helpers.c"
        "ui_events.c"
        "components/ui_comp_hook.c"
        "screens/ui_Screen1.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        lvgl
        comm_link
        gauge_engine
        display_driver
        log
)
