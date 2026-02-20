# UI Component - SquareLine Studio generated UI with event handlers
# Depends on lvgl for rendering, comm_link for vehicle communication

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
        display_driver
        log
)
