# Main application component
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        system
        sys_mgr
        can_driver
        pid_db
        obd2
        poll_engine
        diagnostics
        comm_link
)
