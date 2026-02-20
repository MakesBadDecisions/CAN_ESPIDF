# pid_db - PID Database & Unit Conversions

idf_component_register(
    SRCS
        "pid_db.c"
        "pid_tables.c"
    INCLUDE_DIRS "."
    REQUIRES system pid_types
)
