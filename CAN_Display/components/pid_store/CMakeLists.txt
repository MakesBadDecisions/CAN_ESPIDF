# pid_store — PID selection persistence
# Lightweight: no external component deps, just ESP-IDF base

idf_component_register(
    SRCS
        "pid_store.c"
    INCLUDE_DIRS "."
    REQUIRES
        log
)
