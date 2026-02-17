# poll_engine - Smart Polling Scheduler
# Priority-based polling with adaptive intervals

idf_component_register(
    SRCS "poll_engine.c"
    INCLUDE_DIRS "."
    REQUIRES obd2 pid_db system
)
