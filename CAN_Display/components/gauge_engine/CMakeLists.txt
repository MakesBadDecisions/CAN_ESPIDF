# gauge_engine - Gauge Data Manager
# Per-gauge PID assignment, unit conversion, poll list aggregation
# No LVGL dependency -- UI reads formatted values from here

idf_component_register(
    SRCS "gauge_engine.c"
    INCLUDE_DIRS "."
    REQUIRES comm_link pid_types nvs_flash devices qmi8658
)
