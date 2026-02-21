# IMU Display Component - SquareLine-safe
# Lives outside ui/ so SquareLine exports don't overwrite it.
# Renders G-force / tilt bubble inside gauge slot panels.

idf_component_register(
    SRCS
        "imu_display.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        lvgl
        qmi8658
        devices
        gauge_engine
        log
)
