# gauge_engine - Gauge Rendering & Layout
# Numeric, bar, sweep gauges with alerts

idf_component_register(
    SRCS "gauge_engine.c"
    INCLUDE_DIRS "."
    REQUIRES display_driver comm_link system
)
