# can_driver - CAN Hardware Abstraction Layer
# MCP2515 (Phase 1), TWAI, MCP2518FD backends

idf_component_register(
    SRCS "can_driver.c" "mcp2515.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer log system devices
)
