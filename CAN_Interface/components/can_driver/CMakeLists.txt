# can_driver - CAN Hardware Abstraction Layer
# MCP2515, MCP2518FD, TWAI backends

idf_component_register(
    SRCS "can_driver.c" "mcp2515.c" "mcp2518fd.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer log system devices
)
