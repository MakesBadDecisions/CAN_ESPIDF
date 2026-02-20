# comm_link - UART Communication Link to Display Node
# Bidirectional UART with frame TX/RX, heartbeat, PID batching

idf_component_register(
    SRCS "comm_link.c"
    INCLUDE_DIRS "."
    REQUIRES driver system comm_protocol devices can_driver
)
