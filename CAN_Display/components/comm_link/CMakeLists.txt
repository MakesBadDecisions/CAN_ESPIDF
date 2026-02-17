# comm_link - UART Communication (Display Node RX)
# PID store, UART parsing, config commands

idf_component_register(
    SRCS "comm_link.c"
    INCLUDE_DIRS "."
    REQUIRES driver system comm_protocol devices freertos
)
