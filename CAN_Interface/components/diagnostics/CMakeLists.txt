# diagnostics - Vehicle Diagnostics
# VIN, DTCs, ECU discovery, supported PIDs

idf_component_register(
    SRCS "diagnostics.c"
    INCLUDE_DIRS "."
    REQUIRES obd2 can_driver system comm_link
)
