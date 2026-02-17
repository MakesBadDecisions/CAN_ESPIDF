# obd2 - OBD-II Protocol Stack
# Request/response, ISO-TP multi-frame

idf_component_register(
    SRCS "obd2.c"
    INCLUDE_DIRS "."
    REQUIRES can_driver system
)
