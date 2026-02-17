# wifi_manager - WiFi AP & Config Server
# Log download, gauge config, CAN node proxy

idf_component_register(
    SRCS "wifi_manager.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_http_server data_logger gauge_engine system
)
