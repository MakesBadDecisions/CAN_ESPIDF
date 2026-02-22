# wifi_manager - WiFi AP & Config Server
# SoftAP, QR code display, HTTP REST API, config portal

idf_component_register(
    SRCS
        "wifi_manager.c"
        "wifi_ap.c"
        "qr_screen.c"
        "http_server.c"
        "status_handler.c"
        "log_handler.c"
        "config_handler.c"
        "pid_handler.c"
        "portal_handler.c"
        "time_handler.c"
        "scan_handler.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_http_server
        esp_event
        esp_netif
        nvs_flash
        data_logger
        gauge_engine
        pid_store
        boot_splash
        comm_link
        display_driver
        lvgl
        system
        pcf85063
        log
)
