# wifi_manager - WiFi AP, Config Hub & Log Download Server

## Purpose

Runs the **only WiFi access point** in the CAN_ESPIDF system. The Display
Node is the user-facing hub -- users connect from a phone or laptop to browse
logs, download CSV files, configure gauge layouts, and manage CAN Interface
Node settings (proxied over the UART link).

## Features

- **WiFi AP:** SSID "CAN_ESPIDF" (configurable), WPA2 or open
- **Log browser:** List all CSV files on SD card with size and date
- **File download:** Direct HTTP download of log files
- **Gauge config:** Web UI to configure layouts, PID assignments, alert thresholds
- **CAN node config:** Proxy PID poll list, CAN baud, scan commands to CAN node over UART
- **System status:** Both nodes' health, UART link state, SD card usage

## Endpoints

| Endpoint | Purpose |
|----------|---------|
| `/` | Dashboard - system status, connection health, SD usage |
| `/logs` | Browse log files on SD card |
| `/logs/:filename` | Download a specific log file |
| `/gauges` | Configure gauge layouts and PID assignments |
| `/settings` | System settings (WiFi SSID, display brightness, etc.) |
| `/can` | CAN Interface Node config (PID poll list, baud, scan trigger) |
| `/api/logs` | JSON list of log files |
| `/api/gauges` | JSON gauge layout config (GET/POST) |
| `/api/can/status` | JSON CAN node status (proxied over UART) |
| `/api/can/config` | JSON CAN node config (GET/POST, proxied over UART) |

## CAN Node Configuration Proxy

When the user changes CAN node settings through the web UI, the wifi_manager
sends `CONFIG_CMD` messages to the CAN Interface Node over the UART link
via the `comm_link` component, and waits for `CONFIG_RESP` acknowledgments.

This allows full system configuration from a single WiFi access point.

## Files

```
wifi_manager/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   └── wifi_manager.h       # Public API
└── src/
    ├── wifi_manager.c        # AP init, event handling
    ├── http_server.c         # Route registration
    ├── log_browser.c         # Log file listing and download handlers
    ├── gauge_config.c        # Gauge layout config handlers
    └── can_proxy.c           # CAN node config proxy (UART relay)
```

## Dependencies

- `esp_wifi` (AP mode)
- `esp_http_server`
- `data_logger` (SD card file access)
- `gauge_engine` (layout read/write)
- `comm_link` (UART TX for CAN node config commands)
- `system` (logging, NVS)
