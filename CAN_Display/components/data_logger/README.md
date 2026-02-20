# data_logger - SD Card CSV Logging

## Purpose

Logs PID data to SD card in HP Tuners-compatible CSV format. Automatically starts logging when polling begins and stops when polling ends. Raw CAN values are logged directly — no unit conversion.

## Status: WORKING

Verified on-vehicle with Ford F-150 (VIN: 1FTLR4FEXBPA98994). CSV files open directly in HP Tuners Scanner software.

## CSV Format

Files follow the HP Tuners structured CSV format with bracketed section headers:

```csv
HP Tuners CSV Log File
Version: 1.0

[Log Information]
Creation Time: 0:00:33
Notes: 

[Location Information]

[Driver Information]

[Vehicle Information]
VIN: 1FTLR4FEXBPA98994

[Channel Information]
0,5,17,13,15
Offset,Engine coolant temperature,Throttle position,Vehicle speed,Intake air temperature
s,°C,%,km/h,°C

[Channel Data]
0.102,120,,,
0.205,120,88.23529053,51,
0.315,120,88.23529053,51,167
```

### Header Sections
- **Log Information**: Creation time (elapsed since boot)
- **Vehicle Information**: VIN from comm_link (stored in NVS)
- **Channel Information**: 3 rows — PID numbers (decimal), names from pid_db, units from pid_db
- **Channel Data**: Time offset (seconds, 3dp) + raw values per poll cycle

### Data Rows
- First column: time offset from session start (seconds, millisecond resolution)
- Subsequent columns: raw CAN values (%.10g format for full precision)
- Missing/stale values: empty field (consecutive commas)
- Line endings: `\r\n`

## File Naming

Sequential naming persisted across reboots via NVS:
```
/sdcard/logs/log1.csv
/sdcard/logs/log2.csv
/sdcard/logs/log3.csv
```

File sequence number stored in NVS namespace `"logger"`, key `"filenum"`.

## SD Card Interface

Shares SPI2_HOST bus with XPT2046 touch controller (separate CS pins):
- **MOSI**: GPIO 11 (shared with touch)
- **MISO**: GPIO 13 (shared with touch)
- **SCLK**: GPIO 12 (shared with touch)
- **CS**: GPIO 10 (SD card dedicated)
- **Touch CS**: GPIO 0 (separate)

SPI bus initialized by `touch_driver` with DMA enabled (`SPI_DMA_CH_AUTO`) and `max_transfer_sz=4096` for SD card compatibility. SD card added as second device via `spi_bus_add_device` equivalent (sdspi host).

Filesystem: FAT via `esp_vfs_fat_sdspi_mount()`, mounted at `/sdcard`.

## Buffering Strategy

Single 4KB write buffer, flushed when full or on session stop:

```
comm_link PID cache ──> buf_puts() ──> 4KB buffer ──> fwrite() to SD
                                            │
                                    (flush on full or stop)
```

- **Buffer size**: 4096 bytes (LOGGER_WRITE_BUF_SIZE)
- **Flush trigger**: Buffer full (automatic) or `logger_stop()` (explicit)
- **Non-blocking**: `logger_log_row()` uses 5ms mutex timeout — skips row rather than blocking LVGL

## Session Management

Sessions are controlled automatically by `ui_events.c`:

```c
// Auto-start: when user presses "Connect" to start polling
logger_start(pids, pid_count, vin);

// Log row: called from gauge_update_cb() on every new PID data arrival
logger_log_row();

// Auto-stop: when user presses "Connect" again to stop polling
logger_stop();
```

## API

```c
esp_err_t   logger_init(void);                                          // Mount SD, create /sdcard/logs
esp_err_t   logger_start(const uint16_t *pids, uint8_t pid_count,       // New session
                         const char *vin);
void        logger_stop(void);                                          // Flush + close
void        logger_log_row(void);                                       // Append one data row
logger_status_t logger_get_status(void);                                // State, rows, bytes, SD info
logger_state_t  logger_get_state(void);                                 // UNINIT/READY/LOGGING/ERROR
bool        logger_is_sd_mounted(void);                                 // SD card present?
```

## Files

```
data_logger/
├── README.md           # This file
├── CMakeLists.txt      # Component build config
├── data_logger.h       # Public API, types, constants
└── data_logger.c       # Full implementation (~430 lines)
```

## Dependencies

- `fatfs` — FAT filesystem for SD card
- `sdmmc` — SD/MMC card driver
- `comm_link` — PID name/unit lookup, PID value cache
- `system` — `sys_time_ms()` for creation timestamp
- `devices` — `SD_CS_PIN`, `TOUCH_SPI_HOST` pin definitions
- `driver` — GPIO/SPI driver
- `nvs_flash` — File sequence number persistence

- `sdmmc` or `spi_master` (SD card interface)
- `fatfs` (FAT filesystem via ESP-IDF VFS)
- `comm_link/pid_store` (current PID values to log)
- `shared/pid_types` (PID names and units for CSV headers)
- `system` (logging, timing)
