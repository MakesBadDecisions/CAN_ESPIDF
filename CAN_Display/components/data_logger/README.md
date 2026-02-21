# data_logger - SD Card CSV Logging

## Purpose

Logs PID data to SD card in HP Tuners-compatible CSV format. Automatically starts logging when polling begins and stops when polling ends. Raw CAN values are logged directly — no unit conversion. When built with `HAS_IMU`, two additional columns (Lateral G, Longitudinal G) are automatically appended after CAN PID columns.

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
- **Channel Information**: 3 rows — PID numbers (decimal), names from pid_db, units from pid_db. When HAS_IMU is defined, appends PID 65280 (Lateral G) and 65281 (Longitudinal G) with "G" units.
- **Channel Data**: Time offset (seconds, 3dp) + raw values per poll cycle. IMU columns appended with `%.4f` precision.

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

Board-specific SD card interface selected at compile time via device header
defines (`SD_USE_SDMMC`).

### CrowPanel — SPI (Default)

Shares SPI2_HOST bus with XPT2046 touch controller (separate CS pins):
- **MOSI**: GPIO 11 (shared with touch)
- **MISO**: GPIO 13 (shared with touch)
- **SCLK**: GPIO 12 (shared with touch)
- **CS**: GPIO 10 (SD card dedicated)
- **Touch CS**: GPIO 0 (separate)

SPI bus initialized by `touch_driver` with DMA enabled (`SPI_DMA_CH_AUTO`).
SD card added as second device on the same host. Filesystem: FAT via
`esp_vfs_fat_sdspi_mount()`, mounted at `/sdcard`.

### Waveshare 2.1" — SDMMC 1-bit

Dedicated SD card bus (no SPI sharing):
- **CLK**: GPIO 2
- **CMD**: GPIO 1
- **D0**: GPIO 42
- **D3**: Directly connected (active high)
- **TCA9554 EXIO4**: Directly connected (active high)

SDMMC 1-bit mode via `esp_vfs_fat_sdmmc_mount()`. Higher throughput than SPI.
Signal pins defined in the device header (`waveshare_esp32s3_touch_lcd_2_1.h`).

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
- `devices` — SD pin definitions, `SD_USE_SDMMC` dispatch
- `driver` — GPIO/SPI driver
- `nvs_flash` — File sequence number persistence
- `tca9554` — SD D3 enable on Waveshare (via EXIO4)
- `qmi8658` — IMU orientation data for G-load columns (optional, HAS_IMU guarded)
