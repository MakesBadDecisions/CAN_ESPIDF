# data_logger - SD Card CSV Logging

## Purpose

Logs PID data to SD card in CSV format compatible with HPTuners and other data analysis tools. Manages log sessions, file rotation, and buffered writes to prevent data loss.

## CSV Format

### HPTuners-Compatible Header
```csv
Time (s),Engine RPM (rpm),Vehicle Speed (mph),Coolant Temp (F),Throttle (%),...
0.000,850.0,0.0,185.2,12.5,...
0.050,855.0,0.0,185.2,12.8,...
0.100,860.0,0.5,185.3,15.2,...
```

- First row: column headers with PID name and unit in parentheses
- Subsequent rows: timestamp (seconds from session start) + values
- Comma-separated, one row per sample period
- Missing values: empty field (consecutive commas)

### File Naming
```
/sdcard/logs/YYYYMMDD_HHMMSS_VIN.csv
/sdcard/logs/20260214_153022_1G1YY22G965.csv
```

If VIN is not available: `YYYYMMDD_HHMMSS_session.csv`

## Buffering Strategy

SD card writes are slow and bursty. The logger uses a double-buffer approach:

```
PID Store ──> Buffer A (filling) ──> SD Card Write
                                      ↕ (swap)
              Buffer B (draining) ──> SD Card Write
```

- **Buffer size:** 4KB each (configurable)
- **Flush trigger:** Buffer full OR 1-second timeout (whichever first)
- **Sample rate:** Configurable (default: 20Hz / 50ms per row)
- **No data loss:** If SD write stalls, buffer swap defers until complete

## Session Management

```c
// Start a new log session (creates new CSV file)
esp_err_t logger_start_session(const char *vin, const uint16_t *pids, size_t pid_count);

// Stop current session (flush buffers, close file)
esp_err_t logger_stop_session(void);

// Check session status
logger_session_info_t logger_get_session_info(void);
```

## File Management

- **Max file size:** 50MB (configurable, then auto-rotate to new file)
- **Storage check:** Warn at 90% SD card usage, stop logging at 95%
- **File listing:** API to enumerate log files for WiFi download

## Files

```
data_logger/
├── README.md               # This file
├── CMakeLists.txt
├── include/
│   ├── data_logger.h        # Public API
│   └── csv_writer.h         # CSV formatting utilities
└── src/
    ├── data_logger.c         # Session management, buffer coordination
    ├── csv_writer.c          # Header generation, row formatting
    └── sd_card.c             # SD card init, mount, file operations
```

## Dependencies

- `sdmmc` or `spi_master` (SD card interface)
- `fatfs` (FAT filesystem via ESP-IDF VFS)
- `comm_link/pid_store` (current PID values to log)
- `shared/pid_types` (PID names and units for CSV headers)
- `system` (logging, timing)
