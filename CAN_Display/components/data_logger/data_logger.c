/**
 * @file data_logger.c
 * @brief SD Card CSV Logging Implementation
 *
 * Mounts FAT filesystem on SD card using device-appropriate interface:
 *   - SPI mode (CrowPanel 4.3"): shared SPI2 bus with touch controller
 *   - SDMMC 1-bit mode (Waveshare 2.1"): dedicated SDMMC peripheral
 *
 * Creates sequential log files (log1.csv, log2.csv, ...) with structured
 * headers and PID data rows in HP Tuners Scanner compatible format.
 */

#include "data_logger.h"
#include "comm_link.h"
#include "system.h"
#include "device.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#if defined(SD_USE_SDMMC) && SD_USE_SDMMC
    #include "driver/sdmmc_host.h"
#else
    #include "driver/sdspi_host.h"
    #include "driver/spi_common.h"
#endif

#if defined(HAS_GPIO_EXPANDER) && HAS_GPIO_EXPANDER
    #include "tca9554.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>

static const char *TAG = "logger";

// ============================================================================
// Module State
// ============================================================================

typedef struct {
    // SD card
    sdmmc_card_t   *card;
    bool            sd_mounted;

    // Session
    logger_state_t  state;
    FILE           *file;
    uint32_t        file_number;        // Current file number (persisted in NVS)
    uint32_t        rows_written;
    uint32_t        bytes_written;
    uint32_t        start_tick;         // Session start tick (for offset calc)

    // Channel config for current session
    uint16_t        pids[LOGGER_MAX_CHANNELS];
    uint8_t         pid_count;

    // Write buffer
    char            buf[LOGGER_WRITE_BUF_SIZE];
    int             buf_pos;
    SemaphoreHandle_t mutex;
} logger_ctx_t;

static logger_ctx_t s_ctx = {0};

// ============================================================================
// NVS Helpers — persist file sequence number
// ============================================================================

static uint32_t nvs_load_file_number(void)
{
    nvs_handle_t h;
    uint32_t num = 0;
    if (nvs_open("logger", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, "filenum", &num);
        nvs_close(h);
    }
    return num;
}

static void nvs_save_file_number(uint32_t num)
{
    nvs_handle_t h;
    if (nvs_open("logger", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "filenum", num);
        nvs_commit(h);
        nvs_close(h);
    }
}

// ============================================================================
// Write Buffer
// ============================================================================

static void buf_flush(void)
{
    if (s_ctx.buf_pos > 0 && s_ctx.file) {
        size_t written = fwrite(s_ctx.buf, 1, s_ctx.buf_pos, s_ctx.file);
        if (written > 0) {
            s_ctx.bytes_written += written;
        }
        fflush(s_ctx.file);
        s_ctx.buf_pos = 0;
    }
}

static void buf_write(const char *data, int len)
{
    while (len > 0) {
        int space = LOGGER_WRITE_BUF_SIZE - s_ctx.buf_pos;
        int chunk = (len < space) ? len : space;
        memcpy(&s_ctx.buf[s_ctx.buf_pos], data, chunk);
        s_ctx.buf_pos += chunk;
        data += chunk;
        len -= chunk;

        if (s_ctx.buf_pos >= LOGGER_WRITE_BUF_SIZE) {
            buf_flush();
        }
    }
}

static void buf_puts(const char *str)
{
    buf_write(str, strlen(str));
}

// ============================================================================
// SD Card Mount
// ============================================================================

esp_err_t logger_init(void)
{
    if (s_ctx.sd_mounted) {
        return ESP_OK;
    }

    s_ctx.mutex = xSemaphoreCreateMutex();
    if (!s_ctx.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // SD card mount — device-specific interface
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 2,
        .allocation_unit_size = 16 * 1024,
    };

#if defined(SD_USE_SDMMC) && SD_USE_SDMMC
    // ---- SDMMC 1-bit Mode (Waveshare 2.1") ----

#if defined(HAS_GPIO_EXPANDER) && HAS_GPIO_EXPANDER
    // Enable SD card D3 via TCA9554 EXIO4
    tca9554_set_pin(EXIO_PIN_4, true);
    vTaskDelay(pdMS_TO_TICKS(10));
#endif

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;     // 1-bit mode
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdmmc_slot_config_t slot_cfg = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_cfg.width = 1;
    slot_cfg.clk = SD_SDMMC_CLK;
    slot_cfg.cmd = SD_SDMMC_CMD;
    slot_cfg.d0  = SD_SDMMC_D0;
    slot_cfg.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(
        LOGGER_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &s_ctx.card);

#else
    // ---- SPI Mode (CrowPanel) ----

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = TOUCH_SPI_HOST;  // SPI2_HOST — already initialized by touch

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS_PIN;
    slot_cfg.host_id = host.slot;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(
        LOGGER_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &s_ctx.card);

#endif // SD_USE_SDMMC

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "SD card mount failed (no FAT filesystem?)");
        } else {
            ESP_LOGE(TAG, "SD card init failed: %s", esp_err_to_name(ret));
        }
        s_ctx.state = LOGGER_STATE_ERROR;
        return ret;
    }

    s_ctx.sd_mounted = true;

    // Print card info
    sdmmc_card_print_info(stdout, s_ctx.card);

    // Create logs directory
    struct stat st;
    if (stat(LOGGER_LOG_DIR, &st) != 0) {
        mkdir(LOGGER_LOG_DIR, 0775);
        ESP_LOGI(TAG, "Created %s", LOGGER_LOG_DIR);
    }

    // Load file sequence number from NVS
    s_ctx.file_number = nvs_load_file_number();

    s_ctx.state = LOGGER_STATE_READY;
    ESP_LOGI(TAG, "SD card mounted, next file: log%lu.csv",
             (unsigned long)(s_ctx.file_number + 1));
    return ESP_OK;
}

// ============================================================================
// Write CSV Header (matches samplelog.csv format exactly)
// ============================================================================

static void write_header(const char *vin)
{
    char line[256];

    // File header (HP Tuners prefix required for scanner to open)
    buf_puts("HP Tuners CSV Log File\r\n");
    buf_puts("Version: 1.0\r\n");
    buf_puts("\r\n");

    // Log Information
    buf_puts("[Log Information]\r\n");
    // Use elapsed seconds since boot for creation time
    uint32_t secs = sys_time_ms() / 1000;
    uint32_t mins = secs / 60; secs %= 60;
    uint32_t hrs  = mins / 60; mins %= 60;
    snprintf(line, sizeof(line), "Creation Time: %lu:%02lu:%02lu\r\n",
             (unsigned long)hrs, (unsigned long)mins, (unsigned long)secs);
    buf_puts(line);
    buf_puts("Notes: \r\n");
    buf_puts("\r\n");

    // Location Information (empty)
    buf_puts("[Location Information]\r\n");
    buf_puts("\r\n");

    // Driver Information (empty)
    buf_puts("[Driver Information]\r\n");
    buf_puts("\r\n");

    // Vehicle Information
    buf_puts("[Vehicle Information]\r\n");
    if (vin && vin[0]) {
        snprintf(line, sizeof(line), "VIN: %s\r\n", vin);
        buf_puts(line);
    }
    buf_puts("\r\n");

    // Channel Information — 3 rows
    buf_puts("[Channel Information]\r\n");

    // Row 1: PID numbers (decimal), first column is 0 for time offset
    buf_puts("0");
    for (int i = 0; i < s_ctx.pid_count; i++) {
        snprintf(line, sizeof(line), ",%u", s_ctx.pids[i]);
        buf_puts(line);
    }
    buf_puts("\r\n");

    // Row 2: Channel names, first column is "Offset"
    buf_puts("Offset");
    for (int i = 0; i < s_ctx.pid_count; i++) {
        const char *name = comm_link_get_pid_name(s_ctx.pids[i]);
        snprintf(line, sizeof(line), ",%s", name ? name : "Unknown");
        buf_puts(line);
    }
    buf_puts("\r\n");

    // Row 3: Units, first column is "s"
    buf_puts("s");
    for (int i = 0; i < s_ctx.pid_count; i++) {
        const char *unit = comm_link_get_pid_unit_str(s_ctx.pids[i]);
        snprintf(line, sizeof(line), ",%s", unit ? unit : "?");
        buf_puts(line);
    }
    buf_puts("\r\n");

    buf_puts("\r\n");

    // Channel Data section header
    buf_puts("[Channel Data]\r\n");

    // Flush header to disk immediately
    buf_flush();
}

// ============================================================================
// Start / Stop Session
// ============================================================================

esp_err_t logger_start(const uint16_t *pids, uint8_t pid_count, const char *vin)
{
    if (!s_ctx.sd_mounted) {
        ESP_LOGE(TAG, "SD card not mounted");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.state == LOGGER_STATE_LOGGING) {
        ESP_LOGW(TAG, "Already logging, stopping previous session first");
        logger_stop();
    }
    if (!pids || pid_count == 0) {
        ESP_LOGE(TAG, "No PIDs specified for logging");
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    // Copy channel config
    s_ctx.pid_count = (pid_count > LOGGER_MAX_CHANNELS) ? LOGGER_MAX_CHANNELS : pid_count;
    memcpy(s_ctx.pids, pids, s_ctx.pid_count * sizeof(uint16_t));

    // Increment file number and persist
    s_ctx.file_number++;
    nvs_save_file_number(s_ctx.file_number);

    // Open file
    char path[64];
    snprintf(path, sizeof(path), "%s/log%lu.csv",
             LOGGER_LOG_DIR, (unsigned long)s_ctx.file_number);

    s_ctx.file = fopen(path, "w");
    if (!s_ctx.file) {
        ESP_LOGE(TAG, "Failed to create %s", path);
        xSemaphoreGive(s_ctx.mutex);
        s_ctx.state = LOGGER_STATE_ERROR;
        return ESP_FAIL;
    }

    s_ctx.rows_written = 0;
    s_ctx.bytes_written = 0;
    s_ctx.buf_pos = 0;
    s_ctx.start_tick = xTaskGetTickCount();

    // Write header sections
    write_header(vin);

    s_ctx.state = LOGGER_STATE_LOGGING;

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "Logging started: %s (%d channels)", path, s_ctx.pid_count);
    return ESP_OK;
}

void logger_stop(void)
{
    if (s_ctx.state != LOGGER_STATE_LOGGING) {
        return;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    buf_flush();

    if (s_ctx.file) {
        fclose(s_ctx.file);
        s_ctx.file = NULL;
    }

    ESP_LOGI(TAG, "Logging stopped: %lu rows, %lu bytes written",
             (unsigned long)s_ctx.rows_written,
             (unsigned long)s_ctx.bytes_written);

    s_ctx.state = s_ctx.sd_mounted ? LOGGER_STATE_READY : LOGGER_STATE_UNINIT;

    xSemaphoreGive(s_ctx.mutex);
}

// ============================================================================
// Log One Data Row
// ============================================================================

void logger_log_row(void)
{
    if (s_ctx.state != LOGGER_STATE_LOGGING) {
        return;
    }

    if (xSemaphoreTake(s_ctx.mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;  // Skip this row rather than block
    }

    // Calculate time offset in seconds from session start
    uint32_t elapsed_ticks = xTaskGetTickCount() - s_ctx.start_tick;
    float offset_s = (float)elapsed_ticks / (float)configTICK_RATE_HZ;

    char line[32];
    // Write offset (3 decimal places to match sample format)
    snprintf(line, sizeof(line), "%.3f", offset_s);
    buf_puts(line);

    // Write each channel value
    for (int i = 0; i < s_ctx.pid_count; i++) {
        pid_cache_entry_t val;
        if (comm_link_get_pid(s_ctx.pids[i], &val) && !val.stale) {
            snprintf(line, sizeof(line), ",%.10g", (double)val.value);
        } else {
            // Missing value — empty field
            line[0] = ',';
            line[1] = '\0';
        }
        buf_puts(line);
    }

    buf_puts("\r\n");
    s_ctx.rows_written++;

    xSemaphoreGive(s_ctx.mutex);
}

// ============================================================================
// Status Queries
// ============================================================================

logger_status_t logger_get_status(void)
{
    logger_status_t st = {
        .state = s_ctx.state,
        .rows_written = s_ctx.rows_written,
        .bytes_written = s_ctx.bytes_written,
        .file_number = s_ctx.file_number,
        .sd_total_bytes = 0,
        .sd_free_bytes = 0,
    };

    if (s_ctx.sd_mounted && s_ctx.card) {
        // Approximate from card info
        uint64_t total = (uint64_t)s_ctx.card->csd.capacity * s_ctx.card->csd.sector_size;
        st.sd_total_bytes = total;
        // Free space requires FATFS query — skip for now, expensive
    }

    return st;
}

logger_state_t logger_get_state(void)
{
    return s_ctx.state;
}

bool logger_is_sd_mounted(void)
{
    return s_ctx.sd_mounted;
}

