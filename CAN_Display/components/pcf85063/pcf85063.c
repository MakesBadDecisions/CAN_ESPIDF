/**
 * @file pcf85063.c
 * @brief PCF85063A RTC Driver Implementation
 *
 * Uses the shared I2C bus (i2c_bus.h) with the legacy ESP-IDF I2C API.
 * Same pattern as qmi8658.c and tca9554.c.
 *
 * RTC stores UTC. Timezone offset is persisted in NVS namespace "rtc".
 * Boards without HAS_RTC get stub implementations.
 */

#include "pcf85063.h"
#include "device.h"
#include "esp_log.h"

static const char *TAG = "pcf85063";

#if defined(HAS_RTC) && HAS_RTC

#include "i2c_bus.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <sys/time.h>

// ============================================================================
// PCF85063A Register Map
// ============================================================================

#define REG_CONTROL_1       0x00    // Control/status register 1
#define REG_CONTROL_2       0x01    // Control/status register 2
#define REG_OFFSET          0x02    // Offset register
#define REG_RAM_BYTE        0x03    // RAM byte
#define REG_SECONDS         0x04    // Seconds (BCD) + OS flag (bit 7)
#define REG_MINUTES         0x05    // Minutes (BCD)
#define REG_HOURS           0x06    // Hours   (BCD, 24h mode)
#define REG_DAYS            0x07    // Days    (BCD, 1-31)
#define REG_WEEKDAYS        0x08    // Weekdays (0-6)
#define REG_MONTHS          0x09    // Months  (BCD, 1-12)
#define REG_YEARS           0x0A    // Years   (BCD, 0-99)

// CONTROL_1 bits
#define CTRL1_CAP_SEL       (1 << 0)   // Internal oscillator capacitor selection
#define CTRL1_STOP          (1 << 5)   // Stop bit: 1=clock stopped
#define CTRL1_SR            (1 << 6)   // Software reset (auto-clears)
#define CTRL1_EXT_TEST      (1 << 7)   // External clock test mode

// Seconds register
#define SEC_OS_FLAG         (1 << 7)   // Oscillator-stop flag (1=integrity not guaranteed)

// NVS
#define NVS_NAMESPACE       "rtc"
#define NVS_KEY_TZ_OFFSET   "tz_off"

// ============================================================================
// State
// ============================================================================

static bool s_initialized = false;
static int16_t s_tz_offset_min = 0;    // Cached timezone offset (minutes from UTC)

// ============================================================================
// I2C Helpers (same pattern as QMI8658)
// ============================================================================

static esp_err_t rtc_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(
        i2c_bus_get_port(), PCF85063_I2C_ADDR,
        buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t rtc_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(
        i2c_bus_get_port(), PCF85063_I2C_ADDR,
        &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

static esp_err_t rtc_write_regs(uint8_t start_reg, const uint8_t *data, size_t len)
{
    // Build buffer: [register_address, data0, data1, ...]
    uint8_t buf[16];
    if (len + 1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    buf[0] = start_reg;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(
        i2c_bus_get_port(), PCF85063_I2C_ADDR,
        buf, len + 1, pdMS_TO_TICKS(100));
}

static esp_err_t rtc_read_regs(uint8_t start_reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        i2c_bus_get_port(), PCF85063_I2C_ADDR,
        &start_reg, 1, data, len, pdMS_TO_TICKS(100));
}

// ============================================================================
// BCD Helpers
// ============================================================================

static inline uint8_t dec_to_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

static inline uint8_t bcd_to_dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t pcf85063_init(void)
{
    if (s_initialized) return ESP_OK;

    if (!i2c_bus_is_initialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Verify communication by reading Control_1 register
    uint8_t ctrl1 = 0;
    esp_err_t ret = rtc_read_reg(REG_CONTROL_1, &ctrl1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Control_1 (I2C addr 0x%02X): %s",
                 PCF85063_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "PCF85063 detected (Control_1=0x%02X)", ctrl1);

    // Clear stop bit if set (ensure clock is running)
    if (ctrl1 & CTRL1_STOP) {
        ESP_LOGW(TAG, "Clock was stopped — starting");
        ctrl1 &= ~CTRL1_STOP;
        ret = rtc_write_reg(REG_CONTROL_1, ctrl1);
        if (ret != ESP_OK) return ret;
    }

    // Check oscillator-stop flag in seconds register
    uint8_t sec = 0;
    ret = rtc_read_reg(REG_SECONDS, &sec);
    if (ret != ESP_OK) return ret;

    if (sec & SEC_OS_FLAG) {
        ESP_LOGW(TAG, "Oscillator-stop flag set — time integrity not guaranteed");
        // Clear OS flag by writing seconds back (without bit 7)
        ret = rtc_write_reg(REG_SECONDS, sec & 0x7F);
        if (ret != ESP_OK) return ret;
    }

    // Load timezone offset from NVS
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        int16_t tz = 0;
        if (nvs_get_i16(nvs, NVS_KEY_TZ_OFFSET, &tz) == ESP_OK) {
            s_tz_offset_min = tz;
            ESP_LOGI(TAG, "Timezone offset: %+d min (UTC%+d:%02d)",
                     tz, tz / 60, abs(tz) % 60);
        }
        nvs_close(nvs);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "PCF85063 RTC initialized");
    return ESP_OK;
}

bool pcf85063_is_initialized(void)
{
    return s_initialized;
}

// ============================================================================
// Time Get/Set
// ============================================================================

esp_err_t pcf85063_set_time(const struct tm *utc)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!utc) return ESP_ERR_INVALID_ARG;

    // Encode to BCD and write all 7 time registers in a burst
    uint8_t regs[7];
    regs[0] = dec_to_bcd(utc->tm_sec)  & 0x7F;    // Seconds (clear OS flag)
    regs[1] = dec_to_bcd(utc->tm_min)  & 0x7F;    // Minutes
    regs[2] = dec_to_bcd(utc->tm_hour) & 0x3F;    // Hours (24h)
    regs[3] = dec_to_bcd(utc->tm_mday) & 0x3F;    // Days
    regs[4] = (uint8_t)(utc->tm_wday & 0x07);     // Weekday (0=Sun)
    regs[5] = dec_to_bcd(utc->tm_mon + 1) & 0x1F; // Months (struct tm is 0-based)
    regs[6] = dec_to_bcd(utc->tm_year % 100);     // Years (0-99, relative to 1900 in tm → 2000 in RTC)

    esp_err_t ret = rtc_write_regs(REG_SECONDS, regs, sizeof(regs));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write time registers: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "RTC set to %04d-%02d-%02d %02d:%02d:%02d UTC",
             utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
             utc->tm_hour, utc->tm_min, utc->tm_sec);
    return ESP_OK;
}

esp_err_t pcf85063_set_epoch(time_t epoch_sec)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    // Set ESP32 system time
    struct timeval tv = { .tv_sec = epoch_sec, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    // Convert to broken-down UTC and write to RTC
    struct tm utc;
    gmtime_r(&epoch_sec, &utc);

    esp_err_t ret = pcf85063_set_time(&utc);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "System clock + RTC set to epoch %lld", (long long)epoch_sec);
    }
    return ret;
}

esp_err_t pcf85063_get_time(struct tm *utc)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (!utc) return ESP_ERR_INVALID_ARG;

    // Burst-read all 7 time registers
    uint8_t regs[7];
    esp_err_t ret = rtc_read_regs(REG_SECONDS, regs, sizeof(regs));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read time registers: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check oscillator-stop flag
    if (regs[0] & SEC_OS_FLAG) {
        ESP_LOGW(TAG, "OS flag set during read — time may be invalid");
    }

    memset(utc, 0, sizeof(*utc));
    utc->tm_sec  = bcd_to_dec(regs[0] & 0x7F);
    utc->tm_min  = bcd_to_dec(regs[1] & 0x7F);
    utc->tm_hour = bcd_to_dec(regs[2] & 0x3F);
    utc->tm_mday = bcd_to_dec(regs[3] & 0x3F);
    utc->tm_wday = regs[4] & 0x07;
    utc->tm_mon  = bcd_to_dec(regs[5] & 0x1F) - 1;    // struct tm is 0-based
    utc->tm_year = bcd_to_dec(regs[6]) + 100;          // RTC year 0-99 → 2000-2099, tm_year from 1900

    return ESP_OK;
}

esp_err_t pcf85063_get_epoch(time_t *epoch_sec)
{
    if (!epoch_sec) return ESP_ERR_INVALID_ARG;

    struct tm utc;
    esp_err_t ret = pcf85063_get_time(&utc);
    if (ret != ESP_OK) return ret;

    *epoch_sec = mktime(&utc);  // Note: mktime interprets as local, but TZ is UTC on ESP32 by default
    return ESP_OK;
}

esp_err_t pcf85063_sync_to_system(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    time_t epoch;
    esp_err_t ret = pcf85063_get_epoch(&epoch);
    if (ret != ESP_OK) return ret;

    // Sanity check: epoch should be after Jan 1 2020
    if (epoch < 1577836800) {
        ESP_LOGW(TAG, "RTC time before 2020 (epoch=%lld) — skipping system sync",
                 (long long)epoch);
        return ESP_ERR_INVALID_RESPONSE;
    }

    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    struct tm utc;
    gmtime_r(&epoch, &utc);
    ESP_LOGI(TAG, "System clock synced from RTC: %04d-%02d-%02d %02d:%02d:%02d UTC",
             utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
             utc.tm_hour, utc.tm_min, utc.tm_sec);

    return ESP_OK;
}

// ============================================================================
// Timezone Offset (NVS persisted)
// ============================================================================

esp_err_t pcf85063_save_tz_offset(int16_t offset_min)
{
    s_tz_offset_min = offset_min;

    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_i16(nvs, NVS_KEY_TZ_OFFSET, offset_min);
    if (ret == ESP_OK) ret = nvs_commit(nvs);
    nvs_close(nvs);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Timezone offset saved: %+d min (UTC%+d:%02d)",
                 offset_min, offset_min / 60, abs(offset_min) % 60);
    }
    return ret;
}

int16_t pcf85063_get_tz_offset(void)
{
    return s_tz_offset_min;
}

#else /* !HAS_RTC — stub implementations */

esp_err_t pcf85063_init(void)           { return ESP_ERR_NOT_SUPPORTED; }
bool pcf85063_is_initialized(void)      { return false; }
esp_err_t pcf85063_set_time(const struct tm *utc) { (void)utc; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t pcf85063_set_epoch(time_t epoch_sec)    { (void)epoch_sec; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t pcf85063_get_time(struct tm *utc)       { (void)utc; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t pcf85063_get_epoch(time_t *epoch_sec)   { (void)epoch_sec; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t pcf85063_sync_to_system(void)           { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t pcf85063_save_tz_offset(int16_t offset_min) { (void)offset_min; return ESP_ERR_NOT_SUPPORTED; }
int16_t pcf85063_get_tz_offset(void)              { return 0; }

#endif /* HAS_RTC */
