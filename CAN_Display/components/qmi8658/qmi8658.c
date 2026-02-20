/**
 * @file qmi8658.c
 * @brief QMI8658 6-Axis IMU Driver Implementation
 *
 * Supports accelerometer (±2/4/8/16G) and gyroscope (±16 to ±2048 DPS)
 * on the shared I2C bus. Uses auto-address-increment for efficient burst reads.
 *
 * Devices without HAS_IMU get stub implementations.
 */

#include "qmi8658.h"
#include "device.h"
#include "esp_log.h"

static const char *TAG = "qmi8658";

#if defined(HAS_IMU) && HAS_IMU

#include "i2c_bus.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include <string.h>

// ============================================================================
// QMI8658 Register Map
// ============================================================================

#define REG_WHO_AM_I        0x00
#define REG_REVISION_ID     0x01
#define REG_CTRL1           0x02    // SPI config + oscillator + auto-increment
#define REG_CTRL2           0x03    // Accelerometer scale + ODR
#define REG_CTRL3           0x04    // Gyroscope scale + ODR
#define REG_CTRL5           0x06    // Low-pass filter config
#define REG_CTRL6           0x07    // AttitudeEngine (disabled)
#define REG_CTRL7           0x08    // Sensor enable master register
#define REG_CTRL9           0x0A    // Host command register
#define REG_STATUSINT       0x2D   // Status + interrupt flags
#define REG_TEMP_L          0x33    // Temperature low byte
#define REG_AX_L            0x35    // Accel X low byte (burst read start)
#define REG_GX_L            0x3B    // Gyro X low byte

// Expected chip ID
#define QMI8658_CHIP_ID     0x05

// CTRL7 values
#define CTRL7_ACC_GYRO_EN   0x03    // Acc + Gyro enabled
#define CTRL7_HIGH_CLK      0x40    // High-speed internal clock
#define CTRL7_RUNNING       (CTRL7_ACC_GYRO_EN | CTRL7_HIGH_CLK)  // 0x43
#define CTRL7_POWER_DOWN    0x00

// CTRL5 masks
#define CTRL5_ACC_LPF_EN    0x01
#define CTRL5_ACC_LPF_MASK  0x06    // bits [2:1]
#define CTRL5_GYRO_LPF_EN   0x10
#define CTRL5_GYRO_LPF_MASK 0x60   // bits [6:5]

// LPF modes (bandwidth as % of ODR)
#define LPF_MODE_0          0x00    // 2.66%  (tightest)
#define LPF_MODE_3          0x03    // 13.37% (widest)

// ============================================================================
// Sensitivity tables (full_scale / 32768)
// ============================================================================

static const float s_acc_sensitivity[] = {
    2.0f  / 32768.0f,   // ±2G
    4.0f  / 32768.0f,   // ±4G
    8.0f  / 32768.0f,   // ±8G
    16.0f / 32768.0f,   // ±16G
};

static const float s_gyro_sensitivity[] = {
    16.0f   / 32768.0f, // ±16 DPS
    32.0f   / 32768.0f, // ±32 DPS
    64.0f   / 32768.0f, // ±64 DPS
    128.0f  / 32768.0f, // ±128 DPS
    256.0f  / 32768.0f, // ±256 DPS
    512.0f  / 32768.0f, // ±512 DPS
    1024.0f / 32768.0f, // ±1024 DPS
    2048.0f / 32768.0f, // ±2048 DPS
};

// ============================================================================
// State
// ============================================================================

static bool                 s_initialized = false;
static qmi8658_acc_range_t  s_acc_range   = QMI8658_ACC_RANGE_4G;
static qmi8658_gyro_range_t s_gyro_range  = QMI8658_GYRO_RANGE_64DPS;

// ============================================================================
// I2C Helpers
// ============================================================================

static esp_err_t qmi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(
        i2c_bus_get_port(), QMI8658_I2C_ADDR,
        buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t qmi_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(
        i2c_bus_get_port(), QMI8658_I2C_ADDR,
        &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

static esp_err_t qmi_read_regs(uint8_t start_reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        i2c_bus_get_port(), QMI8658_I2C_ADDR,
        &start_reg, 1, data, len, pdMS_TO_TICKS(100));
}

// ============================================================================
// Range mapping from device header defines to enum values
// ============================================================================

static qmi8658_acc_range_t acc_range_from_define(int range_g)
{
    switch (range_g) {
        case 2:  return QMI8658_ACC_RANGE_2G;
        case 4:  return QMI8658_ACC_RANGE_4G;
        case 8:  return QMI8658_ACC_RANGE_8G;
        case 16: return QMI8658_ACC_RANGE_16G;
        default: return QMI8658_ACC_RANGE_4G;
    }
}

static qmi8658_gyro_range_t gyro_range_from_define(int range_dps)
{
    switch (range_dps) {
        case 16:   return QMI8658_GYRO_RANGE_16DPS;
        case 32:   return QMI8658_GYRO_RANGE_32DPS;
        case 64:   return QMI8658_GYRO_RANGE_64DPS;
        case 128:  return QMI8658_GYRO_RANGE_128DPS;
        case 256:  return QMI8658_GYRO_RANGE_256DPS;
        case 512:  return QMI8658_GYRO_RANGE_512DPS;
        case 1024: return QMI8658_GYRO_RANGE_1024DPS;
        case 2048: return QMI8658_GYRO_RANGE_2048DPS;
        default:   return QMI8658_GYRO_RANGE_64DPS;
    }
}

static qmi8658_odr_t odr_from_define(int odr_hz)
{
    switch (odr_hz) {
        case 8000: return QMI8658_ODR_8000HZ;
        case 4000: return QMI8658_ODR_4000HZ;
        case 2000: return QMI8658_ODR_2000HZ;
        case 1000: return QMI8658_ODR_1000HZ;
        case 500:  return QMI8658_ODR_500HZ;
        case 250:  return QMI8658_ODR_250HZ;
        case 125:  return QMI8658_ODR_125HZ;
        case 62:   return QMI8658_ODR_62HZ;
        case 31:   return QMI8658_ODR_31HZ;
        default:   return QMI8658_ODR_8000HZ;
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t qmi8658_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (!i2c_bus_is_initialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized — call i2c_bus_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    // Read and verify chip ID
    uint8_t chip_id = 0;
    esp_err_t ret = qmi_read_reg(REG_WHO_AM_I, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        return ret;
    }
    if (chip_id != QMI8658_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X (expected 0x%02X)",
                 chip_id, QMI8658_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }

    // Read revision ID
    uint8_t rev_id = 0;
    qmi_read_reg(REG_REVISION_ID, &rev_id);
    ESP_LOGI(TAG, "QMI8658 detected (chip ID: 0x%02X, rev: 0x%02X)", chip_id, rev_id);

    // Configure CTRL1: enable 2MHz oscillator + auto-address-increment
    uint8_t ctrl1 = 0;
    qmi_read_reg(REG_CTRL1, &ctrl1);
    ctrl1 &= 0xFE;     // Clear bit 0 — enable 2MHz oscillator
    ctrl1 |= 0x40;     // Set bit 6 — enable auto-address-increment
    qmi_write_reg(REG_CTRL1, ctrl1);

    // Enable accelerometer + gyroscope with high-speed clock
    qmi_write_reg(REG_CTRL7, CTRL7_RUNNING);

    // Disable AttitudeEngine
    qmi_write_reg(REG_CTRL6, 0x00);

    // Map device header config to enum values
    s_acc_range  = acc_range_from_define(QMI8658_ACCEL_RANGE);
    s_gyro_range = gyro_range_from_define(QMI8658_GYRO_RANGE);
    qmi8658_odr_t acc_odr  = odr_from_define(QMI8658_ACCEL_ODR);
    qmi8658_odr_t gyro_odr = odr_from_define(QMI8658_GYRO_ODR);

    // Configure accelerometer: scale + ODR in CTRL2
    uint8_t ctrl2 = 0;
    qmi_read_reg(REG_CTRL2, &ctrl2);
    ctrl2 &= 0x80;     // Clear bits [6:0], keep reserved bit 7
    ctrl2 |= ((uint8_t)s_acc_range << 4) | (uint8_t)acc_odr;
    qmi_write_reg(REG_CTRL2, ctrl2);

    // Configure gyroscope: scale + ODR in CTRL3
    uint8_t ctrl3 = 0;
    qmi_read_reg(REG_CTRL3, &ctrl3);
    ctrl3 &= 0x80;     // Clear bits [6:0], keep reserved bit 7
    ctrl3 |= ((uint8_t)s_gyro_range << 4) | (uint8_t)gyro_odr;
    qmi_write_reg(REG_CTRL3, ctrl3);

    // Configure low-pass filters in CTRL5
    // Accel: LPF_MODE_0 (2.66% BW, tightest filtering)
    // Gyro: LPF_MODE_3 (13.37% BW, widest filtering)
    uint8_t ctrl5 = 0;
    ctrl5 |= CTRL5_ACC_LPF_EN | (LPF_MODE_0 << 1);    // Accel LPF on, mode 0
    ctrl5 |= CTRL5_GYRO_LPF_EN | (LPF_MODE_3 << 5);   // Gyro LPF on, mode 3
    qmi_write_reg(REG_CTRL5, ctrl5);

    s_initialized = true;

    ESP_LOGI(TAG, "QMI8658 initialized: Accel ±%dG @ %dHz, Gyro ±%d DPS @ %dHz",
             QMI8658_ACCEL_RANGE, QMI8658_ACCEL_ODR,
             QMI8658_GYRO_RANGE, QMI8658_GYRO_ODR);

    return ESP_OK;
}

bool qmi8658_is_initialized(void)
{
    return s_initialized;
}

esp_err_t qmi8658_read(qmi8658_reading_t *reading)
{
    if (!s_initialized || !reading) {
        return ESP_ERR_INVALID_STATE;
    }

    // Burst read 14 bytes: temp(2) + accel(6) + gyro(6) starting at 0x33
    uint8_t buf[14];
    esp_err_t ret = qmi_read_regs(REG_TEMP_L, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }

    // Temperature (signed 16-bit, divide by 256 for °C)
    int16_t raw_temp = (int16_t)((buf[1] << 8) | buf[0]);
    reading->temperature = raw_temp / 256.0f;

    // Accelerometer XYZ (little-endian signed 16-bit)
    int16_t ax = (int16_t)((buf[3]  << 8) | buf[2]);
    int16_t ay = (int16_t)((buf[5]  << 8) | buf[4]);
    int16_t az = (int16_t)((buf[7]  << 8) | buf[6]);

    float acc_sens = s_acc_sensitivity[s_acc_range];
    reading->accel.x = ax * acc_sens;
    reading->accel.y = ay * acc_sens;
    reading->accel.z = az * acc_sens;

    // Gyroscope XYZ (little-endian signed 16-bit)
    int16_t gx = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t gy = (int16_t)((buf[11] << 8) | buf[10]);
    int16_t gz = (int16_t)((buf[13] << 8) | buf[12]);

    float gyro_sens = s_gyro_sensitivity[s_gyro_range];
    reading->gyro.x = gx * gyro_sens;
    reading->gyro.y = gy * gyro_sens;
    reading->gyro.z = gz * gyro_sens;

    reading->timestamp = (uint32_t)(esp_timer_get_time() / 1000);

    return ESP_OK;
}

esp_err_t qmi8658_read_accel_raw(qmi8658_raw_t *raw)
{
    if (!s_initialized || !raw) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[6];
    esp_err_t ret = qmi_read_regs(REG_AX_L, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }

    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);

    return ESP_OK;
}

esp_err_t qmi8658_read_gyro_raw(qmi8658_raw_t *raw)
{
    if (!s_initialized || !raw) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[6];
    esp_err_t ret = qmi_read_regs(REG_GX_L, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }

    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);

    return ESP_OK;
}

esp_err_t qmi8658_read_temperature(float *temp_c)
{
    if (!s_initialized || !temp_c) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[2];
    esp_err_t ret = qmi_read_regs(REG_TEMP_L, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw = (int16_t)((buf[1] << 8) | buf[0]);
    *temp_c = raw / 256.0f;

    return ESP_OK;
}

esp_err_t qmi8658_set_acc_range(qmi8658_acc_range_t range)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (range > QMI8658_ACC_RANGE_16G) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ctrl2 = 0;
    esp_err_t ret = qmi_read_reg(REG_CTRL2, &ctrl2);
    if (ret != ESP_OK) return ret;

    ctrl2 &= ~0x70;    // Clear bits [6:4]
    ctrl2 |= ((uint8_t)range << 4);
    ret = qmi_write_reg(REG_CTRL2, ctrl2);
    if (ret != ESP_OK) return ret;

    s_acc_range = range;
    ESP_LOGI(TAG, "Accel range set to ±%dG",
             (range == 0) ? 2 : (range == 1) ? 4 : (range == 2) ? 8 : 16);

    return ESP_OK;
}

esp_err_t qmi8658_set_gyro_range(qmi8658_gyro_range_t range)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (range > QMI8658_GYRO_RANGE_2048DPS) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ctrl3 = 0;
    esp_err_t ret = qmi_read_reg(REG_CTRL3, &ctrl3);
    if (ret != ESP_OK) return ret;

    ctrl3 &= ~0x70;    // Clear bits [6:4]
    ctrl3 |= ((uint8_t)range << 4);
    ret = qmi_write_reg(REG_CTRL3, ctrl3);
    if (ret != ESP_OK) return ret;

    s_gyro_range = range;
    ESP_LOGI(TAG, "Gyro range set to ±%d DPS", 16 << range);

    return ESP_OK;
}

esp_err_t qmi8658_power_down(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = qmi_write_reg(REG_CTRL7, CTRL7_POWER_DOWN);
    if (ret == ESP_OK) {
        // Disable 2MHz oscillator
        uint8_t ctrl1 = 0;
        qmi_read_reg(REG_CTRL1, &ctrl1);
        ctrl1 |= 0x01;     // Set bit 0 — disable oscillator
        qmi_write_reg(REG_CTRL1, ctrl1);
        ESP_LOGI(TAG, "Powered down");
    }
    return ret;
}

esp_err_t qmi8658_wake(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Re-enable oscillator
    uint8_t ctrl1 = 0;
    qmi_read_reg(REG_CTRL1, &ctrl1);
    ctrl1 &= 0xFE;     // Clear bit 0 — enable oscillator
    qmi_write_reg(REG_CTRL1, ctrl1);

    // Re-enable sensors
    esp_err_t ret = qmi_write_reg(REG_CTRL7, CTRL7_RUNNING);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Woke from power down");
    }
    return ret;
}

#else
// ============================================================================
// Stub implementations for devices without IMU
// ============================================================================

esp_err_t qmi8658_init(void)
{
    ESP_LOGW(TAG, "IMU not available on this device");
    return ESP_ERR_NOT_SUPPORTED;
}

bool qmi8658_is_initialized(void)          { return false; }
esp_err_t qmi8658_read(qmi8658_reading_t *r) { (void)r; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_read_accel_raw(qmi8658_raw_t *r) { (void)r; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_read_gyro_raw(qmi8658_raw_t *r) { (void)r; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_read_temperature(float *t) { (void)t; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_set_acc_range(qmi8658_acc_range_t r) { (void)r; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_set_gyro_range(qmi8658_gyro_range_t r) { (void)r; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_power_down(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_wake(void) { return ESP_ERR_NOT_SUPPORTED; }

#endif // HAS_IMU
