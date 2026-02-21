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

// ============================================================================
// Background Task — Core 0, 50 Hz, Boot Calibration + Complementary Filter
// ============================================================================

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>

// Complementary filter coefficient (0.98 = 98% gyro, 2% accel)
#define COMP_FILTER_ALPHA   0.98f

// Task config
#define IMU_TASK_STACK      4096
#define IMU_TASK_PRIO       3
#define IMU_TASK_CORE       0       // Core 0 — same as touch, away from LVGL DMA
#define IMU_POLL_MS         20      // 50 Hz

// Calibration: 100 samples = 2 seconds at 50 Hz
#define CAL_SAMPLES         100

// Debug: log every N iterations (~1s at 50Hz)
#define IMU_DEBUG_INTERVAL  50

// NVS persistence
#define NVS_NAMESPACE       "imu_cal"
#define NVS_KEY_CAL         "cal"

/** NVS-storable calibration blob (version-tagged for future changes) */
typedef struct __attribute__((packed)) {
    uint8_t version;        // Blob format version (currently 1)
    float   gyro_bias[3];   // Gyro bias in DPS
    float   rot[3][3];      // Rotation matrix (sensor → reference)
} imu_cal_blob_t;

#define IMU_CAL_BLOB_VER    1

// Cached orientation — written by task, read by UI (volatile for cross-core)
static volatile qmi8658_orientation_t s_orient = { 0 };
static TaskHandle_t s_imu_task_handle = NULL;

// ---- Boot calibration data ----
static struct {
    float gyro_bias[3];     // Subtracted from raw gyro before rotation
    float rot[3][3];        // Rotation matrix: sensor frame → reference frame
    bool  valid;
} s_cal = { .valid = false };

// Flag: when true, force live calibration even if NVS has saved data
static bool s_force_recal = false;

// ============================================================================
// NVS Calibration Persistence
// ============================================================================

/** Save current calibration to NVS */
static esp_err_t cal_save_nvs(void)
{
    imu_cal_blob_t blob;
    blob.version = IMU_CAL_BLOB_VER;
    memcpy(blob.gyro_bias, s_cal.gyro_bias, sizeof(blob.gyro_bias));
    memcpy(blob.rot, s_cal.rot, sizeof(blob.rot));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(handle, NVS_KEY_CAL, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved to NVS");
    } else {
        ESP_LOGE(TAG, "NVS write failed: %s", esp_err_to_name(err));
    }
    return err;
}

/** Load calibration from NVS. Returns ESP_OK if valid data found. */
static esp_err_t cal_load_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;  // No namespace yet — first boot
    }

    imu_cal_blob_t blob;
    size_t len = sizeof(blob);
    err = nvs_get_blob(handle, NVS_KEY_CAL, &blob, &len);
    nvs_close(handle);

    if (err != ESP_OK || len != sizeof(blob)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (blob.version != IMU_CAL_BLOB_VER) {
        ESP_LOGW(TAG, "NVS cal version mismatch (%d vs %d), recalibrating",
                 blob.version, IMU_CAL_BLOB_VER);
        return ESP_ERR_INVALID_VERSION;
    }

    memcpy(s_cal.gyro_bias, blob.gyro_bias, sizeof(s_cal.gyro_bias));
    memcpy(s_cal.rot, blob.rot, sizeof(s_cal.rot));
    s_cal.valid = true;

    ESP_LOGI(TAG, "Calibration loaded from NVS: gyro_bias=[%.3f, %.3f, %.3f]",
             s_cal.gyro_bias[0], s_cal.gyro_bias[1], s_cal.gyro_bias[2]);
    ESP_LOGI(TAG, "NVS rot: [%.3f %.3f %.3f] [%.3f %.3f %.3f] [%.3f %.3f %.3f]",
             s_cal.rot[0][0], s_cal.rot[0][1], s_cal.rot[0][2],
             s_cal.rot[1][0], s_cal.rot[1][1], s_cal.rot[1][2],
             s_cal.rot[2][0], s_cal.rot[2][1], s_cal.rot[2][2]);
    return ESP_OK;
}

/**
 * @brief Build rotation matrix from measured gravity to Z-up
 *
 * Uses Rodrigues' formula to find R such that R * g_norm = [0, 0, 1].
 * After applying R, the boot orientation becomes pitch=0, roll=0.
 */
static void build_rotation_matrix(float gx, float gy, float gz)
{
    // Normalize gravity vector
    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    if (mag < 0.1f) {
        ESP_LOGW(TAG, "Cal: accel magnitude too low (%.3f), using identity", mag);
        // Identity matrix — no rotation
        s_cal.rot[0][0] = 1; s_cal.rot[0][1] = 0; s_cal.rot[0][2] = 0;
        s_cal.rot[1][0] = 0; s_cal.rot[1][1] = 1; s_cal.rot[1][2] = 0;
        s_cal.rot[2][0] = 0; s_cal.rot[2][1] = 0; s_cal.rot[2][2] = 1;
        return;
    }
    float ax = gx / mag;
    float ay = gy / mag;
    float az = gz / mag;

    // Rodrigues' formula: rotate vector [ax,ay,az] to [0,0,1]
    // d = 1 + az (denominator for 1/(1+cos(theta)))
    float d = 1.0f + az;

    if (fabsf(d) < 0.001f) {
        // Device nearly upside down (az ≈ -1): 180° rotation about X axis
        ESP_LOGW(TAG, "Cal: device nearly inverted, using 180° X rotation");
        s_cal.rot[0][0] =  1; s_cal.rot[0][1] =  0; s_cal.rot[0][2] =  0;
        s_cal.rot[1][0] =  0; s_cal.rot[1][1] = -1; s_cal.rot[1][2] =  0;
        s_cal.rot[2][0] =  0; s_cal.rot[2][1] =  0; s_cal.rot[2][2] = -1;
    } else {
        // General case
        s_cal.rot[0][0] = 1.0f - ax * ax / d;
        s_cal.rot[0][1] = -ax * ay / d;
        s_cal.rot[0][2] = -ax;
        s_cal.rot[1][0] = -ax * ay / d;
        s_cal.rot[1][1] = 1.0f - ay * ay / d;
        s_cal.rot[1][2] = -ay;
        s_cal.rot[2][0] = ax;
        s_cal.rot[2][1] = ay;
        s_cal.rot[2][2] = az;
    }
}

/** Apply 3x3 rotation matrix to a vector [x,y,z] */
static inline void rotate_vec(const float rot[3][3],
                              float x, float y, float z,
                              float *ox, float *oy, float *oz)
{
    *ox = rot[0][0] * x + rot[0][1] * y + rot[0][2] * z;
    *oy = rot[1][0] * x + rot[1][1] * y + rot[1][2] * z;
    *oz = rot[2][0] * x + rot[2][1] * y + rot[2][2] * z;
}

static void imu_task(void *arg)
{
    (void)arg;

    qmi8658_reading_t reading;
    float pitch = 0.0f, roll = 0.0f;
    bool filter_seeded = false;
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = IMU_POLL_MS / 1000.0f;     // 0.02s
    uint32_t loop_count = 0;

    // Calibration accumulators (running sum over CAL_SAMPLES)
    float cal_ax = 0, cal_ay = 0, cal_az = 0;
    float cal_gx = 0, cal_gy = 0, cal_gz = 0;
    int   cal_count = 0;

    // Try loading calibration from NVS (skip live cal if valid + not forced)
    if (!s_force_recal && cal_load_nvs() == ESP_OK) {
        ESP_LOGI(TAG, "IMU task started on core %d @ %d Hz — using saved calibration",
                 xPortGetCoreID(), 1000 / IMU_POLL_MS);
    } else {
        s_cal.valid = false;
        ESP_LOGI(TAG, "IMU task started on core %d @ %d Hz — calibrating (%d samples)...",
                 xPortGetCoreID(), 1000 / IMU_POLL_MS, CAL_SAMPLES);
    }
    s_force_recal = false;   // Reset flag after use

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_POLL_MS));

        if (qmi8658_read(&reading) != ESP_OK) {
            continue;
        }

        // ================================================================
        // Phase 1: Boot calibration (first 2 seconds — device must be still)
        // ================================================================
        if (!s_cal.valid) {
            cal_ax += reading.accel.x;
            cal_ay += reading.accel.y;
            cal_az += reading.accel.z;
            cal_gx += reading.gyro.x;
            cal_gy += reading.gyro.y;
            cal_gz += reading.gyro.z;
            cal_count++;

            if (cal_count >= CAL_SAMPLES) {
                float n = (float)cal_count;

                // Gyro bias (average at rest — subtract from all future readings)
                s_cal.gyro_bias[0] = cal_gx / n;
                s_cal.gyro_bias[1] = cal_gy / n;
                s_cal.gyro_bias[2] = cal_gz / n;

                // Build rotation matrix from average gravity → Z-up
                build_rotation_matrix(cal_ax / n, cal_ay / n, cal_az / n);
                s_cal.valid = true;

                ESP_LOGI(TAG, "Cal done: gravity=[%.3f, %.3f, %.3f] "
                              "gyro_bias=[%.3f, %.3f, %.3f]",
                         cal_ax / n, cal_ay / n, cal_az / n,
                         s_cal.gyro_bias[0], s_cal.gyro_bias[1], s_cal.gyro_bias[2]);
                ESP_LOGI(TAG, "Cal rot: [%.3f %.3f %.3f] [%.3f %.3f %.3f] [%.3f %.3f %.3f]",
                         s_cal.rot[0][0], s_cal.rot[0][1], s_cal.rot[0][2],
                         s_cal.rot[1][0], s_cal.rot[1][1], s_cal.rot[1][2],
                         s_cal.rot[2][0], s_cal.rot[2][1], s_cal.rot[2][2]);

                // Persist to NVS
                cal_save_nvs();
            }
            continue;   // Skip orientation output during calibration
        }

        // ================================================================
        // Phase 2: Apply calibration and compute orientation
        // ================================================================

        // Subtract gyro bias
        float gx_raw = reading.gyro.x - s_cal.gyro_bias[0];
        float gy_raw = reading.gyro.y - s_cal.gyro_bias[1];
        float gz_raw = reading.gyro.z - s_cal.gyro_bias[2];

        // Rotate accel + gyro into reference frame (boot orientation = level)
        float ax, ay, az;
        rotate_vec(s_cal.rot,
                   reading.accel.x, reading.accel.y, reading.accel.z,
                   &ax, &ay, &az);
        float gx, gy, gz;
        rotate_vec(s_cal.rot, gx_raw, gy_raw, gz_raw, &gx, &gy, &gz);

        // Accelerometer-based tilt (now in reference frame: Z=up, X=forward, Y=right)
        float acc_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);
        float acc_roll  = atan2f(-ay, az) * (180.0f / M_PI);

        if (!filter_seeded) {
            pitch = acc_pitch;
            roll  = acc_roll;
            filter_seeded = true;
        } else {
            // Complementary filter: gyro short-term + accel long-term
            pitch = COMP_FILTER_ALPHA * (pitch + gy * dt) + (1.0f - COMP_FILTER_ALPHA) * acc_pitch;
            roll  = COMP_FILTER_ALPHA * (roll  + gx * dt) + (1.0f - COMP_FILTER_ALPHA) * acc_roll;
        }

        // Clamp angles
        if (pitch >  90.0f) pitch =  90.0f;
        if (pitch < -90.0f) pitch = -90.0f;
        if (roll  >  180.0f) roll =  180.0f;
        if (roll  < -180.0f) roll = -180.0f;

        // Total G magnitude (from raw — rotation doesn't change magnitude)
        float g_total = sqrtf(ax * ax + ay * ay + az * az);

        // Write to volatile cache
        qmi8658_orientation_t o;
        o.pitch       = pitch;
        o.roll        = roll;
        o.yaw_rate    = gz;             // Yaw = rotation about calibrated vertical
        o.accel_lat   = ay;             // Calibrated lateral G (right = +)
        o.accel_lon   = ax;             // Calibrated longitudinal G (forward = +)
        o.accel_vert  = az;             // Calibrated vertical G (up = +, ~1.0 at rest)
        o.g_total     = g_total;
        o.temperature = reading.temperature;
        o.timestamp   = reading.timestamp;
        o.valid       = true;

        *((qmi8658_orientation_t *)&s_orient) = o;

        // Periodic debug output (ESP_LOG_DEBUG — enable with esp_log_level_set("qmi8658", ESP_LOG_DEBUG))
        loop_count++;
        if ((loop_count % IMU_DEBUG_INTERVAL) == 0) {
            ESP_LOGD(TAG, "CAL ACC: X=%.3f Y=%.3f Z=%.3f | GYRO: X=%.2f Y=%.2f Z=%.2f | "
                          "P=%.1f R=%.1f Yaw=%.1f | Gtot=%.2f T=%.1fC",
                     ax, ay, az, gx, gy, gz,
                     pitch, roll, gz, g_total, reading.temperature);
        }
    }
}

esp_err_t qmi8658_start_task(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Cannot start task — QMI8658 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_imu_task_handle) {
        ESP_LOGW(TAG, "IMU task already running");
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        imu_task, "imu_task", IMU_TASK_STACK, NULL,
        IMU_TASK_PRIO, &s_imu_task_handle, IMU_TASK_CORE);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void qmi8658_stop_task(void)
{
    if (s_imu_task_handle) {
        vTaskDelete(s_imu_task_handle);
        s_imu_task_handle = NULL;
        ESP_LOGI(TAG, "IMU task stopped");
    }
}

esp_err_t qmi8658_get_orientation(qmi8658_orientation_t *orient)
{
    if (!orient) return ESP_ERR_INVALID_ARG;
    *orient = *((qmi8658_orientation_t *)&s_orient);
    return orient->valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t qmi8658_calibrate(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    // Stop existing task, force recal, restart
    qmi8658_stop_task();
    s_cal.valid = false;
    s_force_recal = true;
    // Clear orientation cache so UI shows no data during cal
    memset((void *)&s_orient, 0, sizeof(s_orient));
    ESP_LOGI(TAG, "Recalibration requested — hold device still for 2 seconds");
    return qmi8658_start_task();
}

esp_err_t qmi8658_clear_calibration(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_erase_key(handle, NVS_KEY_CAL);
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_commit(handle);
        err = ESP_OK;
    }
    nvs_close(handle);
    s_cal.valid = false;
    ESP_LOGI(TAG, "Calibration cleared from NVS");
    return err;
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
esp_err_t qmi8658_start_task(void) { return ESP_ERR_NOT_SUPPORTED; }
void qmi8658_stop_task(void) { }
esp_err_t qmi8658_get_orientation(qmi8658_orientation_t *o) { (void)o; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_calibrate(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t qmi8658_clear_calibration(void) { return ESP_ERR_NOT_SUPPORTED; }

#endif // HAS_IMU
