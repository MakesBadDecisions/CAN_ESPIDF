/**
 * @file ws_touch_lcd_2_1.h
 * @brief Waveshare ESP32-S3-Touch-LCD-2.1 Display Configuration
 * 
 * Hardware-specific pin definitions and parameters for the Waveshare 2.1" round display.
 * ESP32-S3R8 with 2.1" round IPS RGB LCD (480x480).
 * 16MB Flash, 8MB Octal PSRAM
 * 
 * KEY FEATURES:
 * - Round 480x480 IPS display (ST7701S) with RGB565 parallel interface
 * - CST820 capacitive touch (I2C, single-point) — NOT SPI like CrowPanel
 * - QMI8658 6-axis IMU (accelerometer + gyroscope) — onboard!
 * - PCF85063A RTC — onboard!
 * - TCA9554 GPIO expander controlling LCD reset, touch reset, LCD SPI CS, SD D3, buzzer
 * - Onboard buzzer (via TCA9554 EXIO8)
 * - Battery charging circuit with ADC monitoring (GPIO4)
 * - SD card via SDMMC 1-bit mode (not SPI) — shares GPIO1/2 with LCD SPI init
 * - CH343P USB-UART converter (not native USB)
 * 
 * I2C BUS (shared by all onboard peripherals):
 *   SDA=GPIO15, SCL=GPIO7, 400kHz
 *   0x15 = CST820 touch
 *   0x20 = TCA9554 GPIO expander
 *   0x51 = PCF85063A RTC
 *   0x6B = QMI8658 IMU
 * 
 * GPIO SHARING NOTE:
 *   GPIO1 and GPIO2 are shared between LCD SPI init (MOSI/SCLK) and SD card (CMD/CLK).
 *   LCD SPI is only used during panel initialization, then the pins are available for SD card.
 * 
 * UART COMM LINK:
 *   The 12-pin header and 4-pin UART header expose UART pins.
 *   UART header is DISABLED when USB-C is connected (FSUSB42UMX switch).
 *   Check Waveshare schematic for exact GPIO assignments on the 12-pin header.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Device Identification
// ============================================================================

#define DEVICE_NAME             "WS_TOUCH_LCD_2_1"
#define DEVICE_MODEL            "2.1inch_Round_RGB_LCD"

// ============================================================================
// Display Specifications
// ============================================================================

#define DISPLAY_WIDTH           480
#define DISPLAY_HEIGHT          480
#define DISPLAY_COLOR_DEPTH     16      // RGB565
#define DISPLAY_DATA_WIDTH      16      // RGB parallel data bus width (bits)
#define DISPLAY_BITS_PER_PX     16      // Bits per pixel (RGB565 = 16)
#define DISPLAY_IS_ROUND        1       // Round display — UI must account for corners
#define BOUNCE_BUFFER_LINES     10      // Bounce buffer height (480/10 = 48 draws)

// ============================================================================
// RGB Interface Pin Configuration (ST7701S RGB565 parallel)
// ============================================================================

// Blue pins (B0-B4)
#define PIN_B0      5       // GPIO_NUM_5   — DATA0
#define PIN_B1      45      // GPIO_NUM_45  — DATA1
#define PIN_B2      48      // GPIO_NUM_48  — DATA2
#define PIN_B3      47      // GPIO_NUM_47  — DATA3
#define PIN_B4      21      // GPIO_NUM_21  — DATA4

// Green pins (G0-G5)
#define PIN_G0      14      // GPIO_NUM_14  — DATA5
#define PIN_G1      13      // GPIO_NUM_13  — DATA6
#define PIN_G2      12      // GPIO_NUM_12  — DATA7
#define PIN_G3      11      // GPIO_NUM_11  — DATA8
#define PIN_G4      10      // GPIO_NUM_10  — DATA9
#define PIN_G5      9       // GPIO_NUM_9   — DATA10

// Red pins (R0-R4)
#define PIN_R0      46      // GPIO_NUM_46  — DATA11
#define PIN_R1      3       // GPIO_NUM_3   — DATA12
#define PIN_R2      8       // GPIO_NUM_8   — DATA13
#define PIN_R3      18      // GPIO_NUM_18  — DATA14
#define PIN_R4      17      // GPIO_NUM_17  — DATA15

// Control pins
#define PIN_HSYNC       38      // GPIO_NUM_38
#define PIN_VSYNC       39      // GPIO_NUM_39
#define PIN_DE          40      // GPIO_NUM_40
#define PIN_PCLK        41      // GPIO_NUM_41

// Backlight control (PWM via LEDC)
#define PIN_BACKLIGHT   6       // GPIO_NUM_6

// No direct panel enable or reset pin — handled via TCA9554 GPIO expander
// #define PIN_PANEL_ENABLE  — not applicable

// ============================================================================
// LCD SPI Init Interface (ST7701S register programming)
// ============================================================================
// The ST7701S requires SPI commands to configure the panel before RGB mode.
// These pins are shared with SD card (SDMMC CMD/CLK)!

#define LCD_SPI_HOST        SPI2_HOST
#define LCD_SPI_MOSI        1       // GPIO_NUM_1  — shared with SD CMD
#define LCD_SPI_SCLK        2       // GPIO_NUM_2  — shared with SD CLK
#define LCD_SPI_CS          (-1)    // Controlled via TCA9554 EXIO3
#define LCD_SPI_FREQ        4000000 // 4 MHz

// ============================================================================
// Display Timing Parameters (from Waveshare demo)
// ============================================================================

// Pixel clock (18MHz)
#define DISPLAY_FREQ_WRITE      18000000

// HSYNC timing
#define HSYNC_POLARITY          0
#define HSYNC_FRONT_PORCH       50
#define HSYNC_PULSE_WIDTH       8
#define HSYNC_BACK_PORCH        10

// VSYNC timing
#define VSYNC_POLARITY          0
#define VSYNC_FRONT_PORCH       8
#define VSYNC_PULSE_WIDTH       3
#define VSYNC_BACK_PORCH        8

// Clock settings
#define PCLK_ACTIVE_NEG         0
#define DE_IDLE_HIGH            0
#define PCLK_IDLE_HIGH          0

// ============================================================================
// Backlight PWM Configuration
// ============================================================================

#define BL_LEDC_TIMER           0       // LEDC_TIMER_0
#define BL_LEDC_CHANNEL         0       // LEDC_CHANNEL_0
#define BL_LEDC_SPEED_MODE      1       // LEDC_LOW_SPEED_MODE
#define BL_PWM_RESOLUTION       13      // 13-bit resolution
#define BL_PWM_FREQUENCY        5000    // 5 kHz
#define BL_DEFAULT_BRIGHTNESS   70      // 0-100 percentage

// ============================================================================
// Touch Controller (CST820 — I2C Capacitive, single-point)
// ============================================================================

#define TOUCH_TYPE_CST820       1
#define TOUCH_I2C_ADDR          0x15
#define TOUCH_I2C_PORT          I2C_NUM_0   // Shared I2C bus
#define TOUCH_INT_PIN           16          // GPIO_NUM_16 — negedge interrupt
#define TOUCH_RST_PIN           (-1)        // Reset via TCA9554 EXIO2

// Touch coordinate mapping (matches LCD resolution)
#define TOUCH_X_MIN             0
#define TOUCH_X_MAX             480
#define TOUCH_Y_MIN             0
#define TOUCH_Y_MAX             480
#define TOUCH_X_INVERTED        0
#define TOUCH_SWAP_XY           0
#define TOUCH_MIRROR_X          0
#define TOUCH_MIRROR_Y          0

// ============================================================================
// I2C Bus Configuration (shared by all onboard peripherals)
// ============================================================================

#define I2C_BUS_PORT            I2C_NUM_0
#define I2C_BUS_SDA             15      // GPIO_NUM_15
#define I2C_BUS_SCL             7       // GPIO_NUM_7
#define I2C_BUS_FREQ            400000  // 400 kHz
#define I2C_BUS_TIMEOUT_MS      1000

// ============================================================================
// TCA9554 GPIO Expander (I2C)
// ============================================================================

#define TCA9554_I2C_ADDR        0x20
#define TCA9554_I2C_PORT        I2C_NUM_0   // Shared I2C bus

// EXIO pin assignments (active-high unless noted)
#define EXIO_LCD_RESET          1       // EXIO1 — LCD reset (active-low pulse)
#define EXIO_TOUCH_RESET        2       // EXIO2 — Touch reset (active-low pulse)
#define EXIO_LCD_SPI_CS         3       // EXIO3 — LCD SPI CS (high = enabled)
#define EXIO_SD_D3              4       // EXIO4 — SD card D3 (high = enabled)
#define EXIO_BUZZER             8       // EXIO8 — Buzzer (high = on)

// Unused EXIO pins: 5, 6, 7

// ============================================================================
// QMI8658 6-Axis IMU (Accelerometer + Gyroscope)
// ============================================================================

#define QMI8658_I2C_ADDR        0x6B
#define QMI8658_I2C_PORT        I2C_NUM_0   // Shared I2C bus

#define QMI8658_ACCEL_RANGE     4       // ±4G default
#define QMI8658_ACCEL_ODR       8000    // 8000 Hz
#define QMI8658_GYRO_RANGE      512     // ±512 DPS (64 clips on normal movement)
#define QMI8658_GYRO_ODR        8000    // 8000 Hz

// ============================================================================
// PCF85063A Real-Time Clock
// ============================================================================

#define PCF85063_I2C_ADDR       0x51
#define PCF85063_I2C_PORT       I2C_NUM_0   // Shared I2C bus

// ============================================================================
// UART Communication Link to CAN Interface Node
// ============================================================================

// UART1 on 12-pin multi-functional header.
// NOTE: The 4-pin UART header is DISABLED when USB-C is connected (FSUSB42UMX switch).
#define UART_COMM_TX_PIN        43      // GPIO_NUM_43
#define UART_COMM_RX_PIN        44      // GPIO_NUM_44
#define UART_COMM_BAUD          2000000
#define UART_COMM_PORT          UART_NUM_1

// ============================================================================
// SD Card (SDMMC 1-bit Mode — NOT SPI!)
// ============================================================================
// SD card uses SDMMC peripheral, NOT SPI.
// GPIO1/2 are shared with LCD SPI init — LCD SPI finishes before SD init.

#define SD_USE_SDMMC            1       // SDMMC mode, not SPI
#define SD_SDMMC_CLK            2       // GPIO_NUM_2  — shared with LCD SPI SCLK
#define SD_SDMMC_CMD            1       // GPIO_NUM_1  — shared with LCD SPI MOSI
#define SD_SDMMC_D0             42      // GPIO_NUM_42
#define SD_SDMMC_BUS_WIDTH      1       // 1-bit mode
// SD D3 is controlled via TCA9554 EXIO4 (must be set HIGH to enable)

// ============================================================================
// Battery ADC Monitoring
// ============================================================================

#define BAT_ADC_PIN             4       // GPIO_NUM_4  — ADC1_CH3
#define BAT_ADC_CHANNEL         3       // ADC1 Channel 3
#define BAT_ADC_ATTEN           3       // ADC_ATTEN_DB_12
#define BAT_VOLTAGE_DIVIDER     3.0f    // 3:1 voltage divider
#define BAT_CAL_OFFSET          0.9945f // Calibration factor

// ============================================================================
// Buzzer (via TCA9554 GPIO Expander)
// ============================================================================

// Buzzer has no direct GPIO — controlled via TCA9554 EXIO8
// Use Set_EXIO(EXIO_BUZZER, true/false)
#define HAS_BUZZER              1

// ============================================================================
// LVGL Buffer Configuration
// ============================================================================

#define LVGL_BUFFER_FACTOR      4       // screenWidth * screenHeight / factor
// With 8MB PSRAM we can afford larger buffers than the CrowPanel (2MB)

// ============================================================================
// Device Capabilities
// ============================================================================

#define HAS_TOUCH_CONTROLLER    1
#define HAS_BACKLIGHT_CONTROL   1
#define SUPPORTS_ROTATION       0       // Round display — rotation not meaningful
#define HAS_AUDIO               0       // No I2S audio
#define HAS_SD_CARD             1
#define HAS_IMU                 1       // QMI8658 onboard
#define HAS_RTC                 1       // PCF85063A onboard
#define HAS_GPIO_EXPANDER       1       // TCA9554 onboard
#define HAS_BATTERY_MONITOR     1       // Battery ADC on GPIO4
#define MAX_BRIGHTNESS          100     // Percentage-based (PWM)
#define HAS_PSRAM               1
#define FLASH_SIZE_MB           16
#define PSRAM_SIZE_MB           8
