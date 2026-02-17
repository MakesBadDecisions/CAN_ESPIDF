/**
 * @file dis06043h.h
 * @brief CrowPanel ESP32 HMI 4.3-inch Display Configuration
 * 
 * Hardware-specific pin definitions and parameters for the DIS06043H display.
 * ESP32-S3-WROOM-1-N4R2 with 4.3" RGB LCD (480x272).
 * 4MB Flash, 2MB PSRAM
 * 
 * KEY ADVANTAGE: UART1 available on dedicated HY2.0-4P connector (GPIO17/18)
 * - Same pins as CAN_Interface, direct wiring compatibility
 * - Touch uses SPI (not I2C), no pin conflicts
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Device Identification
// ============================================================================

#define DEVICE_NAME         "DIS06043H"
#define DEVICE_MODEL        "4.3inch_RGB_LCD"

// ============================================================================
// Display Specifications
// ============================================================================

#define DISPLAY_WIDTH       480
#define DISPLAY_HEIGHT      272
#define DISPLAY_COLOR_DEPTH 16   // RGB565

// ============================================================================
// RGB Interface Pin Configuration (DIS06043H specific)
// ============================================================================

// Blue pins (B0-B4)
#define PIN_B0  8   // GPIO_NUM_8
#define PIN_B1  3   // GPIO_NUM_3
#define PIN_B2  46  // GPIO_NUM_46
#define PIN_B3  9   // GPIO_NUM_9
#define PIN_B4  1   // GPIO_NUM_1

// Green pins (G0-G5)
#define PIN_G0  5   // GPIO_NUM_5
#define PIN_G1  6   // GPIO_NUM_6
#define PIN_G2  7   // GPIO_NUM_7
#define PIN_G3  15  // GPIO_NUM_15
#define PIN_G4  16  // GPIO_NUM_16
#define PIN_G5  4   // GPIO_NUM_4

// Red pins (R0-R4)
#define PIN_R0  45  // GPIO_NUM_45
#define PIN_R1  48  // GPIO_NUM_48
#define PIN_R2  47  // GPIO_NUM_47
#define PIN_R3  21  // GPIO_NUM_21
#define PIN_R4  14  // GPIO_NUM_14

// Control pins
#define PIN_HSYNC       39  // GPIO_NUM_39
#define PIN_VSYNC       41  // GPIO_NUM_41
#define PIN_DE          40  // GPIO_NUM_40
#define PIN_PCLK        42  // GPIO_NUM_42

// Backlight control
#define PIN_BACKLIGHT   2   // GPIO_NUM_2

// ============================================================================
// Display Timing Parameters (from Elecrow wiki)
// ============================================================================

// Pixel clock 
#define DISPLAY_FREQ_WRITE      8000000

// HSYNC timing
#define HSYNC_POLARITY          0
#define HSYNC_FRONT_PORCH       8
#define HSYNC_PULSE_WIDTH       4
#define HSYNC_BACK_PORCH        43

// VSYNC timing
#define VSYNC_POLARITY          0
#define VSYNC_FRONT_PORCH       8
#define VSYNC_PULSE_WIDTH       4
#define VSYNC_BACK_PORCH        12

// Clock settings
#define PCLK_ACTIVE_NEG         1
#define DE_IDLE_HIGH            0
#define PCLK_IDLE_HIGH          0

// ============================================================================
// Touch Controller (XPT2046 - Resistive, SPI-based)
// ============================================================================

#define TOUCH_TYPE_XPT2046      1
#define TOUCH_SPI_HOST          SPI2_HOST

#define TOUCH_SPI_SCK           12  // GPIO_NUM_12 (shared with SD)
#define TOUCH_SPI_MOSI          11  // GPIO_NUM_11 (shared with SD)
#define TOUCH_SPI_MISO          13  // GPIO_NUM_13 (shared with SD)
#define TOUCH_SPI_CS            0   // GPIO_NUM_0
#define TOUCH_INT_PIN           36  // GPIO_NUM_36
#define TOUCH_SPI_FREQ          1000000

// Calibration values (raw ADC range)
#define TOUCH_X_MIN             100
#define TOUCH_X_MAX             4000
#define TOUCH_Y_MIN             100
#define TOUCH_Y_MAX             4000

// ============================================================================
// UART Communication Link to CAN Interface Node
// ============================================================================

// UART1 on dedicated HY2.0-4P connector - matches CAN_Interface pins exactly!
#define UART_COMM_TX_PIN        17  // GPIO_NUM_17 (UART1_TX) - to CAN_Interface RX (GPIO18)
#define UART_COMM_RX_PIN        18  // GPIO_NUM_18 (UART1_RX) - from CAN_Interface TX (GPIO17)
#define UART_COMM_BAUD          2000000
#define UART_COMM_PORT          UART_NUM_1

// ============================================================================
// GPIO Expansion Header
// ============================================================================

// Available digital I/O (HY2.0-4P connector)
#define GPIO_D1                 37  // GPIO_NUM_37
#define GPIO_D2                 38  // GPIO_NUM_38

// ============================================================================
// I2S Audio Pins
// ============================================================================

#define I2S_MCLK_PIN            19  // GPIO_NUM_19
#define I2S_BCLK_PIN            35  // GPIO_NUM_35
#define I2S_SDIN_PIN            20  // GPIO_NUM_20

// ============================================================================
// SD Card SPI Pins (shares SPI bus with touch)
// ============================================================================

#define SD_MOSI_PIN             11  // GPIO_NUM_11
#define SD_MISO_PIN             13  // GPIO_NUM_13
#define SD_CLK_PIN              12  // GPIO_NUM_12
#define SD_CS_PIN               10  // GPIO_NUM_10

// ============================================================================
// LVGL Buffer Configuration
// ============================================================================

#define LVGL_BUFFER_FACTOR      5   // screenWidth * screenHeight / factor

// ============================================================================
// Device Capabilities
// ============================================================================

#define HAS_TOUCH_CONTROLLER    1
#define HAS_BACKLIGHT_CONTROL   1
#define SUPPORTS_ROTATION       1
#define HAS_AUDIO               1
#define HAS_SD_CARD             1
#define MAX_BRIGHTNESS          255
#define HAS_PSRAM               1
#define FLASH_SIZE_MB           4
#define PSRAM_SIZE_MB           2
