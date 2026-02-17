/**
 * @file dis07050.h
 * @brief CrowPanel ESP32 HMI 5.0-inch Display Configuration
 * 
 * Hardware-specific pin definitions and parameters for the DIS07050H display.
 * ESP32-S3-WROOM-1-N4R8 with 5" RGB LCD (800x480).
 * 4MB Flash, 8MB Octal PSRAM
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Device Identification
// ============================================================================

#define DEVICE_NAME         "DIS07050H"
#define DEVICE_MODEL        "5inch_RGB_LCD"

// ============================================================================
// Display Specifications
// ============================================================================

#define DISPLAY_WIDTH       800
#define DISPLAY_HEIGHT      480
#define DISPLAY_COLOR_DEPTH 16   // RGB565

// ============================================================================
// RGB Interface Pin Configuration (DIS07050 specific)
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
#define PIN_PCLK        0   // GPIO_NUM_0

// Backlight control
#define PIN_BACKLIGHT   2   // GPIO_NUM_2

// ============================================================================
// Display Timing Parameters (from Elecrow wiki)
// ============================================================================

// Pixel clock 
#define DISPLAY_FREQ_WRITE      15000000

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
// I2C Configuration
// ============================================================================

#define I2C_SDA_PIN             19  // GPIO_NUM_19
#define I2C_SCL_PIN             20  // GPIO_NUM_20
#define I2C_FREQ_HZ             400000

// Touch controller (GT911)
#define TOUCH_I2C_SDA           I2C_SDA_PIN
#define TOUCH_I2C_SCL           I2C_SCL_PIN
#define TOUCH_I2C_FREQ          I2C_FREQ_HZ
#define TOUCH_I2C_ADDR          0x5D

// ============================================================================
// Hardware Interface Pins
// ============================================================================

// UART Communication Link to CAN Interface Node
// Using UART0 (USB-CDC handles debug logging)
#define UART_COMM_TX_PIN        43  // GPIO_NUM_43 (UART0_TX) - to CAN_Interface RX
#define UART_COMM_RX_PIN        44  // GPIO_NUM_44 (UART0_RX) - from CAN_Interface TX
#define UART_COMM_BAUD          2000000

// Alternative: Repurpose I2C pins (loses touch functionality)
// #define UART_COMM_TX_PIN     20  // GPIO_NUM_20 (SCL)
// #define UART_COMM_RX_PIN     19  // GPIO_NUM_19 (SDA)


// ============================================================================
// I2S Audio Pins (Available, not used in base project)
// ============================================================================

#define I2S_LRCLK_PIN           18  // GPIO_NUM_18
#define I2S_BCLK_PIN            42  // GPIO_NUM_42
#define I2S_SDIN_PIN            17  // GPIO_NUM_17

// ============================================================================
// SD Card SPI Pins (Available, not used in base project)
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
#define HAS_BACKLIGHT_CONTROL   0
#define SUPPORTS_ROTATION       1
#define HAS_AUDIO               1
#define HAS_SD_CARD             1
#define MAX_BRIGHTNESS          255
#define HAS_PSRAM               1
#define FLASH_SIZE_MB           4
#define PSRAM_SIZE_MB           8

