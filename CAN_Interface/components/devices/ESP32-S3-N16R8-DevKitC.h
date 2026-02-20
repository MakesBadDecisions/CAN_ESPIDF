/**
 * @file ESP32-S3-N16R8-DevKitC.h
 * @brief ESP32-S3-DevKitC-1 (N16R8) Board Configuration
 *
 * Hardware-specific pin definitions and parameters for the ESP32-S3-N16R8
 * DevKitC-1 v1.1 development board used as the CAN Interface Node.
 *
 * Module: ESP32-S3-WROOM-1 (N16R8 marking)
 *   - 16MB Quad SPI Flash (verify with device_test)
 *   - 8MB Octal SPI PSRAM (verify with device_test)
 *   - 240MHz dual-core Xtensa LX7
 *
 * Reference: https://docs.espressif.com/projects/esp-dev-kits/en/latest/
 *            esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html
 *
 * IMPORTANT: GPIO 35, 36, 37 are reserved for internal Octal SPI PSRAM
 * communication and are NOT available for external use on this board.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Device Identification
// ============================================================================

#define DEVICE_NAME             "ESP32-S3-N16R8-DevKitC"
#define DEVICE_MODEL            "DevKitC-1_v1.1"
#define DEVICE_MCU              "ESP32-S3"

// ============================================================================
// Memory Configuration (verify with device_test utility)
// ============================================================================

#define FLASH_SIZE_MB           16
#define PSRAM_SIZE_MB           8
#define HAS_PSRAM               1

// ============================================================================
// Reserved GPIOs - DO NOT USE
// ============================================================================
// GPIO 35, 36, 37: Octal SPI flash/PSRAM internal bus
// GPIO 43, 44: UART0 TX/RX (USB-UART bridge, used for debug logging)

// ============================================================================
// On-Board Peripherals
// ============================================================================

// RGB LED (WS2812 / addressable, active on v1.1 boards)
#define PIN_RGB_LED             38  // GPIO_NUM_38

// Boot button (directly usable as input when not flashing)
#define PIN_BOOT_BUTTON         0   // GPIO_NUM_0

// ============================================================================
// UART0 - Debug / Console (USB-UART Bridge)
// ============================================================================

#define PIN_UART0_TX            43  // GPIO_NUM_43 (default, via USB-UART bridge)
#define PIN_UART0_RX            44  // GPIO_NUM_44 (default, via USB-UART bridge)

// ============================================================================
// UART1 - Inter-Node Communication (to Display Node via USB-C cable)
// ============================================================================
// Directly wired to USB-C D+/D- lines on custom breakout adapter.
// Baud rate: 2Mbps recommended, hardware supports up to 5Mbps.

#define PIN_UART1_TX            17  // GPIO_NUM_17
#define PIN_UART1_RX            18  // GPIO_NUM_18

// ============================================================================
// SPI2 (FSPI) - CAN Controller SPI Bus
// ============================================================================
// Shared SPI bus for all CAN SPI devices. Phase 1 uses MCP2515 on CS=10/INT=9.
// Phase 7 swaps to Waveshare 2-CH CAN FD HAT (MCP2518FD x2) on the same pins.

#define PIN_SPI2_MOSI           11  // GPIO_NUM_11
#define PIN_SPI2_MISO           13  // GPIO_NUM_13
#define PIN_SPI2_SCK            12  // GPIO_NUM_12

// MCP2515 (Phase 1 - standalone CAN 2.0B controller)
#define PIN_MCP2515_CS          10  // GPIO_NUM_10 (same pin as CAN FD CH0 CS)
#define PIN_MCP2515_INT         9   // GPIO_NUM_9  (same pin as CAN FD CH0 INT)

// CAN FD Channel 0 (Phase 7 - Waveshare HAT, primary OBD-II bus)
#define PIN_CAN_FD_CH0_CS       10  // GPIO_NUM_10
#define PIN_CAN_FD_CH0_INT      9   // GPIO_NUM_9  (active-low interrupt)

// CAN FD Channel 1 (Phase 7 - Waveshare HAT, secondary bus / GM extended)
#define PIN_CAN_FD_CH1_CS       46  // GPIO_NUM_46
#define PIN_CAN_FD_CH1_INT      3   // GPIO_NUM_3  (active-low interrupt)

// ============================================================================
// TWAI - Native CAN Peripheral (reserved for 1-Wire GM legacy)
// ============================================================================
// Directly connected to a CAN transceiver (SN65HVD230 or TJA1050).
// Used for legacy 1-wire GM networks via adapter circuit.

#define PIN_TWAI_TX             4   // GPIO_NUM_4
#define PIN_TWAI_RX             5   // GPIO_NUM_5

// ============================================================================
// Available GPIOs (confirmed free for future use)
// ============================================================================
// J1 header: GPIO 6, 7, 8, 14, 15, 16
// J3 header: GPIO 1, 2, 19, 20, 21, 39, 40, 41, 42, 45, 47, 48
//
// NOTE: GPIO 19/20 are also USB D+/D- for the ESP32-S3 native USB port.
// If native USB is not used, they are available as general GPIO.

// Status LED (optional, external)
#define PIN_STATUS_LED          48  // GPIO_NUM_48

// ============================================================================
// Device Capabilities
// ============================================================================

#define HAS_RGB_LED             1
#define HAS_BOOT_BUTTON         1
#define HAS_USB_UART_BRIDGE     1
#define HAS_NATIVE_USB          1   // ESP32-S3 USB OTG on GPIO 19/20

// ============================================================================
// SPI Bus Configuration Constants
// ============================================================================

#define CAN_SPI_HOST            SPI2_HOST
#define MCP2515_SPI_CLOCK_HZ   10000000    // 10 MHz max for MCP2515
#define MCP2515_CRYSTAL_HZ     8000000     // 8 MHz crystal (verify on module)
#define CAN_FD_SPI_CLOCK_HZ    20000000    // 20 MHz max for MCP2518FD
#define MCP2518FD_CRYSTAL_HZ   40000000    // 40 MHz crystal (Waveshare HAT)

// ============================================================================
// UART1 Configuration Constants
// ============================================================================

#define INTERNODE_UART_NUM      UART_NUM_1
#define INTERNODE_UART_BAUD     2000000     // 2 Mbps
