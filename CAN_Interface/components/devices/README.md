# devices/ - Hardware Board Definitions (CAN Interface)

## Purpose

Contains hardware-specific configuration headers for each board variant used
as the CAN Interface Node. Each header defines pin assignments, memory sizes,
peripheral mappings, and capability flags for a specific development board.

## How It Works

The build includes exactly one device header based on the target board. All
component code references the generic `PIN_*` and `HAS_*` defines rather than
hard-coded GPIO numbers, making it easy to swap boards without modifying
application code.

```c
#include "ESP32-S3-N16R8-DevKitC.h"

// All components use the same defines:
// PIN_SPI2_MOSI, PIN_CAN_FD_CH0_CS, PIN_UART1_TX, etc.
```

## Board Files

| File | Board | MCU | Flash | PSRAM | Role |
|------|-------|-----|-------|-------|------|
| `ESP32-S3-N16R8-DevKitC.h` | ESP32-S3-DevKitC-1 v1.1 | ESP32-S3 | 16MB | 8MB Octal | CAN Interface Node |

## Adding a New Board

1. Create a new `.h` file named after the board (e.g., `ESP32-S3-N8R8-DevKitC.h`)
2. Follow the existing section layout:
   - Device Identification
   - Memory Configuration
   - Reserved GPIOs
   - On-Board Peripherals
   - UART pins (debug + inter-node)
   - SPI pins (CAN FD HAT)
   - TWAI pins
   - Available GPIOs
   - Device Capabilities
   - Configuration Constants
3. Update the build system to select the correct header

## Device Verification

Each new board should be verified using the device test utility before
deployment. The test checks:
- Flash size matches `FLASH_SIZE_MB`
- PSRAM size matches `PSRAM_SIZE_MB`
- Chip model and revision
- MAC address
- GPIO pin accessibility (reserved pins not conflicting)

## Pin Assignment Rules

- **GPIO 35, 36, 37** are reserved on all Octal PSRAM boards (N8R8, N16R8, etc.)
- **GPIO 43, 44** are the default UART0 pins (USB-UART bridge) -- avoid reassigning
- **GPIO 0** is the BOOT button strapping pin -- usable as input after boot
- Verify pin availability against the specific module datasheet before assigning
