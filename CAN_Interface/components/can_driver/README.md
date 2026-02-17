# can_driver - CAN Hardware Abstraction Layer

## Purpose

Provides a unified interface for sending and receiving CAN frames regardless of the underlying hardware. The CAN interface node supports multiple CAN backends simultaneously:

- **MCP2515** - Phase 1 primary CAN backend. Standalone SPI-based CAN 2.0B controller, proven reliable from the Arduino project. Single channel on SPI2 (CS=GPIO 10, INT=GPIO 9). Gets the full OBD-II stack working before adding hardware complexity.
- **TWAI** - ESP32-S3 native CAN peripheral. Added in Phase 2 to validate the HAL abstraction with a second backend. Also reserved for 1-Wire GM legacy single-wire network interface via adapter circuit.
- **MCP2518FD** - Phase 7 CAN-FD backend. Two channels via Waveshare 2-CH CAN FD HAT over SPI2. Supports CAN 2.0B and CAN-FD (up to 8Mbps data phase). Provides galvanic isolation, TVS surge protection, and switchable 120-ohm termination. Drops in behind the same HAL built in Phase 1 (similar SPI command set to MCP2515).

## Architecture

```
+---------------------------------------------+
|         Application Layer                    |
|  (obd2, poll_engine, diagnostics)            |
+------------------+---------------------------+
                   |  can_driver API
+------------------+---------------------------+
|         can_driver.h                         |
|  - can_driver_init()                         |
|  - can_driver_send()                         |
|  - can_driver_receive()                      |
|  - can_driver_set_filter()                   |
|  - can_driver_get_status()                   |
+--+----------+-----------+--------------------+
   |          |           |
+--+------+ +-+-------+ +-+--------+
| MCP2515 | |  TWAI   | |MCP2518FD|
| back    | | back    | | back    |
| end     | | end     | | end     |
|(Phase 1)| |(Phase 2 | |(Phase 7 |
| primary | | + GM)   | | CAN-FD) |
+---------+ +---------+ +----------+
```

## API Design

The HAL uses a backend registration pattern. Each backend implements a common `can_backend_t` interface:

```c
typedef struct {
    const char *name;
    esp_err_t (*init)(const can_backend_config_t *config);
    esp_err_t (*deinit)(void);
    esp_err_t (*send)(const can_frame_t *frame, uint32_t timeout_ms);
    esp_err_t (*receive)(can_frame_t *frame, uint32_t timeout_ms);
    esp_err_t (*set_filter)(uint32_t id, uint32_t mask);
    esp_err_t (*get_status)(can_bus_status_t *status);
    esp_err_t (*set_mode)(can_mode_t mode);
} can_backend_ops_t;
```

The driver supports multiple active backends (e.g., MCP2518FD CH0 + CH1 on different buses). Each is identified by a `can_bus_id_t`.

## Key Types

```c
// CAN frame - common to all backends
typedef struct {
    uint32_t id;            // CAN ID (11-bit standard or 29-bit extended)
    uint8_t  data[64];      // Payload (8 bytes CAN 2.0, up to 64 bytes CAN-FD)
    uint8_t  dlc;           // Data length code (0-8, 0-64 for FD)
    bool     extended;      // Extended frame flag
    bool     rtr;           // Remote transmission request
    bool     fd;            // CAN-FD flag
    bool     brs;           // Bit rate switch (FD data phase at higher rate)
    int64_t  timestamp_us;  // Receive timestamp (esp_timer_get_time)
} can_frame_t;

// Bus identification for multi-bus operation
typedef enum {
    CAN_BUS_PRIMARY = 0,    // MCP2518FD Channel 0 (primary OBD-II bus)
    CAN_BUS_SECONDARY,      // MCP2518FD Channel 1 (secondary / GM extended)
    CAN_BUS_MAX
} can_bus_id_t;

// Bus status
typedef struct {
    can_bus_state_t state;  // OK, ERROR_ACTIVE, ERROR_PASSIVE, BUS_OFF
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t msgs_sent;
    uint32_t msgs_received;
    uint32_t errors;
} can_bus_status_t;
```

## MCP2515 Backend Details (Phase 1 Primary)

Standalone MCP2515 SPI CAN controller. Proven from the Arduino project where it was the primary CAN interface. Uses the same SPI2 bus and pin assignments that the MCP2518FD will later use, enabling a seamless hardware swap.

- **Interface:** SPI2 (FSPI) at 8-10 MHz
- **Crystal:** 8 MHz or 16 MHz (must match config)
- **CAN 2.0B:** Standard/extended frames at 5kbps - 1Mbps
- **Filters:** 2 masks, 6 filters (hardware acceptance filtering)
- **TX Buffers:** 3 transmit buffers with priority
- **RX Buffers:** 2 receive buffers with rollover
- **Interrupt:** Active-low INT pin for RX ready / error / TX complete
- **Pins:** CS=GPIO 10, INT=GPIO 9 (same pins as MCP2518FD CH0 for easy swap)

## MCP2518FD Backend Details (Phase 7)

Waveshare 2-CH CAN FD HAT with two MCP2518FD controllers on a shared SPI2 bus. Drop-in replacement behind the same HAL -- similar SPI register access pattern to MCP2515.

- **Interface:** SPI2 (FSPI) at up to 20MHz
- **Channels:** 2 independent CAN-FD channels with separate CS and INT lines
- **CAN 2.0B:** Standard/extended frames at 125kbps - 1Mbps
- **CAN-FD:** Up to 8Mbps data phase, 64-byte payload
- **Filters:** Configurable hardware acceptance filters per channel
- **Isolation:** Galvanic isolation on both channels (capacitive coupling)
- **Protection:** TVS diodes for surge/ESD protection
- **Termination:** 120-ohm termination resistor with solder jumper (per channel)
- **Interrupt:** Active-low INT pin per channel for RX ready / error

## TWAI Backend Details (Phase 2 / 1-Wire GM Legacy)

ESP32-S3's built-in CAN controller. Reserved for 1-Wire GM legacy single-wire networks via adapter circuit. Requires an external CAN transceiver (SN65HVD230/TJA1050).

- **Baud rates:** 25kbps - 1Mbps (33.3kbps typical for GM single-wire)
- **Filters:** Hardware acceptance filtering (single or dual filter mode)
- **Alerts:** TX complete, RX ready, bus error, bus off, etc.
- **RX buffer:** Configurable (default 32 frames)

## Configuration

Pin assignments are defined in `components/devices/ESP32-S3-N16R8-DevKitC.h`. Backend selection via menuconfig (Kconfig) or compile-time defines:

```
# MCP2515 (Phase 1 Primary)
CONFIG_CAN_MCP2515_ENABLED=y
CONFIG_CAN_MCP2515_SPI_HOST=SPI2_HOST
CONFIG_CAN_MCP2515_SPI_CLOCK_HZ=10000000
CONFIG_CAN_MCP2515_MOSI_GPIO=11
CONFIG_CAN_MCP2515_MISO_GPIO=13
CONFIG_CAN_MCP2515_SCK_GPIO=12
CONFIG_CAN_MCP2515_CS_GPIO=10
CONFIG_CAN_MCP2515_INT_GPIO=9
CONFIG_CAN_MCP2515_CRYSTAL_HZ=8000000

# TWAI (Phase 2 - HAL validation + 1-Wire GM legacy)
CONFIG_CAN_TWAI_ENABLED=n
CONFIG_CAN_TWAI_TX_GPIO=4
CONFIG_CAN_TWAI_RX_GPIO=5
CONFIG_CAN_TWAI_BAUD=33300

# MCP2518FD (Phase 7 - Waveshare 2-CH CAN FD HAT)
CONFIG_CAN_MCP2518FD_ENABLED=n
CONFIG_CAN_MCP2518FD_SPI_HOST=SPI2_HOST
CONFIG_CAN_MCP2518FD_SPI_CLOCK_HZ=20000000
CONFIG_CAN_MCP2518FD_CH0_CS_GPIO=10
CONFIG_CAN_MCP2518FD_CH0_INT_GPIO=9
CONFIG_CAN_MCP2518FD_CH1_CS_GPIO=46
CONFIG_CAN_MCP2518FD_CH1_INT_GPIO=3
```

## Files

```
can_driver/
├── README.md               # This file
├── CMakeLists.txt           # Component build config
├── Kconfig                  # Menuconfig options
├── include/
│   ├── can_driver.h         # Public API
│   ├── can_types.h          # Frame types, enums, status
│   └── can_backend.h        # Backend interface definition
└── src/
    ├── can_driver.c          # Driver manager (registration, dispatch)
    ├── mcp2515_backend.c     # MCP2515 SPI implementation (Phase 1 primary)
    ├── twai_backend.c        # TWAI implementation (Phase 2 / 1-Wire GM)
    └── mcp2518fd_backend.c   # MCP2518FD SPI implementation (Phase 7 CAN-FD)
```

## Dependencies

- `driver` (ESP-IDF TWAI driver, SPI master)
- `esp_timer` (timestamps)
- `system` component (logging)
