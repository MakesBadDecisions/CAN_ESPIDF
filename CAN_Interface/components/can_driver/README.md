# can_driver - CAN Hardware Abstraction Layer

## Purpose

Provides a unified interface for sending and receiving CAN frames regardless of the underlying hardware. The CAN interface node supports multiple CAN backends simultaneously:

- **MCP2515** - Phase 1 CAN backend. Standalone SPI-based CAN 2.0B controller, proven reliable from the Arduino project. Single channel on SPI2 (CS=GPIO 10, INT=GPIO 9). Vehicle-tested (Ford F-150, 500kbps, 49 PIDs verified).
- **MCP2518FD** - **Current primary backend.** Drop-in replacement for MCP2515 on the same SPI2 bus and CS/INT pins. Runs in CAN 2.0 classic mode for OBD-II compatibility. FIFO-based TX/RX architecture, interrupt-driven, 20MHz SPI clock, 40MHz crystal. Full implementation complete — pending vehicle validation.
- **TWAI** - ESP32-S3 native CAN peripheral. Reserved for 1-Wire GM legacy single-wire network interface via adapter circuit.

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

## MCP2515 Backend Details (Phase 1, Retained)

Standalone MCP2515 SPI CAN controller. Proven from the Arduino project. Superseded by MCP2518FD as primary backend but retained for reference and fallback.

- **Interface:** SPI2 (FSPI) at 8-10 MHz
- **Crystal:** 8 MHz or 16 MHz (must match config)
- **CAN 2.0B:** Standard/extended frames at 5kbps - 1Mbps
- **Filters:** 2 masks, 6 filters (hardware acceptance filtering)
- **TX Buffers:** 3 transmit buffers with priority
- **RX Buffers:** 2 receive buffers with rollover
- **Interrupt:** Active-low INT pin for RX ready / error / TX complete
- **Pins:** CS=GPIO 10, INT=GPIO 9 (same pins as MCP2518FD CH0 for easy swap)

## MCP2518FD Backend Details (Current Primary)

Waveshare 2-CH CAN FD HAT with MCP2518FD controller on SPI2 bus. Drop-in replacement for MCP2515 — same CS=GPIO10, INT=GPIO9 pins, same HAL API.

**Architecture:**
- FIFO-based TX/RX in 2KB device RAM (not fixed buffers like MCP2515)
- FIFO1 = TX (8 message deep, 8-byte payload, unlimited retransmit)
- FIFO2 = RX (16 message deep, 8-byte payload, overflow + not-empty interrupts)
- Filter 0 accepts all frames → routes to FIFO2
- No TEF (Transmit Event FIFO) — unnecessary for OBD-II polling
- ISR task pattern: GPIO falling edge → vTaskNotifyGive → drain RX FIFO → FreeRTOS queue

**SPI Protocol:**
- 16-bit big-endian command word: instruction (4 bits) | 12-bit register address
- Register data is little-endian on the wire (4-byte word-aligned)
- Instructions: RESET=0x0000, WRITE=0x2000|addr, READ=0x3000|addr
- SPI2 (FSPI) at 20MHz, Mode 0,0

**Specifications:**
- **Interface:** SPI2 (FSPI) at 20 MHz
- **Crystal:** 40 MHz (configurable, 20 MHz also supported)
- **CAN Mode:** Classic CAN 2.0 (REQOP mode 6) for OBD-II; CAN-FD capable for future use
- **Bitrates:** 125kbps, 250kbps, 500kbps pre-computed (~80% sample point)
- **Channels:** 2 independent channels with separate CS/INT (CH0: CS=10/INT=9, CH1: CS=46/INT=3)
- **Isolation:** Galvanic isolation on both channels (capacitive coupling)
- **Protection:** TVS diodes for surge/ESD protection
- **Termination:** 120-ohm termination resistor with solder jumper (per channel)

**Implementation Files:**
- `mcp2518fd_defs.h` — Complete register map (CiCON, CiNBTCFG, CiINT, FIFO, filter, OSC, message objects)
- `mcp2518fd.h` — Config struct and API declarations
- `mcp2518fd.c` — Full driver (~600 lines): SPI primitives, mode control, bitrate config, FIFO/filter setup, TX/RX frame packing, ISR task, init/deinit with error cleanup chain

## MCP2515 Backend Details (Phase 1, Retained)

Standalone MCP2515 SPI CAN controller. Proven from the Arduino project. Superseded by MCP2518FD as primary backend but retained for reference and fallback.

- **Interface:** SPI2 (FSPI) at 8-10 MHz
- **Crystal:** 8 MHz or 16 MHz (must match config)
- **CAN 2.0B:** Standard/extended frames at 5kbps - 1Mbps
- **Filters:** 2 masks, 6 filters (hardware acceptance filtering)
- **TX Buffers:** 3 transmit buffers with priority
- **RX Buffers:** 2 receive buffers with rollover
- **Interrupt:** Active-low INT pin for RX ready / error / TX complete
- **Pins:** CS=GPIO 10, INT=GPIO 9 (same pins as MCP2518FD CH0)

## TWAI Backend Details (Phase 2 / 1-Wire GM Legacy)

ESP32-S3's built-in CAN controller. Reserved for 1-Wire GM legacy single-wire networks via adapter circuit. Requires an external CAN transceiver (SN65HVD230/TJA1050).

- **Baud rates:** 25kbps - 1Mbps (33.3kbps typical for GM single-wire)
- **Filters:** Hardware acceptance filtering (single or dual filter mode)
- **Alerts:** TX complete, RX ready, bus error, bus off, etc.
- **RX buffer:** Configurable (default 32 frames)

## Configuration

Pin assignments are defined in `components/devices/ESP32-S3-N16R8-DevKitC.h`. Backend selection via menuconfig (Kconfig) or compile-time defines:

```
# MCP2518FD (Current Primary -- Waveshare 2-CH CAN FD HAT)
CONFIG_CAN_MCP2518FD_ENABLED=y
CONFIG_CAN_MCP2518FD_SPI_HOST=SPI2_HOST
CONFIG_CAN_MCP2518FD_SPI_CLOCK_HZ=20000000
CONFIG_CAN_MCP2518FD_CRYSTAL_HZ=40000000
CONFIG_CAN_MCP2518FD_CH0_CS_GPIO=10
CONFIG_CAN_MCP2518FD_CH0_INT_GPIO=9
CONFIG_CAN_MCP2518FD_CH1_CS_GPIO=46
CONFIG_CAN_MCP2518FD_CH1_INT_GPIO=3

# MCP2515 (Phase 1 -- retained as fallback)
CONFIG_CAN_MCP2515_ENABLED=n

# TWAI (Phase 2 - HAL validation + 1-Wire GM legacy)
CONFIG_CAN_TWAI_ENABLED=n
CONFIG_CAN_TWAI_TX_GPIO=4
CONFIG_CAN_TWAI_RX_GPIO=5
CONFIG_CAN_TWAI_BAUD=33300
```

## Files

```
can_driver/
├── README.md               # This file
├── CMakeLists.txt           # Component build config
├── can_driver.c             # HAL dispatch layer (backend selection, config mapping)
├── can_driver.h             # Public API, can_config_t, can_frame_t
├── mcp2515.c                # MCP2515 SPI backend (Phase 1, retained)
├── mcp2515.h                # MCP2515 config and API
├── mcp2515_defs.h           # MCP2515 register map
├── mcp2518fd.c              # MCP2518FD SPI backend (current primary)
├── mcp2518fd.h              # MCP2518FD config and API
└── mcp2518fd_defs.h         # MCP2518FD register map, FIFO defs, bit timing tables
```

## Dependencies

- `driver` (ESP-IDF TWAI driver, SPI master)
- `esp_timer` (timestamps)
- `system` component (logging)
