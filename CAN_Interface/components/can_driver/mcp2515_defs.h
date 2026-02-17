/**
 * @file mcp2515_defs.h
 * @brief MCP2515 Register Definitions and Constants
 */

#pragma once

#include <stdint.h>

// ============================================================================
// SPI Instructions
// ============================================================================

#define MCP2515_RESET           0xC0
#define MCP2515_READ            0x03
#define MCP2515_WRITE           0x02
#define MCP2515_READ_RX0        0x90    // Read RX buffer 0 starting at RXB0SIDH
#define MCP2515_READ_RX1        0x94    // Read RX buffer 1 starting at RXB1SIDH
#define MCP2515_LOAD_TX0        0x40    // Load TX buffer 0 starting at TXB0SIDH
#define MCP2515_LOAD_TX1        0x42    // Load TX buffer 1 starting at TXB1SIDH
#define MCP2515_LOAD_TX2        0x44    // Load TX buffer 2 starting at TXB2SIDH
#define MCP2515_RTS_TX0         0x81    // Request to send TX buffer 0
#define MCP2515_RTS_TX1         0x82    // Request to send TX buffer 1
#define MCP2515_RTS_TX2         0x84    // Request to send TX buffer 2
#define MCP2515_RTS_ALL         0x87    // Request to send all TX buffers
#define MCP2515_READ_STATUS     0xA0
#define MCP2515_RX_STATUS       0xB0
#define MCP2515_BIT_MODIFY      0x05

// ============================================================================
// Registers
// ============================================================================

// Configuration registers (in config mode only)
#define MCP2515_CNF1            0x2A
#define MCP2515_CNF2            0x2B
#define MCP2515_CNF3            0x2C

// Control registers
#define MCP2515_CANCTRL         0x0F
#define MCP2515_CANSTAT         0x0E

// Interrupt registers
#define MCP2515_CANINTE         0x2B
#define MCP2515_CANINTF         0x2C

// Error registers
#define MCP2515_TEC             0x1C    // TX error counter
#define MCP2515_REC             0x1D    // RX error counter
#define MCP2515_EFLG            0x2D    // Error flag register

// TX Buffer 0
#define MCP2515_TXB0CTRL        0x30
#define MCP2515_TXB0SIDH        0x31
#define MCP2515_TXB0SIDL        0x32
#define MCP2515_TXB0EID8        0x33
#define MCP2515_TXB0EID0        0x34
#define MCP2515_TXB0DLC         0x35
#define MCP2515_TXB0D0          0x36

// TX Buffer 1
#define MCP2515_TXB1CTRL        0x40
#define MCP2515_TXB1SIDH        0x41

// TX Buffer 2
#define MCP2515_TXB2CTRL        0x50
#define MCP2515_TXB2SIDH        0x51

// RX Buffer 0
#define MCP2515_RXB0CTRL        0x60
#define MCP2515_RXB0SIDH        0x61
#define MCP2515_RXB0SIDL        0x62
#define MCP2515_RXB0EID8        0x63
#define MCP2515_RXB0EID0        0x64
#define MCP2515_RXB0DLC         0x65
#define MCP2515_RXB0D0          0x66

// RX Buffer 1
#define MCP2515_RXB1CTRL        0x70
#define MCP2515_RXB1SIDH        0x71

// Filter and mask registers
#define MCP2515_RXF0SIDH        0x00
#define MCP2515_RXF0SIDL        0x01
#define MCP2515_RXF1SIDH        0x04
#define MCP2515_RXF2SIDH        0x08
#define MCP2515_RXF3SIDH        0x10
#define MCP2515_RXF4SIDH        0x14
#define MCP2515_RXF5SIDH        0x18
#define MCP2515_RXM0SIDH        0x20
#define MCP2515_RXM0SIDL        0x21
#define MCP2515_RXM1SIDH        0x24
#define MCP2515_RXM1SIDL        0x25

// ============================================================================
// CANCTRL Register Bits
// ============================================================================

#define CANCTRL_REQOP_MASK      0xE0
#define CANCTRL_REQOP_NORMAL    0x00
#define CANCTRL_REQOP_SLEEP     0x20
#define CANCTRL_REQOP_LOOPBACK  0x40
#define CANCTRL_REQOP_LISTENONLY 0x60
#define CANCTRL_REQOP_CONFIG    0x80
#define CANCTRL_ABAT            0x10    // Abort all pending TX
#define CANCTRL_OSM             0x08    // One-shot mode
#define CANCTRL_CLKEN           0x04    // CLKOUT enable
#define CANCTRL_CLKPRE_MASK     0x03    // CLKOUT prescaler

// ============================================================================
// CANSTAT Register Bits
// ============================================================================

#define CANSTAT_OPMOD_MASK      0xE0
#define CANSTAT_OPMOD_NORMAL    0x00
#define CANSTAT_OPMOD_SLEEP     0x20
#define CANSTAT_OPMOD_LOOPBACK  0x40
#define CANSTAT_OPMOD_LISTENONLY 0x60
#define CANSTAT_OPMOD_CONFIG    0x80

// ============================================================================
// CANINTE/CANINTF Register Bits (Interrupts)
// ============================================================================

#define CANINT_RX0I             0x01    // RX buffer 0 full
#define CANINT_RX1I             0x02    // RX buffer 1 full
#define CANINT_TX0I             0x04    // TX buffer 0 empty
#define CANINT_TX1I             0x08    // TX buffer 1 empty
#define CANINT_TX2I             0x10    // TX buffer 2 empty
#define CANINT_ERRI             0x20    // Error interrupt
#define CANINT_WAKI             0x40    // Wake-up interrupt
#define CANINT_MERR             0x80    // Message error interrupt

// ============================================================================
// TXBnCTRL Register Bits
// ============================================================================

#define TXBCTRL_TXREQ           0x08    // TX request
#define TXBCTRL_TXP_MASK        0x03    // TX priority

// ============================================================================
// RXBnCTRL Register Bits
// ============================================================================

#define RXBCTRL_RXM_MASK        0x60    // Receive buffer operating mode
#define RXBCTRL_RXM_ANY         0x60    // Receive any message
#define RXBCTRL_RXM_EXT         0x40    // Receive extended ID only
#define RXBCTRL_RXM_STD         0x20    // Receive standard ID only
#define RXBCTRL_RXM_FILTER      0x00    // Receive messages matching filters
#define RXBCTRL_BUKT            0x04    // Rollover enable (RXB0 only)

// ============================================================================
// EFLG Register Bits (Error Flags)
// ============================================================================

#define EFLG_RX1OVR             0x80    // RX buffer 1 overflow
#define EFLG_RX0OVR             0x40    // RX buffer 0 overflow
#define EFLG_TXBO               0x20    // Bus-off error
#define EFLG_TXEP               0x10    // TX error-passive
#define EFLG_RXEP               0x08    // RX error-passive
#define EFLG_TXWAR              0x04    // TX error warning
#define EFLG_RXWAR              0x02    // RX error warning
#define EFLG_EWARN              0x01    // Error warning

// ============================================================================
// Bit Timing for common clock/bitrate combinations
// MCP2515 with 8MHz crystal (common module configuration)
// ============================================================================

// CNF1, CNF2, CNF3 for 500kbps @ 8MHz crystal
#define MCP2515_8MHZ_500KBPS_CNF1   0x00
#define MCP2515_8MHZ_500KBPS_CNF2   0x90
#define MCP2515_8MHZ_500KBPS_CNF3   0x02

// CNF1, CNF2, CNF3 for 250kbps @ 8MHz crystal
#define MCP2515_8MHZ_250KBPS_CNF1   0x00
#define MCP2515_8MHZ_250KBPS_CNF2   0xB1
#define MCP2515_8MHZ_250KBPS_CNF3   0x05

// CNF1, CNF2, CNF3 for 125kbps @ 8MHz crystal
#define MCP2515_8MHZ_125KBPS_CNF1   0x01
#define MCP2515_8MHZ_125KBPS_CNF2   0xB1
#define MCP2515_8MHZ_125KBPS_CNF3   0x05

// ============================================================================
// MCP2515 with 16MHz crystal
// ============================================================================

// CNF1, CNF2, CNF3 for 500kbps @ 16MHz crystal
#define MCP2515_16MHZ_500KBPS_CNF1  0x00
#define MCP2515_16MHZ_500KBPS_CNF2  0xF0
#define MCP2515_16MHZ_500KBPS_CNF3  0x06

// CNF1, CNF2, CNF3 for 250kbps @ 16MHz crystal
#define MCP2515_16MHZ_250KBPS_CNF1  0x01
#define MCP2515_16MHZ_250KBPS_CNF2  0xF0
#define MCP2515_16MHZ_250KBPS_CNF3  0x06

// CNF1, CNF2, CNF3 for 125kbps @ 16MHz crystal
#define MCP2515_16MHZ_125KBPS_CNF1  0x03
#define MCP2515_16MHZ_125KBPS_CNF2  0xF0
#define MCP2515_16MHZ_125KBPS_CNF3  0x06

