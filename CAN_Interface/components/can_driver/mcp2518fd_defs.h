/**
 * @file mcp2518fd_defs.h
 * @brief MCP2518FD (MCP251863) Register Definitions and Constants
 *
 * Register map and bit definitions for the Microchip MCP2518FD
 * CAN FD controller, derived from the MCP25xxFD Family Reference
 * Manual and verified against the Linux kernel mcp251xfd driver
 * (drivers/net/can/spi/mcp251xfd/).
 *
 * Register data is little-endian on the SPI bus.
 * SPI commands are 16-bit big-endian: (instruction | 12-bit address).
 */

#pragma once

#include <stdint.h>

// ============================================================================
// SPI Instructions (16-bit, sent big-endian)
// ============================================================================

#define MCP2518FD_SPI_RESET             0x0000
#define MCP2518FD_SPI_WRITE             0x2000
#define MCP2518FD_SPI_READ              0x3000
#define MCP2518FD_SPI_WRITE_CRC         0xA000
#define MCP2518FD_SPI_READ_CRC          0xB000
#define MCP2518FD_SPI_WRITE_CRC_SAFE    0xC000
#define MCP2518FD_SPI_ADDR_MASK         0x0FFF  // 12-bit address space

// ============================================================================
// CAN FD Controller Module SFR
// ============================================================================

// --- CiCON: CAN Control Register (0x000) ---
#define MCP2518FD_REG_CON                0x000
#define MCP2518FD_CON_TXBWS_MASK         (0x0FUL << 28)  // TX bandwidth sharing
#define MCP2518FD_CON_ABAT               (1UL << 27)     // Abort all TX
#define MCP2518FD_CON_REQOP_MASK         (0x07UL << 24)  // Request Operation Mode
#define MCP2518FD_CON_REQOP_SHIFT        24
#define MCP2518FD_CON_OPMOD_MASK         (0x07UL << 21)  // Op Mode Status (read-only)
#define MCP2518FD_CON_OPMOD_SHIFT        21
#define MCP2518FD_CON_TXQEN             (1UL << 20)     // Enable TXQ
#define MCP2518FD_CON_STEF              (1UL << 19)     // Store in TEF
#define MCP2518FD_CON_SERR2LOM          (1UL << 18)     // SysErr → Listen Only
#define MCP2518FD_CON_ESIGM             (1UL << 17)     // ESI in gateway mode
#define MCP2518FD_CON_RTXAT             (1UL << 16)     // Restrict TX attempts
#define MCP2518FD_CON_BRSDIS            (1UL << 12)     // BRS disable
#define MCP2518FD_CON_BUSY              (1UL << 11)     // Module busy
#define MCP2518FD_CON_WFT_MASK          (0x03UL << 9)   // Wake-up filter time
#define MCP2518FD_CON_WAKFIL            (1UL << 8)      // Wake-up filter enable
#define MCP2518FD_CON_PXEDIS            (1UL << 6)      // Protocol exception disable
#define MCP2518FD_CON_ISOCRCEN          (1UL << 5)      // ISO CRC enable
#define MCP2518FD_CON_DNCNT_MASK        (0x1FUL << 0)   // DeviceNet filter bits

// CiCON Operation Modes (for REQOP and OPMOD)
#define MCP2518FD_MODE_MIXED            0   // Normal (CAN FD + Classic)
#define MCP2518FD_MODE_SLEEP            1
#define MCP2518FD_MODE_INT_LOOPBACK     2   // Internal Loopback
#define MCP2518FD_MODE_LISTENONLY        3
#define MCP2518FD_MODE_CONFIG           4   // Configuration mode
#define MCP2518FD_MODE_EXT_LOOPBACK     5   // External Loopback
#define MCP2518FD_MODE_CAN20            6   // Normal CAN 2.0 (classic only)
#define MCP2518FD_MODE_RESTRICTED       7

// --- CiNBTCFG: Nominal Bit Time Configuration (0x004) ---
#define MCP2518FD_REG_NBTCFG            0x004
#define MCP2518FD_NBTCFG_BRP_MASK       (0xFFUL << 24)
#define MCP2518FD_NBTCFG_BRP_SHIFT      24
#define MCP2518FD_NBTCFG_TSEG1_MASK     (0xFFUL << 16)
#define MCP2518FD_NBTCFG_TSEG1_SHIFT    16
#define MCP2518FD_NBTCFG_TSEG2_MASK     (0x7FUL << 8)
#define MCP2518FD_NBTCFG_TSEG2_SHIFT    8
#define MCP2518FD_NBTCFG_SJW_MASK       (0x7FUL << 0)
#define MCP2518FD_NBTCFG_SJW_SHIFT      0

// --- CiDBTCFG: Data Bit Time Configuration (0x008) - CAN FD only ---
#define MCP2518FD_REG_DBTCFG            0x008

// --- CiTDC: Transmitter Delay Compensation (0x00C) ---
#define MCP2518FD_REG_TDC               0x00C

// --- CiTBC: Time Base Counter (0x010) ---
#define MCP2518FD_REG_TBC               0x010

// --- CiTSCON: Timestamp Control (0x014) ---
#define MCP2518FD_REG_TSCON             0x014
#define MCP2518FD_TSCON_TBCEN           (1UL << 16)
#define MCP2518FD_TSCON_TBCPRE_MASK     (0x3FFUL << 0)

// --- CiVEC: Interrupt Code / Filter Hit (0x018) ---
#define MCP2518FD_REG_VEC               0x018

// ============================================================================
// Interrupt Registers
// ============================================================================

// --- CiINT: Interrupt Register (0x01C) ---
//   Upper 16 bits = IE (enable), Lower 16 bits = IF (flags)
#define MCP2518FD_REG_INT               0x01C

// Interrupt Enable bits (bits 31:16)
#define MCP2518FD_INT_IVMIE             (1UL << 31)
#define MCP2518FD_INT_WAKIE             (1UL << 30)
#define MCP2518FD_INT_CERRIE            (1UL << 29)
#define MCP2518FD_INT_SERRIE            (1UL << 28)
#define MCP2518FD_INT_RXOVIE            (1UL << 27)
#define MCP2518FD_INT_TXATIE            (1UL << 26)
#define MCP2518FD_INT_SPICRCIE          (1UL << 25)
#define MCP2518FD_INT_ECCIE             (1UL << 24)
#define MCP2518FD_INT_TEFIE             (1UL << 20)
#define MCP2518FD_INT_MODIE             (1UL << 19)
#define MCP2518FD_INT_TBCIE             (1UL << 18)
#define MCP2518FD_INT_RXIE              (1UL << 17)
#define MCP2518FD_INT_TXIE              (1UL << 16)

// Interrupt Flag bits (bits 15:0)
#define MCP2518FD_INT_IVMIF             (1UL << 15)
#define MCP2518FD_INT_WAKIF             (1UL << 14)
#define MCP2518FD_INT_CERRIF            (1UL << 13)
#define MCP2518FD_INT_SERRIF            (1UL << 12)
#define MCP2518FD_INT_RXOVIF            (1UL << 11)
#define MCP2518FD_INT_TXATIF            (1UL << 10)
#define MCP2518FD_INT_SPICRCIF          (1UL << 9)
#define MCP2518FD_INT_ECCIF             (1UL << 8)
#define MCP2518FD_INT_TEFIF             (1UL << 4)
#define MCP2518FD_INT_MODIF             (1UL << 3)
#define MCP2518FD_INT_TBCIF             (1UL << 2)
#define MCP2518FD_INT_RXIF              (1UL << 1)
#define MCP2518FD_INT_TXIF              (1UL << 0)

// Flags that must be cleared by software (write 0 to clear)
#define MCP2518FD_INT_IF_CLEARABLE      \
    (MCP2518FD_INT_IVMIF | MCP2518FD_INT_WAKIF | MCP2518FD_INT_CERRIF | \
     MCP2518FD_INT_SERRIF | MCP2518FD_INT_MODIF)

// --- CiRXIF: RX FIFO Interrupt Flags (0x020) ---
#define MCP2518FD_REG_RXIF              0x020

// --- CiTXIF: TX FIFO Interrupt Flags (0x024) ---
#define MCP2518FD_REG_TXIF              0x024

// --- CiRXOVIF: RX Overflow Interrupt Flags (0x028) ---
#define MCP2518FD_REG_RXOVIF            0x028

// --- CiTXATIF: TX Attempt Interrupt Flags (0x02C) ---
#define MCP2518FD_REG_TXATIF            0x02C

// --- CiTXREQ: TX Request Register (0x030) ---
#define MCP2518FD_REG_TXREQ             0x030

// ============================================================================
// Error Counters / Diagnostics
// ============================================================================

// --- CiTREC: TX/RX Error Count (0x034) ---
#define MCP2518FD_REG_TREC              0x034
#define MCP2518FD_TREC_TXBO             (1UL << 21)  // TX Bus-Off
#define MCP2518FD_TREC_TXBP             (1UL << 20)  // TX Error-Passive
#define MCP2518FD_TREC_RXBP             (1UL << 19)  // RX Error-Passive
#define MCP2518FD_TREC_TXWARN           (1UL << 18)  // TX Error Warning
#define MCP2518FD_TREC_RXWARN           (1UL << 17)  // RX Error Warning
#define MCP2518FD_TREC_EWARN            (1UL << 16)  // Error Warning
#define MCP2518FD_TREC_TEC_MASK         (0xFFUL << 8)
#define MCP2518FD_TREC_TEC_SHIFT        8
#define MCP2518FD_TREC_REC_MASK         (0xFFUL << 0)
#define MCP2518FD_TREC_REC_SHIFT        0

// --- CiBDIAG0: Bus Diagnostic 0 (0x038) ---
#define MCP2518FD_REG_BDIAG0            0x038

// --- CiBDIAG1: Bus Diagnostic 1 (0x03C) ---
#define MCP2518FD_REG_BDIAG1            0x03C

// ============================================================================
// TEF (Transmit Event FIFO) - not used in our configuration
// ============================================================================

#define MCP2518FD_REG_TEFCON            0x040
#define MCP2518FD_REG_TEFSTA            0x044
#define MCP2518FD_REG_TEFUA             0x048

// ============================================================================
// TXQ (Transmit Queue) - not used; we use FIFO1 for TX
// ============================================================================

#define MCP2518FD_REG_TXQCON            0x050
#define MCP2518FD_REG_TXQSTA            0x054
#define MCP2518FD_REG_TXQUA             0x058

// ============================================================================
// FIFO Registers (x = FIFO number 1..31)
// ============================================================================

// Register addresses: base + 0xC * fifo_num
#define MCP2518FD_REG_FIFOCON(x)        (0x050 + 0x0C * (x))
#define MCP2518FD_REG_FIFOSTA(x)        (0x054 + 0x0C * (x))
#define MCP2518FD_REG_FIFOUA(x)         (0x058 + 0x0C * (x))

// CiFIFOCON bit definitions
#define MCP2518FD_FIFOCON_PLSIZE_MASK   (0x07UL << 29)
#define MCP2518FD_FIFOCON_PLSIZE_SHIFT  29
#define MCP2518FD_FIFOCON_PLSIZE_8      0   // 8 bytes (classic CAN)
#define MCP2518FD_FIFOCON_PLSIZE_12     1
#define MCP2518FD_FIFOCON_PLSIZE_16     2
#define MCP2518FD_FIFOCON_PLSIZE_20     3
#define MCP2518FD_FIFOCON_PLSIZE_24     4
#define MCP2518FD_FIFOCON_PLSIZE_32     5
#define MCP2518FD_FIFOCON_PLSIZE_48     6
#define MCP2518FD_FIFOCON_PLSIZE_64     7   // 64 bytes (CAN FD)
#define MCP2518FD_FIFOCON_FSIZE_MASK    (0x1FUL << 24)
#define MCP2518FD_FIFOCON_FSIZE_SHIFT   24
#define MCP2518FD_FIFOCON_TXAT_MASK     (0x03UL << 21)
#define MCP2518FD_FIFOCON_TXAT_ONE      0   // One-shot
#define MCP2518FD_FIFOCON_TXAT_THREE    1   // Three attempts
#define MCP2518FD_FIFOCON_TXAT_UNLIM    3   // Unlimited retransmit
#define MCP2518FD_FIFOCON_TXPRI_MASK    (0x1FUL << 16)
#define MCP2518FD_FIFOCON_FRESET        (1UL << 10)  // FIFO Reset (auto-clear)
#define MCP2518FD_FIFOCON_TXREQ         (1UL << 9)   // TX Request (auto-clear)
#define MCP2518FD_FIFOCON_UINC          (1UL << 8)   // User Increment (auto-clear)
#define MCP2518FD_FIFOCON_TXEN          (1UL << 7)   // TX Enable
#define MCP2518FD_FIFOCON_RTREN         (1UL << 6)   // Auto RTR enable
#define MCP2518FD_FIFOCON_RXTSEN        (1UL << 5)   // RX Timestamp enable
#define MCP2518FD_FIFOCON_TXATIE        (1UL << 4)   // TX Attempt IE
#define MCP2518FD_FIFOCON_RXOVIE        (1UL << 3)   // RX Overflow IE
#define MCP2518FD_FIFOCON_TFERFFIE      (1UL << 2)   // Full/Empty IE
#define MCP2518FD_FIFOCON_TFHRFHIE      (1UL << 1)   // Half-full IE
#define MCP2518FD_FIFOCON_TFNRFNIE      (1UL << 0)   // Not-full/Not-empty IE

// CiFIFOSTA bit definitions
#define MCP2518FD_FIFOSTA_FIFOCI_MASK   (0x1FUL << 8)
#define MCP2518FD_FIFOSTA_FIFOCI_SHIFT  8
#define MCP2518FD_FIFOSTA_TXABT         (1UL << 7)   // TX Aborted
#define MCP2518FD_FIFOSTA_TXLARB        (1UL << 6)   // TX Lost Arbitration
#define MCP2518FD_FIFOSTA_TXERR         (1UL << 5)   // TX Error
#define MCP2518FD_FIFOSTA_TXATIF        (1UL << 4)   // TX Attempt IF
#define MCP2518FD_FIFOSTA_RXOVIF        (1UL << 3)   // RX Overflow IF
#define MCP2518FD_FIFOSTA_TFERFFIF      (1UL << 2)   // Full (TX) / Full (RX)
#define MCP2518FD_FIFOSTA_TFHRFHIF      (1UL << 1)   // Half-full
#define MCP2518FD_FIFOSTA_TFNRFNIF      (1UL << 0)   // Not-full(TX) / Not-empty(RX)

// ============================================================================
// Filter Registers
// ============================================================================

// CiFLTCON: Filter Control (4 filters per register)
#define MCP2518FD_REG_FLTCON(x)         (0x1D0 + 0x04 * (x))
#define MCP2518FD_FLTCON_FLTEN(n)       (1UL << (7 + 8 * ((n) & 0x3)))
#define MCP2518FD_FLTCON_FBP(n, fifo)   ((uint32_t)(fifo) << (8 * ((n) & 0x3)))

// CiFLTOBJ: Filter Object
#define MCP2518FD_REG_FLTOBJ(x)         (0x1F0 + 0x08 * (x))
#define MCP2518FD_FLTOBJ_EXIDE          (1UL << 30)
#define MCP2518FD_FLTOBJ_SID11          (1UL << 29)
#define MCP2518FD_FLTOBJ_EID_MASK       (0x3FFFFUL << 11)
#define MCP2518FD_FLTOBJ_SID_MASK       (0x7FFUL << 0)

// CiFLTMASK: Filter Mask
#define MCP2518FD_REG_FLTMASK(x)        (0x1F4 + 0x08 * (x))
#define MCP2518FD_MASK_MIDE             (1UL << 30)  // Match IDE bit
#define MCP2518FD_MASK_MSID11           (1UL << 29)
#define MCP2518FD_MASK_MEID_MASK        (0x3FFFFUL << 11)
#define MCP2518FD_MASK_MSID_MASK        (0x7FFUL << 0)

// ============================================================================
// Message RAM
// ============================================================================

#define MCP2518FD_RAM_START             0x400
#define MCP2518FD_RAM_SIZE              2048    // 2 KB

// ============================================================================
// Message Object Format (in RAM)
// ============================================================================

// Word 0 (ID word) - same for TX and RX objects
#define MCP2518FD_OBJ_ID_SID11         (1UL << 29)
#define MCP2518FD_OBJ_ID_EID_MASK      (0x3FFFFUL << 11)   // GENMASK(28, 11)
#define MCP2518FD_OBJ_ID_EID_SHIFT     11
#define MCP2518FD_OBJ_ID_SID_MASK      (0x7FFUL << 0)      // GENMASK(10, 0)
#define MCP2518FD_OBJ_ID_SID_SHIFT     0

// Word 1 (Flags/Control word)
#define MCP2518FD_OBJ_FLAGS_SEQ_MASK   (0x7FFFFFUL << 9)   // GENMASK(31, 9)
#define MCP2518FD_OBJ_FLAGS_ESI        (1UL << 8)
#define MCP2518FD_OBJ_FLAGS_FDF        (1UL << 7)   // FD Format (0 = classic CAN)
#define MCP2518FD_OBJ_FLAGS_BRS        (1UL << 6)   // Bit Rate Switch (FD)
#define MCP2518FD_OBJ_FLAGS_RTR        (1UL << 5)   // Remote Transmit Request
#define MCP2518FD_OBJ_FLAGS_IDE        (1UL << 4)   // Extended ID
#define MCP2518FD_OBJ_FLAGS_DLC_MASK   (0x0FUL << 0)
#define MCP2518FD_OBJ_FLAGS_DLC_SHIFT  0

// TX Message Object: {id(4), flags(4), data(up to 64)}
#define MCP2518FD_TX_OBJ_HEADER_SIZE   8    // 2 words
#define MCP2518FD_TX_OBJ_SIZE_CAN      16   // 8 + 8 bytes (classic CAN)

// RX Message Object: {id(4), flags(4), timestamp(4), data(up to 64)}
#define MCP2518FD_RX_OBJ_HEADER_SIZE   12   // 3 words (includes timestamp)
#define MCP2518FD_RX_OBJ_SIZE_CAN      20   // 12 + 8 bytes (classic CAN)

// CAN frame EFF (29-bit) ID decomposition:
//   SID = CAN_ID[28:18], EID = CAN_ID[17:0]
#define MCP2518FD_FRAME_EFF_SID_SHIFT  18
#define MCP2518FD_FRAME_EFF_SID_MASK   (0x7FFUL << 18)  // GENMASK(28, 18)
#define MCP2518FD_FRAME_EFF_EID_MASK   (0x3FFFFUL << 0) // GENMASK(17, 0)

// ============================================================================
// MFR / Device Registers (outside CAN controller module)
// ============================================================================

// --- OSC: Oscillator Control (0xE00) ---
#define MCP2518FD_REG_OSC              0xE00
#define MCP2518FD_OSC_SCLKRDY          (1UL << 12)  // System Clock Ready
#define MCP2518FD_OSC_OSCRDY           (1UL << 10)  // Oscillator Ready
#define MCP2518FD_OSC_PLLRDY           (1UL << 8)   // PLL Ready
#define MCP2518FD_OSC_SCLKDIV          (1UL << 4)   // SYSCLK ÷ 2
#define MCP2518FD_OSC_LPMEN            (1UL << 3)   // Low-Power Mode (MCP2518FD)
#define MCP2518FD_OSC_OSCDIS           (1UL << 2)   // Oscillator Disable
#define MCP2518FD_OSC_PLLEN            (1UL << 0)   // PLL Enable

// --- IOCON: I/O Control (0xE04) ---
#define MCP2518FD_REG_IOCON            0xE04
#define MCP2518FD_IOCON_INTOD          (1UL << 30)  // INT pin open-drain
#define MCP2518FD_IOCON_SOF            (1UL << 29)  // SOF on CLKO
#define MCP2518FD_IOCON_TXCANOD        (1UL << 28)  // TXCAN open-drain
#define MCP2518FD_IOCON_PM1            (1UL << 25)  // GPIO1 pin mode
#define MCP2518FD_IOCON_PM0            (1UL << 24)  // GPIO0 pin mode
#define MCP2518FD_IOCON_GPIO1          (1UL << 17)  // GPIO1 value
#define MCP2518FD_IOCON_GPIO0          (1UL << 16)  // GPIO0 value
#define MCP2518FD_IOCON_LAT1           (1UL << 9)   // GPIO1 latch
#define MCP2518FD_IOCON_LAT0           (1UL << 8)   // GPIO0 latch
#define MCP2518FD_IOCON_XSTBYEN        (1UL << 6)   // Transceiver standby
#define MCP2518FD_IOCON_TRIS1          (1UL << 1)   // GPIO1 direction
#define MCP2518FD_IOCON_TRIS0          (1UL << 0)   // GPIO0 direction

// --- CRC: CRC Register (0xE08) ---
#define MCP2518FD_REG_CRC              0xE08
#define MCP2518FD_CRC_FERRIE           (1UL << 25)
#define MCP2518FD_CRC_CRCERRIE         (1UL << 24)
#define MCP2518FD_CRC_FERRIF           (1UL << 17)
#define MCP2518FD_CRC_CRCERRIF         (1UL << 16)
#define MCP2518FD_CRC_CRC_MASK         (0xFFFFUL << 0)

// --- ECCCON: ECC Control (0xE0C) ---
#define MCP2518FD_REG_ECCCON           0xE0C
#define MCP2518FD_ECCCON_DEDIE         (1UL << 2)
#define MCP2518FD_ECCCON_SECIE         (1UL << 1)
#define MCP2518FD_ECCCON_ECCEN         (1UL << 0)

// --- ECCSTAT: ECC Status (0xE10) ---
#define MCP2518FD_REG_ECCSTAT          0xE10

// --- DEVID: Device ID (0xE14) - MCP2518FD/MCP251863 only ---
#define MCP2518FD_REG_DEVID            0xE14
#define MCP2518FD_DEVID_ID_MASK        (0x0FUL << 4)
#define MCP2518FD_DEVID_REV_MASK       (0x0FUL << 0)

// ============================================================================
// FIFO Configuration (our application constants)
// ============================================================================

// FIFO1 = TX, FIFO2 = RX (FIFO0 is TXQ, not used)
#define MCP2518FD_TX_FIFO              1
#define MCP2518FD_RX_FIFO              2

#define MCP2518FD_TX_FIFO_DEPTH        8    // 8 TX message slots
#define MCP2518FD_RX_FIFO_DEPTH        16   // 16 RX message slots

// ============================================================================
// Nominal Bit Timing: Pre-computed CiNBTCFG values
//
// Register format:
//   [31:24] BRP   (actual prescaler = BRP + 1)
//   [23:16] TSEG1 (actual = TSEG1 + 1, includes prop_seg + phase_seg1)
//   [14:8]  TSEG2 (actual = TSEG2 + 1)
//   [6:0]   SJW   (actual = SJW + 1)
//
// Bit time (TQ) = Sync(1) + TSEG1(+1) + TSEG2(+1)
// Bitrate = FSYS / ((BRP+1) * bit_time_tq)
//
// All configs target ~80% sample point.
// ============================================================================

// --- 40 MHz crystal (Waveshare 2-CH CAN FD HAT) ---
// 500 kbps: BRP=0(÷1), TSEG1=62(63TQ), TSEG2=15(16TQ), SJW=15
//   Bit = 1+63+16 = 80 TQ, SP = 64/80 = 80%
#define MCP2518FD_40MHZ_500KBPS        0x003E0F0F

// 250 kbps: BRP=1(÷2), TSEG1=62(63TQ), TSEG2=15(16TQ), SJW=15
//   Bit = 80 TQ @ 50ns each = 4µs, SP = 80%
#define MCP2518FD_40MHZ_250KBPS        0x013E0F0F

// 125 kbps: BRP=3(÷4), TSEG1=62(63TQ), TSEG2=15(16TQ), SJW=15
//   Bit = 80 TQ @ 100ns each = 8µs, SP = 80%
#define MCP2518FD_40MHZ_125KBPS        0x033E0F0F

// --- 20 MHz crystal ---
// 500 kbps: BRP=0(÷1), TSEG1=29(30TQ), TSEG2=8(9TQ), SJW=8
//   Bit = 1+30+9 = 40 TQ, SP = 31/40 = 77.5%
#define MCP2518FD_20MHZ_500KBPS        0x001D0808

// 250 kbps: BRP=0(÷1), TSEG1=62(63TQ), TSEG2=15(16TQ), SJW=15
//   Bit = 80 TQ, SP = 80%
#define MCP2518FD_20MHZ_250KBPS        0x003E0F0F

// 125 kbps: BRP=1(÷2), TSEG1=62(63TQ), TSEG2=15(16TQ), SJW=15
//   Bit = 80 TQ, SP = 80%
#define MCP2518FD_20MHZ_125KBPS        0x013E0F0F

// ============================================================================
// System Clock
// ============================================================================

#define MCP2518FD_SYSCLOCK_HZ_MAX      40000000  // 40 MHz max
