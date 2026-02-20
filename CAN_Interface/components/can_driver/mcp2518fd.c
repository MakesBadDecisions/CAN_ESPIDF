/**
 * @file mcp2518fd.c
 * @brief MCP2518FD CAN FD Controller Driver Implementation
 *
 * Drop-in replacement for the MCP2515 backend. Uses the same
 * ISR → task notification → queue pattern.
 *
 * Key architectural differences from MCP2515:
 *   - SPI commands are 16-bit (instruction | 12-bit address)
 *   - Registers are 32-bit, little-endian on the SPI bus
 *   - FIFO-based TX/RX (configurable in 2KB device RAM)
 *   - Error status in CiTREC register (not separate EFLG)
 *
 * FIFO layout for Classic CAN @ OBD-II:
 *   FIFO1: TX — 8 slots × 16 bytes = 128 bytes RAM
 *   FIFO2: RX — 16 slots × 20 bytes = 320 bytes RAM
 *   Total: 448 / 2048 bytes used
 */

#include <string.h>
#include "mcp2518fd.h"
#include "mcp2518fd_defs.h"
#include "system.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "mcp2518fd";

// ============================================================================
// Driver State
// ============================================================================

typedef struct {
    spi_device_handle_t spi_handle;
    QueueHandle_t       rx_queue;
    SemaphoreHandle_t   spi_mutex;
    TaskHandle_t        isr_task_handle;
    can_stats_t         stats;
    can_state_t         state;
    mcp2518fd_config_t  config;
    uint32_t            tx_fifo_con;    // Cached TX FIFO config
    uint32_t            rx_fifo_con;    // Cached RX FIFO config
    volatile bool       isr_task_running;
    bool                initialized;
} mcp2518fd_driver_t;

static mcp2518fd_driver_t s_drv = {
    .state = CAN_STATE_UNINIT,
    .initialized = false,
};

// ============================================================================
// SPI Primitives
// ============================================================================

/**
 * @brief Raw SPI transfer (full-duplex)
 */
static esp_err_t spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t txn = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    xSemaphoreTake(s_drv.spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(s_drv.spi_handle, &txn);
    xSemaphoreGive(s_drv.spi_mutex);

    return ret;
}

/**
 * @brief Send SPI RESET command (0x0000)
 */
static void mcp2518fd_reset(void)
{
    uint8_t tx[2] = { 0x00, 0x00 };
    uint8_t rx[2];
    spi_transfer(tx, rx, 2);
}

/**
 * @brief Read a 32-bit register
 */
uint32_t mcp2518fd_read_register(uint16_t addr)
{
    uint8_t tx[6] = { 0 };
    uint8_t rx[6] = { 0 };

    // SPI command: READ | addr (big-endian)
    uint16_t cmd = MCP2518FD_SPI_READ | (addr & MCP2518FD_SPI_ADDR_MASK);
    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] = cmd & 0xFF;

    spi_transfer(tx, rx, 6);

    // Data is little-endian
    return (uint32_t)rx[2] |
           ((uint32_t)rx[3] << 8) |
           ((uint32_t)rx[4] << 16) |
           ((uint32_t)rx[5] << 24);
}

/**
 * @brief Write a 32-bit register
 */
void mcp2518fd_write_register(uint16_t addr, uint32_t value)
{
    uint8_t tx[6];
    uint8_t rx[6];

    uint16_t cmd = MCP2518FD_SPI_WRITE | (addr & MCP2518FD_SPI_ADDR_MASK);
    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] = cmd & 0xFF;
    tx[2] = (value >> 0) & 0xFF;
    tx[3] = (value >> 8) & 0xFF;
    tx[4] = (value >> 16) & 0xFF;
    tx[5] = (value >> 24) & 0xFF;

    spi_transfer(tx, rx, 6);
}

/**
 * @brief Read N bytes starting at addr (for message objects)
 */
static void mcp2518fd_read_bytes(uint16_t addr, uint8_t *data, size_t len)
{
    uint8_t tx[2 + 64] = { 0 };  // cmd + max 64 bytes
    uint8_t rx[2 + 64] = { 0 };

    uint16_t cmd = MCP2518FD_SPI_READ | (addr & MCP2518FD_SPI_ADDR_MASK);
    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] = cmd & 0xFF;

    size_t total = 2 + len;
    spi_transfer(tx, rx, total);
    memcpy(data, &rx[2], len);
}

/**
 * @brief Write N bytes starting at addr (for message objects)
 */
static void mcp2518fd_write_bytes(uint16_t addr, const uint8_t *data, size_t len)
{
    uint8_t tx[2 + 64] = { 0 };
    uint8_t rx[2 + 64];

    uint16_t cmd = MCP2518FD_SPI_WRITE | (addr & MCP2518FD_SPI_ADDR_MASK);
    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] = cmd & 0xFF;
    memcpy(&tx[2], data, len);

    spi_transfer(tx, rx, 2 + len);
}

// ============================================================================
// Mode Control
// ============================================================================

/**
 * @brief Get current operation mode from CiCON OPMOD field
 */
static uint8_t mcp2518fd_get_mode(void)
{
    uint32_t con = mcp2518fd_read_register(MCP2518FD_REG_CON);
    return (con & MCP2518FD_CON_OPMOD_MASK) >> MCP2518FD_CON_OPMOD_SHIFT;
}

/**
 * @brief Request operation mode change and wait for confirmation
 *
 * @param mode  Desired mode (MCP2518FD_MODE_xxx)
 * @return ESP_OK or ESP_ERR_TIMEOUT
 */
static esp_err_t mcp2518fd_set_mode(uint8_t mode)
{
    // Read current CiCON, update REQOP field
    uint32_t con = mcp2518fd_read_register(MCP2518FD_REG_CON);
    con &= ~MCP2518FD_CON_REQOP_MASK;
    con |= ((uint32_t)mode << MCP2518FD_CON_REQOP_SHIFT) & MCP2518FD_CON_REQOP_MASK;
    mcp2518fd_write_register(MCP2518FD_REG_CON, con);

    // Poll OPMOD until it matches or timeout (100ms)
    for (int i = 0; i < 100; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (mcp2518fd_get_mode() == mode) {
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "Mode change to %d timed out (stuck in %d)", mode, mcp2518fd_get_mode());
    return ESP_ERR_TIMEOUT;
}

// ============================================================================
// Bit Timing Configuration
// ============================================================================

/**
 * @brief Set nominal bit timing from pre-computed lookup table
 */
static esp_err_t mcp2518fd_configure_bitrate(uint32_t crystal, uint32_t bitrate)
{
    uint32_t nbtcfg = 0;

    if (crystal == 40000000) {
        switch (bitrate) {
            case 500000: nbtcfg = MCP2518FD_40MHZ_500KBPS; break;
            case 250000: nbtcfg = MCP2518FD_40MHZ_250KBPS; break;
            case 125000: nbtcfg = MCP2518FD_40MHZ_125KBPS; break;
            default:
                ESP_LOGE(TAG, "Unsupported bitrate %lu for 40MHz crystal", (unsigned long)bitrate);
                return ESP_ERR_NOT_SUPPORTED;
        }
    } else if (crystal == 20000000) {
        switch (bitrate) {
            case 500000: nbtcfg = MCP2518FD_20MHZ_500KBPS; break;
            case 250000: nbtcfg = MCP2518FD_20MHZ_250KBPS; break;
            case 125000: nbtcfg = MCP2518FD_20MHZ_125KBPS; break;
            default:
                ESP_LOGE(TAG, "Unsupported bitrate %lu for 20MHz crystal", (unsigned long)bitrate);
                return ESP_ERR_NOT_SUPPORTED;
        }
    } else {
        ESP_LOGE(TAG, "Unsupported crystal frequency: %lu Hz", (unsigned long)crystal);
        return ESP_ERR_NOT_SUPPORTED;
    }

    mcp2518fd_write_register(MCP2518FD_REG_NBTCFG, nbtcfg);

    SYS_LOGI(TAG, "Bitrate: %lu bps, crystal: %lu MHz, NBTCFG: 0x%08lX",
             (unsigned long)bitrate, (unsigned long)(crystal / 1000000),
             (unsigned long)nbtcfg);

    return ESP_OK;
}

// ============================================================================
// FIFO & Filter Configuration
// ============================================================================

/**
 * @brief Configure TX and RX FIFOs
 */
static void mcp2518fd_configure_fifos(void)
{
    // ----- FIFO1: TX -----
    // 8-byte payload, 8 deep, unlimited retransmit, TX enabled
    s_drv.tx_fifo_con =
        ((uint32_t)MCP2518FD_FIFOCON_PLSIZE_8 << MCP2518FD_FIFOCON_PLSIZE_SHIFT) |
        ((uint32_t)(MCP2518FD_TX_FIFO_DEPTH - 1) << MCP2518FD_FIFOCON_FSIZE_SHIFT) |
        ((uint32_t)MCP2518FD_FIFOCON_TXAT_UNLIM << 21) |
        MCP2518FD_FIFOCON_TXEN |
        MCP2518FD_FIFOCON_TXATIE;   // TX attempt interrupts

    mcp2518fd_write_register(MCP2518FD_REG_FIFOCON(MCP2518FD_TX_FIFO),
                             s_drv.tx_fifo_con);

    // ----- FIFO2: RX -----
    // 8-byte payload, 16 deep, timestamp, overflow + not-empty interrupts
    s_drv.rx_fifo_con =
        ((uint32_t)MCP2518FD_FIFOCON_PLSIZE_8 << MCP2518FD_FIFOCON_PLSIZE_SHIFT) |
        ((uint32_t)(MCP2518FD_RX_FIFO_DEPTH - 1) << MCP2518FD_FIFOCON_FSIZE_SHIFT) |
        MCP2518FD_FIFOCON_RXTSEN |  // Timestamp
        MCP2518FD_FIFOCON_RXOVIE |  // Overflow interrupt
        MCP2518FD_FIFOCON_TFNRFNIE; // Not-empty interrupt

    mcp2518fd_write_register(MCP2518FD_REG_FIFOCON(MCP2518FD_RX_FIFO),
                             s_drv.rx_fifo_con);

    SYS_LOGD(TAG, "FIFOs configured: TX=%d deep, RX=%d deep",
             MCP2518FD_TX_FIFO_DEPTH, MCP2518FD_RX_FIFO_DEPTH);
}

/**
 * @brief Configure filter 0 to accept all messages → RX FIFO
 */
static void mcp2518fd_configure_filter(void)
{
    // Filter 0 → point to RX FIFO (FIFO2), enable
    uint32_t fltcon = MCP2518FD_FLTCON_FLTEN(0) |
                      MCP2518FD_FLTCON_FBP(0, MCP2518FD_RX_FIFO);
    mcp2518fd_write_register(MCP2518FD_REG_FLTCON(0), fltcon);

    // Filter object: match any (value = 0)
    mcp2518fd_write_register(MCP2518FD_REG_FLTOBJ(0), 0x00000000);

    // Mask: all don't-care (0 = accept all)
    mcp2518fd_write_register(MCP2518FD_REG_FLTMASK(0), 0x00000000);

    SYS_LOGD(TAG, "Filter 0: accept all → FIFO%d", MCP2518FD_RX_FIFO);
}

// ============================================================================
// Frame TX / RX Helpers
// ============================================================================

/**
 * @brief Build a TX message object from a can_frame_t
 *
 * TX object layout: [id(4)] [flags(4)] [data(8)] = 16 bytes
 */
static void build_tx_object(const can_frame_t *frame, uint8_t *obj)
{
    uint32_t id_word = 0;
    uint32_t flags_word = 0;

    if (frame->extended) {
        // 29-bit extended ID → split into SID (upper 11) + EID (lower 18)
        uint32_t sid = (frame->id >> MCP2518FD_FRAME_EFF_SID_SHIFT) & 0x7FF;
        uint32_t eid = frame->id & 0x3FFFF;
        id_word = (eid << MCP2518FD_OBJ_ID_EID_SHIFT) |
                  (sid << MCP2518FD_OBJ_ID_SID_SHIFT);
        flags_word |= MCP2518FD_OBJ_FLAGS_IDE;
    } else {
        // 11-bit standard ID → SID only
        id_word = (frame->id & 0x7FF) << MCP2518FD_OBJ_ID_SID_SHIFT;
    }

    if (frame->rtr) {
        flags_word |= MCP2518FD_OBJ_FLAGS_RTR;
    }

    // DLC (0-8 for classic CAN)
    uint8_t dlc = frame->dlc > 8 ? 8 : frame->dlc;
    flags_word |= (uint32_t)dlc << MCP2518FD_OBJ_FLAGS_DLC_SHIFT;

    // Pack as little-endian 32-bit words
    obj[0] = (id_word >> 0) & 0xFF;
    obj[1] = (id_word >> 8) & 0xFF;
    obj[2] = (id_word >> 16) & 0xFF;
    obj[3] = (id_word >> 24) & 0xFF;

    obj[4] = (flags_word >> 0) & 0xFF;
    obj[5] = (flags_word >> 8) & 0xFF;
    obj[6] = (flags_word >> 16) & 0xFF;
    obj[7] = (flags_word >> 24) & 0xFF;

    // Copy data (after 8-byte header)
    if (!frame->rtr) {
        memcpy(&obj[8], frame->data, dlc);
        // Zero-pad remaining bytes
        if (dlc < 8) {
            memset(&obj[8 + dlc], 0, 8 - dlc);
        }
    } else {
        memset(&obj[8], 0, 8);
    }
}

/**
 * @brief Parse an RX message object into a can_frame_t
 *
 * RX object layout: [id(4)] [flags(4)] [timestamp(4)] [data(8)] = 20 bytes
 */
static void parse_rx_object(const uint8_t *obj, can_frame_t *frame)
{
    // Extract little-endian 32-bit words
    uint32_t id_word = (uint32_t)obj[0] |
                       ((uint32_t)obj[1] << 8) |
                       ((uint32_t)obj[2] << 16) |
                       ((uint32_t)obj[3] << 24);

    uint32_t flags_word = (uint32_t)obj[4] |
                          ((uint32_t)obj[5] << 8) |
                          ((uint32_t)obj[6] << 16) |
                          ((uint32_t)obj[7] << 24);

    // obj[8..11] = timestamp (we don't use it currently)

    memset(frame, 0, sizeof(can_frame_t));

    if (flags_word & MCP2518FD_OBJ_FLAGS_IDE) {
        // Extended frame: reconstruct 29-bit ID from SID + EID
        uint32_t sid = (id_word & MCP2518FD_OBJ_ID_SID_MASK) >> MCP2518FD_OBJ_ID_SID_SHIFT;
        uint32_t eid = (id_word & MCP2518FD_OBJ_ID_EID_MASK) >> MCP2518FD_OBJ_ID_EID_SHIFT;
        frame->id = (sid << MCP2518FD_FRAME_EFF_SID_SHIFT) | eid;
        frame->extended = true;
    } else {
        // Standard frame: 11-bit SID
        frame->id = (id_word & MCP2518FD_OBJ_ID_SID_MASK) >> MCP2518FD_OBJ_ID_SID_SHIFT;
        frame->extended = false;
    }

    frame->rtr = (flags_word & MCP2518FD_OBJ_FLAGS_RTR) ? true : false;
    frame->dlc = (flags_word & MCP2518FD_OBJ_FLAGS_DLC_MASK) >> MCP2518FD_OBJ_FLAGS_DLC_SHIFT;
    if (frame->dlc > 8) frame->dlc = 8;

    // Copy data (starts at byte 12 in RX object, after timestamp)
    if (!frame->rtr) {
        memcpy(frame->data, &obj[12], frame->dlc);
    }
}

// ============================================================================
// Error State Management
// ============================================================================

/**
 * @brief Update CAN state from CiTREC register
 */
static void update_error_state(void)
{
    uint32_t trec = mcp2518fd_read_register(MCP2518FD_REG_TREC);

    if (trec & MCP2518FD_TREC_TXBO) {
        if (s_drv.state != CAN_STATE_BUS_OFF) {
            SYS_LOGW(TAG, "Bus-Off detected (TEC=%lu, REC=%lu)",
                     (unsigned long)((trec & MCP2518FD_TREC_TEC_MASK) >> MCP2518FD_TREC_TEC_SHIFT),
                     (unsigned long)(trec & MCP2518FD_TREC_REC_MASK));
            s_drv.state = CAN_STATE_BUS_OFF;
        }
    } else if ((trec & MCP2518FD_TREC_TXBP) || (trec & MCP2518FD_TREC_RXBP)) {
        s_drv.state = CAN_STATE_ERROR_PASSIVE;
    } else if (trec & MCP2518FD_TREC_EWARN) {
        s_drv.state = CAN_STATE_ERROR_WARNING;
    } else if (s_drv.state != CAN_STATE_STOPPED) {
        s_drv.state = CAN_STATE_RUNNING;
    }
}

// ============================================================================
// ISR Task (GPIO interrupt → task notification → process)
// ============================================================================

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_drv.isr_task_handle, &higher_prio_woken);
    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief ISR task — processes MCP2518FD interrupts
 *
 * Woken by GPIO falling edge on INT pin.
 * Reads CiINT, processes RX frames, handles errors.
 */
static void mcp2518fd_isr_task(void *arg)
{
    s_drv.isr_task_running = true;

    while (s_drv.isr_task_running) {
        // Wait for GPIO interrupt notification
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));  // Periodic wakeup for safety

        if (!s_drv.initialized || !s_drv.isr_task_running) {
            continue;
        }

        // Read main interrupt register
        uint32_t intf = mcp2518fd_read_register(MCP2518FD_REG_INT);
        uint16_t flags = intf & 0xFFFF;

        if (flags == 0) {
            continue;  // Spurious wake
        }

        // ----- RX Handling -----
        if (flags & MCP2518FD_INT_RXIF) {
            // Check which FIFOs have data
            uint32_t rxif = mcp2518fd_read_register(MCP2518FD_REG_RXIF);

            if (rxif & (1UL << MCP2518FD_RX_FIFO)) {
                // Drain RX FIFO
                for (int burst = 0; burst < MCP2518FD_RX_FIFO_DEPTH; burst++) {
                    uint32_t sta = mcp2518fd_read_register(
                        MCP2518FD_REG_FIFOSTA(MCP2518FD_RX_FIFO));

                    if (!(sta & MCP2518FD_FIFOSTA_TFNRFNIF)) {
                        break;  // FIFO empty
                    }

                    // Get RAM address of next RX message
                    uint32_t ua = mcp2518fd_read_register(
                        MCP2518FD_REG_FIFOUA(MCP2518FD_RX_FIFO));

                    // Read RX object (20 bytes for classic CAN with timestamp)
                    uint8_t rx_obj[MCP2518FD_RX_OBJ_SIZE_CAN];
                    mcp2518fd_read_bytes((uint16_t)(ua + MCP2518FD_RAM_START),
                                        rx_obj, sizeof(rx_obj));

                    // Advance FIFO pointer (UINC)
                    mcp2518fd_write_register(
                        MCP2518FD_REG_FIFOCON(MCP2518FD_RX_FIFO),
                        s_drv.rx_fifo_con | MCP2518FD_FIFOCON_UINC);

                    // Parse into can_frame_t
                    can_frame_t frame;
                    parse_rx_object(rx_obj, &frame);

                    // Enqueue
                    if (xQueueSend(s_drv.rx_queue, &frame, 0) == pdTRUE) {
                        s_drv.stats.rx_frames++;
                    } else {
                        s_drv.stats.rx_overflows++;
                    }
                }
            }
        }

        // ----- RX Overflow -----
        if (flags & MCP2518FD_INT_RXOVIF) {
            uint32_t rxovif = mcp2518fd_read_register(MCP2518FD_REG_RXOVIF);
            if (rxovif & (1UL << MCP2518FD_RX_FIFO)) {
                s_drv.stats.rx_overflows++;
                SYS_LOGW(TAG, "RX FIFO%d overflow", MCP2518FD_RX_FIFO);

                // Clear overflow flag in FIFO status
                uint32_t sta = mcp2518fd_read_register(
                    MCP2518FD_REG_FIFOSTA(MCP2518FD_RX_FIFO));
                sta &= ~MCP2518FD_FIFOSTA_RXOVIF;
                mcp2518fd_write_register(
                    MCP2518FD_REG_FIFOSTA(MCP2518FD_RX_FIFO), sta);
            }
        }

        // ----- TX Attempt Failure -----
        if (flags & MCP2518FD_INT_TXATIF) {
            s_drv.stats.tx_errors++;
            SYS_LOGD(TAG, "TX attempt failure");

            // Clear TX FIFO attempt flag
            uint32_t sta = mcp2518fd_read_register(
                MCP2518FD_REG_FIFOSTA(MCP2518FD_TX_FIFO));
            if (sta & MCP2518FD_FIFOSTA_TXATIF) {
                sta &= ~MCP2518FD_FIFOSTA_TXATIF;
                mcp2518fd_write_register(
                    MCP2518FD_REG_FIFOSTA(MCP2518FD_TX_FIFO), sta);
            }
        }

        // ----- CAN Error / System Error -----
        if (flags & (MCP2518FD_INT_CERRIF | MCP2518FD_INT_SERRIF)) {
            update_error_state();

            if (flags & MCP2518FD_INT_SERRIF) {
                s_drv.stats.rx_errors++;
                SYS_LOGW(TAG, "System error interrupt");
            }
        }

        // ----- Clear clearable interrupt flags -----
        uint32_t to_clear = flags & MCP2518FD_INT_IF_CLEARABLE;
        if (to_clear) {
            // Write IE bits unchanged, clear the handled IF bits
            mcp2518fd_write_register(MCP2518FD_REG_INT, intf & ~to_clear);
        }
    }

    vTaskDelete(NULL);
}

// ============================================================================
// Public API
// ============================================================================

bool mcp2518fd_probe(void)
{
    if (!s_drv.initialized) {
        return false;
    }

    // Reset device
    mcp2518fd_reset();
    vTaskDelay(pdMS_TO_TICKS(5));  // Wait for oscillator

    // Check oscillator is ready
    uint32_t osc = mcp2518fd_read_register(MCP2518FD_REG_OSC);
    if (!(osc & MCP2518FD_OSC_OSCRDY)) {
        // Retry once after additional delay
        vTaskDelay(pdMS_TO_TICKS(10));
        osc = mcp2518fd_read_register(MCP2518FD_REG_OSC);
        if (!(osc & MCP2518FD_OSC_OSCRDY)) {
            ESP_LOGE(TAG, "Oscillator not ready (OSC=0x%08lX)", (unsigned long)osc);
            return false;
        }
    }

    // Verify device is in Configuration mode after reset
    uint8_t mode = mcp2518fd_get_mode();
    if (mode != MCP2518FD_MODE_CONFIG) {
        ESP_LOGE(TAG, "Not in Config mode after reset (mode=%d)", mode);
        return false;
    }

    // Try to read DEVID register (MCP2518FD-specific)
    uint32_t devid = mcp2518fd_read_register(MCP2518FD_REG_DEVID);
    SYS_LOGI(TAG, "Probe OK: DEVID=0x%08lX, OSC=0x%08lX",
             (unsigned long)devid, (unsigned long)osc);

    return true;
}

esp_err_t mcp2518fd_init(const mcp2518fd_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_drv.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;

    // Store configuration
    memcpy(&s_drv.config, config, sizeof(mcp2518fd_config_t));

    // Create mutex
    s_drv.spi_mutex = xSemaphoreCreateMutex();
    if (!s_drv.spi_mutex) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = config->pin_miso,
        .sclk_io_num = config->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,  // Enough for message objects + burst reads
    };

    ret = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        goto err_spi_bus;
    }

    // Add SPI device (Mode 0,0 — same as MCP2515)
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,  // CPOL=0, CPHA=0
        .clock_speed_hz = config->spi_clock_hz,
        .spics_io_num = config->pin_cs,
        .queue_size = 4,
    };

    ret = spi_bus_add_device(config->spi_host, &dev_cfg, &s_drv.spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        goto err_spi_device;
    }

    s_drv.initialized = true;

    // Reset and probe
    if (!mcp2518fd_probe()) {
        ret = ESP_ERR_NOT_FOUND;
        goto err_probe;
    }

    // Configure oscillator: no PLL, no clock division
    mcp2518fd_write_register(MCP2518FD_REG_OSC, 0x00000000);
    vTaskDelay(pdMS_TO_TICKS(3));

    // Verify oscillator settled
    uint32_t osc = mcp2518fd_read_register(MCP2518FD_REG_OSC);
    if (!(osc & MCP2518FD_OSC_OSCRDY)) {
        ESP_LOGE(TAG, "Oscillator not stable after config (OSC=0x%08lX)", (unsigned long)osc);
        ret = ESP_ERR_TIMEOUT;
        goto err_probe;
    }

    // Configure ECC (enable with no interrupts — just for data integrity)
    mcp2518fd_write_register(MCP2518FD_REG_ECCCON, MCP2518FD_ECCCON_ECCEN);

    // Configure bit timing (still in config mode after reset)
    ret = mcp2518fd_configure_bitrate(config->crystal_freq, config->can_bitrate);
    if (ret != ESP_OK) {
        goto err_bitrate;
    }

    // Configure CiCON for Classic CAN 2.0 operation
    //   - No TXQ, no TEF (we use FIFO1 for TX)
    //   - ISO CRC enabled (good practice)
    //   - REQOP stays Config until start() is called
    uint32_t con = ((uint32_t)MCP2518FD_MODE_CONFIG << MCP2518FD_CON_REQOP_SHIFT) |
                   MCP2518FD_CON_ISOCRCEN |
                   MCP2518FD_CON_PXEDIS;    // Disable protocol exceptions
    mcp2518fd_write_register(MCP2518FD_REG_CON, con);

    // Configure FIFOs
    mcp2518fd_configure_fifos();

    // Configure filter: accept all → RX FIFO
    mcp2518fd_configure_filter();

    // Enable timestamp counter (for RX timestamps)
    mcp2518fd_write_register(MCP2518FD_REG_TSCON, MCP2518FD_TSCON_TBCEN);

    // Create FreeRTOS RX queue
    s_drv.rx_queue = xQueueCreate(config->rx_queue_len, sizeof(can_frame_t));
    if (!s_drv.rx_queue) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        ret = ESP_ERR_NO_MEM;
        goto err_queue;
    }

    // Configure interrupt pin (active-low, same pattern as MCP2515)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_int),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(config->pin_int, gpio_isr_handler, NULL);

    // Enable interrupts on MCP2518FD
    //   RX, TX attempt, CAN error, system error, RX overflow
    uint32_t int_enable = MCP2518FD_INT_RXIE |
                          MCP2518FD_INT_TXATIE |
                          MCP2518FD_INT_CERRIE |
                          MCP2518FD_INT_SERRIE |
                          MCP2518FD_INT_RXOVIE;

    // Read current (preserves IF bits), set IE bits
    uint32_t int_reg = mcp2518fd_read_register(MCP2518FD_REG_INT);
    int_reg &= 0x0000FFFF;  // Keep only IF bits
    int_reg |= int_enable;   // Set IE bits
    mcp2518fd_write_register(MCP2518FD_REG_INT, int_reg);

    // Configure IOCON: INT pin as active-low push-pull
    uint32_t iocon = mcp2518fd_read_register(MCP2518FD_REG_IOCON);
    iocon &= ~MCP2518FD_IOCON_INTOD;   // Push-pull (not open-drain)
    iocon |= MCP2518FD_IOCON_XSTBYEN;  // Transceiver standby via GPIO
    mcp2518fd_write_register(MCP2518FD_REG_IOCON, iocon);

    // Create ISR handling task
    BaseType_t task_ret = xTaskCreate(mcp2518fd_isr_task, "mcp2518fd_isr",
                                      4096, NULL, 10, &s_drv.isr_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ISR task");
        ret = ESP_ERR_NO_MEM;
        goto err_task;
    }

    s_drv.state = CAN_STATE_STOPPED;
    memset(&s_drv.stats, 0, sizeof(can_stats_t));

    SYS_LOGI(TAG, "Initialized: %lu bps, %lu MHz crystal, SPI @ %lu Hz",
             (unsigned long)config->can_bitrate,
             (unsigned long)(config->crystal_freq / 1000000),
             (unsigned long)config->spi_clock_hz);

    return ESP_OK;

err_task:
    gpio_isr_handler_remove(config->pin_int);
    vQueueDelete(s_drv.rx_queue);
err_queue:
err_bitrate:
err_probe:
    spi_bus_remove_device(s_drv.spi_handle);
    s_drv.initialized = false;
err_spi_device:
    spi_bus_free(config->spi_host);
err_spi_bus:
    vSemaphoreDelete(s_drv.spi_mutex);
    return ret;
}

esp_err_t mcp2518fd_deinit(void)
{
    if (!s_drv.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Stop ISR task
    s_drv.isr_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(200));

    // Remove GPIO ISR
    gpio_isr_handler_remove(s_drv.config.pin_int);

    // Put device in config mode
    mcp2518fd_set_mode(MCP2518FD_MODE_CONFIG);

    // Free resources
    vQueueDelete(s_drv.rx_queue);
    spi_bus_remove_device(s_drv.spi_handle);
    spi_bus_free(s_drv.config.spi_host);
    vSemaphoreDelete(s_drv.spi_mutex);

    s_drv.initialized = false;
    s_drv.state = CAN_STATE_UNINIT;

    SYS_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

esp_err_t mcp2518fd_start(void)
{
    if (!s_drv.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t mode;
    if (s_drv.config.loopback) {
        mode = MCP2518FD_MODE_INT_LOOPBACK;
        SYS_LOGI(TAG, "Starting in loopback mode");
    } else if (s_drv.config.listen_only) {
        mode = MCP2518FD_MODE_LISTENONLY;
        SYS_LOGI(TAG, "Starting in listen-only mode");
    } else {
        mode = MCP2518FD_MODE_CAN20;
        SYS_LOGI(TAG, "Starting in Normal CAN 2.0 mode");
    }

    esp_err_t ret = mcp2518fd_set_mode(mode);
    if (ret == ESP_OK) {
        s_drv.state = CAN_STATE_RUNNING;
    }

    return ret;
}

esp_err_t mcp2518fd_stop(void)
{
    if (!s_drv.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = mcp2518fd_set_mode(MCP2518FD_MODE_CONFIG);
    if (ret == ESP_OK) {
        s_drv.state = CAN_STATE_STOPPED;
    }

    return ret;
}

esp_err_t mcp2518fd_send(const can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_drv.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_drv.state == CAN_STATE_BUS_OFF) {
        SYS_LOGW(TAG, "TX blocked: bus-off");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_drv.state == CAN_STATE_STOPPED) {
        SYS_LOGW(TAG, "TX blocked: not started");
        return ESP_ERR_INVALID_STATE;
    }

    if (!frame || frame->dlc > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check TX FIFO has space
    uint32_t sta = mcp2518fd_read_register(
        MCP2518FD_REG_FIFOSTA(MCP2518FD_TX_FIFO));

    if (!(sta & MCP2518FD_FIFOSTA_TFNRFNIF)) {
        // TX FIFO full
        s_drv.stats.tx_errors++;
        return ESP_ERR_TIMEOUT;
    }

    // Get RAM address for next TX slot
    uint32_t ua = mcp2518fd_read_register(
        MCP2518FD_REG_FIFOUA(MCP2518FD_TX_FIFO));

    // Build TX message object (16 bytes for classic CAN)
    uint8_t tx_obj[MCP2518FD_TX_OBJ_SIZE_CAN];
    build_tx_object(frame, tx_obj);

    // Write message object to device RAM
    mcp2518fd_write_bytes((uint16_t)(ua + MCP2518FD_RAM_START),
                          tx_obj, sizeof(tx_obj));

    // Set UINC + TXREQ to queue the message
    mcp2518fd_write_register(
        MCP2518FD_REG_FIFOCON(MCP2518FD_TX_FIFO),
        s_drv.tx_fifo_con | MCP2518FD_FIFOCON_UINC | MCP2518FD_FIFOCON_TXREQ);

    s_drv.stats.tx_frames++;

    return ESP_OK;
}

esp_err_t mcp2518fd_receive(can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_drv.initialized || !frame) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    if (xQueueReceive(s_drv.rx_queue, frame, ticks) == pdTRUE) {
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

uint32_t mcp2518fd_get_rx_queue_count(void)
{
    if (!s_drv.rx_queue) {
        return 0;
    }
    return uxQueueMessagesWaiting(s_drv.rx_queue);
}

QueueHandle_t mcp2518fd_get_rx_queue(void)
{
    return s_drv.rx_queue;
}

esp_err_t mcp2518fd_get_stats(can_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update error counters from hardware
    if (s_drv.initialized) {
        uint32_t trec = mcp2518fd_read_register(MCP2518FD_REG_TREC);
        s_drv.stats.tec = (trec & MCP2518FD_TREC_TEC_MASK) >> MCP2518FD_TREC_TEC_SHIFT;
        s_drv.stats.rec = (trec & MCP2518FD_TREC_REC_MASK) >> MCP2518FD_TREC_REC_SHIFT;
    }

    memcpy(stats, &s_drv.stats, sizeof(can_stats_t));
    return ESP_OK;
}

can_state_t mcp2518fd_get_state(void)
{
    return s_drv.state;
}

esp_err_t mcp2518fd_abort_all_tx(void)
{
    if (!s_drv.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Set ABAT in CiCON to abort all pending TX
    uint32_t con = mcp2518fd_read_register(MCP2518FD_REG_CON);
    con |= MCP2518FD_CON_ABAT;
    mcp2518fd_write_register(MCP2518FD_REG_CON, con);

    // Wait for TX FIFO to drain (up to 10ms)
    for (int i = 0; i < 100; i++) {
        uint32_t txreq = mcp2518fd_read_register(MCP2518FD_REG_TXREQ);
        if (txreq == 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Clear ABAT
    con = mcp2518fd_read_register(MCP2518FD_REG_CON);
    con &= ~MCP2518FD_CON_ABAT;
    mcp2518fd_write_register(MCP2518FD_REG_CON, con);

    SYS_LOGD(TAG, "TX aborted");
    return ESP_OK;
}

esp_err_t mcp2518fd_clear_errors(void)
{
    if (!s_drv.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Bus-off recovery: abort TX, cycle through config mode
    if (s_drv.state == CAN_STATE_BUS_OFF) {
        SYS_LOGW(TAG, "Bus-off recovery: cycling config mode...");
        mcp2518fd_abort_all_tx();

        esp_err_t ret = mcp2518fd_set_mode(MCP2518FD_MODE_CONFIG);
        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            ret = mcp2518fd_set_mode(MCP2518FD_MODE_CAN20);
        }
        if (ret == ESP_OK) {
            s_drv.state = CAN_STATE_RUNNING;
            SYS_LOGI(TAG, "Bus-off recovery complete");
        } else {
            ESP_LOGE(TAG, "Bus-off recovery failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Clear diagnostic registers
    mcp2518fd_write_register(MCP2518FD_REG_BDIAG0, 0);
    mcp2518fd_write_register(MCP2518FD_REG_BDIAG1, 0);

    // Clear error interrupt flags
    uint32_t intf = mcp2518fd_read_register(MCP2518FD_REG_INT);
    intf &= ~(MCP2518FD_INT_CERRIF | MCP2518FD_INT_SERRIF);
    mcp2518fd_write_register(MCP2518FD_REG_INT, intf);

    // Reset statistics
    s_drv.stats.tx_errors = 0;
    s_drv.stats.rx_errors = 0;
    s_drv.stats.rx_overflows = 0;

    // If we were in a transient error state, return to running
    if (s_drv.state == CAN_STATE_ERROR_WARNING ||
        s_drv.state == CAN_STATE_ERROR_PASSIVE) {
        s_drv.state = CAN_STATE_RUNNING;
    }

    return ESP_OK;
}
