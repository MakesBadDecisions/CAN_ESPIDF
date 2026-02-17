/**
 * @file mcp2515.c
 * @brief MCP2515 SPI CAN Controller Driver Implementation
 */

#include "mcp2515.h"
#include "mcp2515_defs.h"
#include "can_driver.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "mcp2515";

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
    mcp2515_config_t    config;
    bool                initialized;
    volatile bool       isr_task_running;
} mcp2515_driver_t;

static mcp2515_driver_t s_driver = {0};

// ============================================================================
// SPI Communication Primitives
// ============================================================================

static esp_err_t spi_transfer(const uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
{
    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    return spi_device_polling_transmit(s_driver.spi_handle, &trans);
}

uint8_t mcp2515_read_register(uint8_t addr)
{
    uint8_t tx[3] = {MCP2515_READ, addr, 0x00};
    uint8_t rx[3] = {0};
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, rx, 3);
    xSemaphoreGive(s_driver.spi_mutex);
    
    return rx[2];
}

void mcp2515_write_register(uint8_t addr, uint8_t value)
{
    uint8_t tx[3] = {MCP2515_WRITE, addr, value};
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, NULL, 3);
    xSemaphoreGive(s_driver.spi_mutex);
}

// Note: mcp2515_read_registers and mcp2515_write_registers removed - not currently used
// Can be restored if needed for filter configuration or bulk register access

static void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t tx[4] = {MCP2515_BIT_MODIFY, addr, mask, value};
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, NULL, 4);
    xSemaphoreGive(s_driver.spi_mutex);
}

static void mcp2515_reset(void)
{
    uint8_t tx[1] = {MCP2515_RESET};
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, NULL, 1);
    xSemaphoreGive(s_driver.spi_mutex);
    
    // Wait for oscillator to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));
}

static uint8_t mcp2515_read_status(void)
{
    uint8_t tx[2] = {MCP2515_READ_STATUS, 0x00};
    uint8_t rx[2] = {0};
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, rx, 2);
    xSemaphoreGive(s_driver.spi_mutex);
    
    return rx[1];
}

// ============================================================================
// Mode Control
// ============================================================================

static esp_err_t mcp2515_set_mode(uint8_t mode)
{
    mcp2515_bit_modify(MCP2515_CANCTRL, CANCTRL_REQOP_MASK, mode);
    
    // Wait for mode change with timeout
    for (int i = 0; i < 100; i++) {
        uint8_t canstat = mcp2515_read_register(MCP2515_CANSTAT);
        if ((canstat & CANSTAT_OPMOD_MASK) == mode) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGE(TAG, "Mode change timeout (requested: 0x%02X)", mode);
    return ESP_ERR_TIMEOUT;
}

// ============================================================================
// Bit Timing Configuration
// ============================================================================

static esp_err_t mcp2515_configure_bitrate(can_crystal_t crystal, can_bitrate_t bitrate)
{
    uint8_t cnf1, cnf2, cnf3;
    
    if (crystal == CAN_CRYSTAL_8MHZ) {
        switch (bitrate) {
            case CAN_BITRATE_500K:
                cnf1 = MCP2515_8MHZ_500KBPS_CNF1;
                cnf2 = MCP2515_8MHZ_500KBPS_CNF2;
                cnf3 = MCP2515_8MHZ_500KBPS_CNF3;
                break;
            case CAN_BITRATE_250K:
                cnf1 = MCP2515_8MHZ_250KBPS_CNF1;
                cnf2 = MCP2515_8MHZ_250KBPS_CNF2;
                cnf3 = MCP2515_8MHZ_250KBPS_CNF3;
                break;
            case CAN_BITRATE_125K:
                cnf1 = MCP2515_8MHZ_125KBPS_CNF1;
                cnf2 = MCP2515_8MHZ_125KBPS_CNF2;
                cnf3 = MCP2515_8MHZ_125KBPS_CNF3;
                break;
            default:
                ESP_LOGE(TAG, "Unsupported bitrate for 8MHz crystal");
                return ESP_ERR_NOT_SUPPORTED;
        }
    } else if (crystal == CAN_CRYSTAL_16MHZ) {
        switch (bitrate) {
            case CAN_BITRATE_500K:
                cnf1 = MCP2515_16MHZ_500KBPS_CNF1;
                cnf2 = MCP2515_16MHZ_500KBPS_CNF2;
                cnf3 = MCP2515_16MHZ_500KBPS_CNF3;
                break;
            case CAN_BITRATE_250K:
                cnf1 = MCP2515_16MHZ_250KBPS_CNF1;
                cnf2 = MCP2515_16MHZ_250KBPS_CNF2;
                cnf3 = MCP2515_16MHZ_250KBPS_CNF3;
                break;
            case CAN_BITRATE_125K:
                cnf1 = MCP2515_16MHZ_125KBPS_CNF1;
                cnf2 = MCP2515_16MHZ_125KBPS_CNF2;
                cnf3 = MCP2515_16MHZ_125KBPS_CNF3;
                break;
            default:
                ESP_LOGE(TAG, "Unsupported bitrate for 16MHz crystal");
                return ESP_ERR_NOT_SUPPORTED;
        }
    } else {
        ESP_LOGE(TAG, "Unsupported crystal frequency");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    mcp2515_write_register(MCP2515_CNF1, cnf1);
    mcp2515_write_register(MCP2515_CNF2, cnf2);
    mcp2515_write_register(MCP2515_CNF3, cnf3);
    
    ESP_LOGI(TAG, "Configured bitrate: %d Hz (CNF1=0x%02X, CNF2=0x%02X, CNF3=0x%02X)",
             bitrate, cnf1, cnf2, cnf3);
    
    return ESP_OK;
}

// ============================================================================
// Frame Handling
// ============================================================================

static void mcp2515_read_rx_buffer(uint8_t buffer_num, can_frame_t *frame)
{
    uint8_t read_cmd = (buffer_num == 0) ? MCP2515_READ_RX0 : MCP2515_READ_RX1;
    
    // Use quick read command - faster than sequential reads
    uint8_t tx[14] = {read_cmd};
    uint8_t rx[14] = {0};
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, rx, 14);
    xSemaphoreGive(s_driver.spi_mutex);
    
    // Parse ID
    uint8_t sidh = rx[1];
    uint8_t sidl = rx[2];
    uint8_t eid8 = rx[3];
    uint8_t eid0 = rx[4];
    uint8_t dlc = rx[5];
    
    frame->extended = (sidl & 0x08) != 0;
    
    if (frame->extended) {
        // 29-bit extended ID
        frame->id = ((uint32_t)sidh << 21) |
                    ((uint32_t)(sidl & 0xE0) << 13) |
                    ((uint32_t)(sidl & 0x03) << 16) |
                    ((uint32_t)eid8 << 8) |
                    (uint32_t)eid0;
    } else {
        // 11-bit standard ID
        frame->id = ((uint32_t)sidh << 3) | ((uint32_t)sidl >> 5);
    }
    
    frame->rtr = (dlc & 0x40) != 0;
    frame->dlc = dlc & 0x0F;
    if (frame->dlc > 8) frame->dlc = 8;
    
    // Copy data bytes
    memcpy(frame->data, &rx[6], frame->dlc);
}

static esp_err_t mcp2515_write_tx_buffer(uint8_t buffer_num, const can_frame_t *frame)
{
    uint8_t buf[13];  // SIDH, SIDL, EID8, EID0, DLC, D0-D7
    
    if (frame->extended) {
        // 29-bit extended ID
        buf[0] = (frame->id >> 21) & 0xFF;                          // SIDH
        buf[1] = ((frame->id >> 13) & 0xE0) |                       // SIDL
                 0x08 |                                              // EXIDE bit
                 ((frame->id >> 16) & 0x03);
        buf[2] = (frame->id >> 8) & 0xFF;                           // EID8
        buf[3] = frame->id & 0xFF;                                  // EID0
    } else {
        // 11-bit standard ID
        buf[0] = (frame->id >> 3) & 0xFF;                           // SIDH
        buf[1] = (frame->id << 5) & 0xE0;                           // SIDL
        buf[2] = 0;                                                  // EID8
        buf[3] = 0;                                                  // EID0
    }
    
    buf[4] = (frame->rtr ? 0x40 : 0x00) | (frame->dlc & 0x0F);      // DLC
    memcpy(&buf[5], frame->data, frame->dlc);
    
    // Use load TX buffer command
    uint8_t load_cmd;
    switch (buffer_num) {
        case 0: load_cmd = MCP2515_LOAD_TX0; break;
        case 1: load_cmd = MCP2515_LOAD_TX1; break;
        case 2: load_cmd = MCP2515_LOAD_TX2; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t tx[14];
    tx[0] = load_cmd;
    memcpy(&tx[1], buf, 5 + frame->dlc);
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, NULL, 6 + frame->dlc);
    xSemaphoreGive(s_driver.spi_mutex);
    
    return ESP_OK;
}

static void mcp2515_request_to_send(uint8_t buffer_mask)
{
    uint8_t tx[1] = {(uint8_t)(0x80 | buffer_mask)};
    
    xSemaphoreTake(s_driver.spi_mutex, portMAX_DELAY);
    spi_transfer(tx, NULL, 1);
    xSemaphoreGive(s_driver.spi_mutex);
}

// ============================================================================
// Interrupt Service Task
// ============================================================================

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t higher_priority_woken = pdFALSE;
    if (s_driver.isr_task_handle) {
        vTaskNotifyGiveFromISR(s_driver.isr_task_handle, &higher_priority_woken);
    }
    portYIELD_FROM_ISR(higher_priority_woken);
}

static void mcp2515_isr_task(void *arg)
{
    ESP_LOGI(TAG, "ISR task started");
    s_driver.isr_task_running = true;
    
    while (s_driver.isr_task_running) {
        // Wait for interrupt notification or timeout (for periodic status check)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        
        if (!s_driver.initialized || s_driver.state == CAN_STATE_STOPPED) {
            continue;
        }
        
        // Read interrupt flags
        uint8_t canintf = mcp2515_read_register(MCP2515_CANINTF);
        
        // Handle RX0 interrupt
        if (canintf & CANINT_RX0I) {
            can_frame_t frame;
            mcp2515_read_rx_buffer(0, &frame);
            
            // Dispatch to registered callbacks
            can_driver_dispatch_rx_callbacks(&frame);
            
            if (xQueueSend(s_driver.rx_queue, &frame, 0) == pdTRUE) {
                s_driver.stats.rx_frames++;
            } else {
                s_driver.stats.rx_overflows++;
            }
            
            mcp2515_bit_modify(MCP2515_CANINTF, CANINT_RX0I, 0);
        }
        
        // Handle RX1 interrupt
        if (canintf & CANINT_RX1I) {
            can_frame_t frame;
            mcp2515_read_rx_buffer(1, &frame);
            
            // Dispatch to registered callbacks
            can_driver_dispatch_rx_callbacks(&frame);
            
            if (xQueueSend(s_driver.rx_queue, &frame, 0) == pdTRUE) {
                s_driver.stats.rx_frames++;
            } else {
                s_driver.stats.rx_overflows++;
            }
            
            mcp2515_bit_modify(MCP2515_CANINTF, CANINT_RX1I, 0);
        }
        
        // Handle TX complete interrupts
        if (canintf & (CANINT_TX0I | CANINT_TX1I | CANINT_TX2I)) {
            mcp2515_bit_modify(MCP2515_CANINTF, 
                               CANINT_TX0I | CANINT_TX1I | CANINT_TX2I, 0);
        }
        
        // Handle error interrupt
        if (canintf & CANINT_ERRI) {
            uint8_t eflg = mcp2515_read_register(MCP2515_EFLG);
            s_driver.stats.tec = mcp2515_read_register(MCP2515_TEC);
            s_driver.stats.rec = mcp2515_read_register(MCP2515_REC);
            
            if (eflg & EFLG_TXBO) {
                s_driver.state = CAN_STATE_BUS_OFF;
                ESP_LOGE(TAG, "Bus-off condition detected");
            } else if (eflg & (EFLG_TXEP | EFLG_RXEP)) {
                s_driver.state = CAN_STATE_ERROR_PASSIVE;
            } else if (eflg & EFLG_EWARN) {
                s_driver.state = CAN_STATE_ERROR_WARNING;
            }
            
            if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
                s_driver.stats.rx_overflows++;
                // Clear overflow flags
                mcp2515_bit_modify(MCP2515_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
            }
            
            mcp2515_bit_modify(MCP2515_CANINTF, CANINT_ERRI, 0);
        }
        
        // Handle message error
        if (canintf & CANINT_MERR) {
            s_driver.stats.tx_errors++;
            mcp2515_bit_modify(MCP2515_CANINTF, CANINT_MERR, 0);
        }
    }
    
    ESP_LOGI(TAG, "ISR task stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// Public API
// ============================================================================

bool mcp2515_probe(void)
{
    if (!s_driver.initialized) {
        return false;
    }
    
    // Try to set config mode and verify
    mcp2515_reset();
    uint8_t canstat = mcp2515_read_register(MCP2515_CANSTAT);
    
    // After reset, MCP2515 should be in config mode (0x80)
    bool success = (canstat & CANSTAT_OPMOD_MASK) == CANSTAT_OPMOD_CONFIG;
    
    if (success) {
        ESP_LOGI(TAG, "MCP2515 detected (CANSTAT=0x%02X)", canstat);
    } else {
        ESP_LOGE(TAG, "MCP2515 not responding (CANSTAT=0x%02X)", canstat);
    }
    
    return success;
}

esp_err_t mcp2515_init(const mcp2515_config_t *config)
{
    if (s_driver.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    // Store configuration
    memcpy(&s_driver.config, config, sizeof(mcp2515_config_t));
    
    // Create mutex
    s_driver.spi_mutex = xSemaphoreCreateMutex();
    if (!s_driver.spi_mutex) {
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
        .max_transfer_sz = 32,
    };
    
    ret = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        goto err_spi_bus;
    }
    
    // Add SPI device
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,  // CPOL=0, CPHA=0
        .clock_speed_hz = config->spi_clock_hz,
        .spics_io_num = config->pin_cs,
        .queue_size = 4,
    };
    
    ret = spi_bus_add_device(config->spi_host, &dev_cfg, &s_driver.spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        goto err_spi_device;
    }
    
    s_driver.initialized = true;
    
    // Reset MCP2515 and verify communication
    if (!mcp2515_probe()) {
        ret = ESP_ERR_NOT_FOUND;
        goto err_probe;
    }
    
    // Configure bit timing (still in config mode after reset)
    ret = mcp2515_configure_bitrate(config->crystal_freq, config->can_bitrate);
    if (ret != ESP_OK) {
        goto err_bitrate;
    }
    
    // Configure RX buffers to receive any message (no filtering)
    mcp2515_write_register(MCP2515_RXB0CTRL, RXBCTRL_RXM_ANY | RXBCTRL_BUKT);  // Rollover enabled
    mcp2515_write_register(MCP2515_RXB1CTRL, RXBCTRL_RXM_ANY);
    
    // Clear all masks (accept all messages)
    mcp2515_write_register(MCP2515_RXM0SIDH, 0x00);
    mcp2515_write_register(MCP2515_RXM0SIDL, 0x00);
    mcp2515_write_register(MCP2515_RXM1SIDH, 0x00);
    mcp2515_write_register(MCP2515_RXM1SIDL, 0x00);
    
    // Create RX queue
    s_driver.rx_queue = xQueueCreate(config->rx_queue_len, sizeof(can_frame_t));
    if (!s_driver.rx_queue) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        ret = ESP_ERR_NO_MEM;
        goto err_queue;
    }
    
    // Configure interrupt pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_int),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // MCP2515 INT is active-low
    };
    gpio_config(&io_conf);
    
    // Install GPIO ISR service and add handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(config->pin_int, gpio_isr_handler, NULL);
    
    // Enable interrupts on MCP2515
    mcp2515_write_register(MCP2515_CANINTE, 
                           CANINT_RX0I | CANINT_RX1I |      // RX interrupts
                           CANINT_TX0I | CANINT_TX1I | CANINT_TX2I |  // TX interrupts
                           CANINT_ERRI | CANINT_MERR);       // Error interrupts
    
    // Create ISR handling task
    BaseType_t task_ret = xTaskCreate(mcp2515_isr_task, "mcp2515_isr", 
                                       4096, NULL, 10, &s_driver.isr_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ISR task");
        ret = ESP_ERR_NO_MEM;
        goto err_task;
    }
    
    s_driver.state = CAN_STATE_STOPPED;
    memset(&s_driver.stats, 0, sizeof(can_stats_t));
    
    ESP_LOGI(TAG, "Initialized successfully");
    return ESP_OK;

err_task:
    gpio_isr_handler_remove(config->pin_int);
    vQueueDelete(s_driver.rx_queue);
err_queue:
err_bitrate:
err_probe:
    spi_bus_remove_device(s_driver.spi_handle);
    s_driver.initialized = false;
err_spi_device:
    spi_bus_free(config->spi_host);
err_spi_bus:
    vSemaphoreDelete(s_driver.spi_mutex);
    return ret;
}

esp_err_t mcp2515_deinit(void)
{
    if (!s_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop ISR task
    s_driver.isr_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for task to finish
    
    // Remove GPIO ISR
    gpio_isr_handler_remove(s_driver.config.pin_int);
    
    // Put MCP2515 in config mode (stop CAN)
    mcp2515_set_mode(CANCTRL_REQOP_CONFIG);
    
    // Free resources
    vQueueDelete(s_driver.rx_queue);
    spi_bus_remove_device(s_driver.spi_handle);
    spi_bus_free(s_driver.config.spi_host);
    vSemaphoreDelete(s_driver.spi_mutex);
    
    s_driver.initialized = false;
    s_driver.state = CAN_STATE_UNINIT;
    
    ESP_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

esp_err_t mcp2515_start(void)
{
    if (!s_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t mode;
    if (s_driver.config.loopback) {
        mode = CANCTRL_REQOP_LOOPBACK;
        ESP_LOGI(TAG, "Starting in loopback mode");
    } else if (s_driver.config.listen_only) {
        mode = CANCTRL_REQOP_LISTENONLY;
        ESP_LOGI(TAG, "Starting in listen-only mode");
    } else {
        mode = CANCTRL_REQOP_NORMAL;
        ESP_LOGI(TAG, "Starting in normal mode");
    }
    
    esp_err_t ret = mcp2515_set_mode(mode);
    if (ret == ESP_OK) {
        s_driver.state = CAN_STATE_RUNNING;
    }
    
    return ret;
}

esp_err_t mcp2515_stop(void)
{
    if (!s_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = mcp2515_set_mode(CANCTRL_REQOP_CONFIG);
    if (ret == ESP_OK) {
        s_driver.state = CAN_STATE_STOPPED;
    }
    
    return ret;
}

esp_err_t mcp2515_send(const can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_driver.initialized || s_driver.state != CAN_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!frame || frame->dlc > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find an empty TX buffer
    uint8_t status = mcp2515_read_status();
    int buffer_num = -1;
    
    if (!(status & 0x04)) {  // TXB0CNTRL.TXREQ clear
        buffer_num = 0;
    } else if (!(status & 0x10)) {  // TXB1CNTRL.TXREQ clear
        buffer_num = 1;
    } else if (!(status & 0x40)) {  // TXB2CNTRL.TXREQ clear
        buffer_num = 2;
    }
    
    if (buffer_num < 0) {
        // All buffers full - could implement wait here
        s_driver.stats.tx_errors++;
        return ESP_ERR_TIMEOUT;
    }
    
    // Load frame into buffer
    esp_err_t ret = mcp2515_write_tx_buffer(buffer_num, frame);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Request to send
    mcp2515_request_to_send(1 << buffer_num);
    s_driver.stats.tx_frames++;
    
    return ESP_OK;
}

esp_err_t mcp2515_receive(can_frame_t *frame, uint32_t timeout_ms)
{
    if (!s_driver.initialized || !frame) {
        return ESP_ERR_INVALID_STATE;
    }
    
    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    if (xQueueReceive(s_driver.rx_queue, frame, ticks) == pdTRUE) {
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

uint32_t mcp2515_get_rx_queue_count(void)
{
    if (!s_driver.rx_queue) {
        return 0;
    }
    return uxQueueMessagesWaiting(s_driver.rx_queue);
}

QueueHandle_t mcp2515_get_rx_queue(void)
{
    return s_driver.rx_queue;
}

esp_err_t mcp2515_get_stats(can_stats_t *stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update error counters from hardware
    if (s_driver.initialized) {
        s_driver.stats.tec = mcp2515_read_register(MCP2515_TEC);
        s_driver.stats.rec = mcp2515_read_register(MCP2515_REC);
    }
    
    memcpy(stats, &s_driver.stats, sizeof(can_stats_t));
    return ESP_OK;
}

can_state_t mcp2515_get_state(void)
{
    return s_driver.state;
}

esp_err_t mcp2515_clear_errors(void)
{
    if (!s_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear error flags
    mcp2515_bit_modify(MCP2515_EFLG, 0xFF, 0x00);
    mcp2515_bit_modify(MCP2515_CANINTF, CANINT_ERRI | CANINT_MERR, 0);
    
    // Reset statistics
    s_driver.stats.tx_errors = 0;
    s_driver.stats.rx_errors = 0;
    s_driver.stats.rx_overflows = 0;
    
    // If we were in error state, try to return to running
    if (s_driver.state == CAN_STATE_ERROR_WARNING ||
        s_driver.state == CAN_STATE_ERROR_PASSIVE) {
        s_driver.state = CAN_STATE_RUNNING;
    }
    
    return ESP_OK;
}
