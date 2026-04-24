/**
 * @file nrf24.c
 * @brief NRF24L01+ SPI driver implementation
 *
 * Updated to support Dynamic Payload Length (DPL) to match PIC PRX config.
 * Includes SPI sanity check diagnostics and timing/CRC fixes.
 */

#include "nrf24.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "NRF24";

/* NRF24L01 Commands */
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define ACTIVATE      0x50
#define NOP_CMD       0xFF

/* NRF24L01 Registers */
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P0  0x0A
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D

static spi_device_handle_t spi;

/* ── SPI Low Level ─────────────────────────────────────────────────────── */

static uint8_t spi_transfer_byte(uint8_t data)
{
    uint8_t rx_data;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .rx_buffer = &rx_data
    };
    spi_device_polling_transmit(spi, &t);
    return rx_data;
}

static void nrf_write_register(uint8_t reg, const uint8_t *data, uint8_t len)
{
    gpio_set_level(NRF_CSN_PIN, 0);
    spi_transfer_byte(W_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer_byte(data[i]);
    }
    gpio_set_level(NRF_CSN_PIN, 1);
}

static void nrf_write_register_byte(uint8_t reg, uint8_t value)
{
    nrf_write_register(reg, &value, 1);
}

static uint8_t nrf_read_register_byte(uint8_t reg)
{
    gpio_set_level(NRF_CSN_PIN, 0);
    spi_transfer_byte(reg & 0x1F);
    uint8_t val = spi_transfer_byte(NOP_CMD);
    gpio_set_level(NRF_CSN_PIN, 1);
    return val;
}

/* Send a single-byte command (no data) */
static void nrf_command(uint8_t cmd)
{
    gpio_set_level(NRF_CSN_PIN, 0);
    spi_transfer_byte(cmd);
    gpio_set_level(NRF_CSN_PIN, 1);
}

/* Activate feature registers (required on some NRF24L01 clones) */
static void nrf_activate_features(void)
{
    gpio_set_level(NRF_CSN_PIN, 0);
    spi_transfer_byte(ACTIVATE);
    spi_transfer_byte(0x73);     /* Magic "activate" value */
    gpio_set_level(NRF_CSN_PIN, 1);
}

/* ── Public API ────────────────────────────────────────────────────────── */

esp_err_t nrf24_init(const uint8_t *address, uint8_t channel, uint8_t payload_size)
{
    if (payload_size > 32) return ESP_ERR_INVALID_ARG;

    /* 1. Init GPIOs */
    gpio_reset_pin(NRF_CE_PIN);
    gpio_set_direction(NRF_CE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(NRF_CE_PIN, 0);

    gpio_reset_pin(NRF_CSN_PIN);
    gpio_set_direction(NRF_CSN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(NRF_CSN_PIN, 1);

    /* 2. Init SPI Bus */
    spi_bus_config_t buscfg = {
        .miso_io_num = NRF_MISO_PIN,
        .mosi_io_num = NRF_MOSI_PIN,
        .sclk_io_num = NRF_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    /* Initialize the SPI bus */
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* Invalid state means it's already initialized, which is fine */
        ESP_LOGE(TAG, "Failed to init SPI bus");
        return ret;
    }

    /* 3. Add Device to SPI Bus - lowered to 1MHz for diagnostics */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,    /* 1 MHz (was 4MHz) - safer during bringup */
        .mode = 0,                    /* SPI mode 0 */
        .spics_io_num = -1,           /* We handle CS manually for NRF24 */
        .queue_size = 1,
    };
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); /* Longer wait for NRF power on reset */

    /* =========================================================
     * SPI SANITY CHECK (before any register writes)
     * CONFIG register default value after power-up is 0x08.
     * If we read 0x00 or 0xFF, SPI/wiring is broken.
     * ========================================================= */
    uint8_t cfg_initial = nrf_read_register_byte(REG_CONFIG);
    ESP_LOGI(TAG, "Initial CFG readback: 0x%02X (expected 0x08 after power-up)", cfg_initial);

    if (cfg_initial == 0x00 || cfg_initial == 0xFF) {
        ESP_LOGE(TAG, "*** NRF24 NOT RESPONDING via SPI ***");
        ESP_LOGE(TAG, "Check: MISO/MOSI swap, CSN wiring, 3.3V power, GND");
    }

    /* 4. Configure NRF24L01 */
    /* FIX: Changed 0x0A to 0x0E for 2-Byte CRC to match the PIC side */
    nrf_write_register_byte(REG_CONFIG, 0x0E);      /* PWR_UP=1, PRIM_RX=0 (TX mode), EN_CRC=1, CRCO=1 */
    vTaskDelay(pdMS_TO_TICKS(5));                   /* Wait for PWR_UP */

    nrf_write_register_byte(REG_EN_AA, 0x01);       /* Enable Auto ACK on pipe 0 */
    nrf_write_register_byte(REG_EN_RXADDR, 0x01);   /* Enable RX address on pipe 0 (required for ACK) */
    nrf_write_register_byte(REG_SETUP_AW, 0x03);    /* 5 byte address width */
    nrf_write_register_byte(REG_SETUP_RETR, 0x3A);  /* 1000us delay, 10 retries (matches PIC) */
    nrf_write_register_byte(REG_RF_CH, channel);    /* RF Channel */
    nrf_write_register_byte(REG_RF_SETUP, 0x06);    /* 0dBm, 1Mbps */

    /* =========================================================
     * Enable Dynamic Payload Length (DPL) + ACK payloads
     * Must match the PIC PRX side.
     * ========================================================= */
    nrf_activate_features();                        /* Required for some clones */
    nrf_write_register_byte(REG_FEATURE, 0x06);     /* EN_DPL + EN_ACK_PAY */
    nrf_write_register_byte(REG_DYNPD,   0x01);     /* DPL on pipe 0 */

    nrf_write_register(REG_TX_ADDR, address, 5);    /* Set TX address */
    /* Also need to set RX pipe 0 address to same for Auto-ACK to work */
    nrf_write_register(REG_RX_ADDR_P0, address, 5);

    /* RX_PW_P0 is ignored when DPL is enabled, but set anyway */
    nrf_write_register_byte(REG_RX_PW_P0, payload_size);

    /* Flush buffers */
    nrf_command(FLUSH_TX);
    nrf_command(FLUSH_RX);

    /* Clear status flags */
    nrf_write_register_byte(REG_STATUS, 0x70);

    /* =========================================================
     * FULL REGISTER READBACK - verifies SPI + config
     * ========================================================= */
    uint8_t cfg      = nrf_read_register_byte(REG_CONFIG);
    uint8_t en_aa    = nrf_read_register_byte(REG_EN_AA);
    uint8_t en_rx    = nrf_read_register_byte(REG_EN_RXADDR);
    uint8_t setup_aw = nrf_read_register_byte(REG_SETUP_AW);
    uint8_t rf_ch    = nrf_read_register_byte(REG_RF_CH);
    uint8_t rf_setup = nrf_read_register_byte(REG_RF_SETUP);
    uint8_t status   = nrf_read_register_byte(REG_STATUS);
    uint8_t feature  = nrf_read_register_byte(REG_FEATURE);
    uint8_t dynpd    = nrf_read_register_byte(REG_DYNPD);
    uint8_t rx_pw_p0 = nrf_read_register_byte(REG_RX_PW_P0);

    ESP_LOGI(TAG, "=== NRF24 Config Readback ===");
    /* Check updated to look for 0x0E instead of 0x0A */
    ESP_LOGI(TAG, "CFG     = 0x%02X (expected 0x0E)", cfg);
    ESP_LOGI(TAG, "EN_AA   = 0x%02X (expected 0x01)", en_aa);
    ESP_LOGI(TAG, "EN_RX   = 0x%02X (expected 0x01)", en_rx);
    ESP_LOGI(TAG, "SETUP_AW= 0x%02X (expected 0x03)", setup_aw);
    ESP_LOGI(TAG, "RF_CH   = %u    (expected %u)",    rf_ch, channel);
    ESP_LOGI(TAG, "RF_SETUP= 0x%02X (expected 0x06)", rf_setup);
    ESP_LOGI(TAG, "STATUS  = 0x%02X",                 status);
    ESP_LOGI(TAG, "FEATURE = 0x%02X (expected 0x06)", feature);
    ESP_LOGI(TAG, "DYNPD   = 0x%02X (expected 0x01)", dynpd);
    ESP_LOGI(TAG, "RX_PW_P0= %u",                     rx_pw_p0);
    ESP_LOGI(TAG, "=============================");

    if (cfg == 0x0E) {
        ESP_LOGI(TAG, "NRF24L01 initialized OK.");
    } else {
        ESP_LOGE(TAG, "NRF24L01 config FAILED - writes not sticking!");
    }

    return ESP_OK;
}

esp_err_t nrf24_transmit(const uint8_t *data, uint8_t len)
{
    /* Clear status flags */
    nrf_write_register_byte(REG_STATUS, 0x70);

    /* Flush TX FIFO to start clean */
    nrf_command(FLUSH_TX);

    /* Write payload to TX FIFO */
    gpio_set_level(NRF_CSN_PIN, 0);
    spi_transfer_byte(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++) {
        spi_transfer_byte(data[i]);
    }
    gpio_set_level(NRF_CSN_PIN, 1);

    /* Pulse CE to transmit */
    gpio_set_level(NRF_CE_PIN, 1);
    ets_delay_us(15);
    gpio_set_level(NRF_CE_PIN, 0);

    /* Wait for completion (TX_DS or MAX_RT) */
    uint8_t status;
    uint32_t timeout = 0;
    do {
        status = nrf_read_register_byte(REG_STATUS);
        
        /* FIX: Replaced FreeRTOS delay to guarantee exactly 1ms hardware delay per loop */
        ets_delay_us(1000); 
        
        timeout++;
        if (timeout > 50) {
            ESP_LOGW(TAG, "TX Timeout - STATUS=0x%02X", status);
            return ESP_FAIL;
        }
    } while (!(status & 0x30)); /* Bit 5: TX_DS, Bit 4: MAX_RT */

    nrf_write_register_byte(REG_STATUS, status & 0x30); /* Clear the flags */

    if (status & 0x10) { /* MAX_RT (Max retries reached) */
        /* Flush TX FIFO */
        nrf_command(FLUSH_TX);
        return ESP_FAIL;
    }

    return ESP_OK;
}

uint8_t nrf24_receive(uint8_t *data, uint8_t max_len)
{
    uint8_t status = nrf_read_register_byte(REG_STATUS);
    if (!(status & 0x40)) { /* RX_DR not set */
        return 0;
    }

    /* Read dynamic payload length */
    gpio_set_level(NRF_CSN_PIN, 0);
    spi_transfer_byte(0x60); // R_RX_PL_WID
    uint8_t len = spi_transfer_byte(0xFF);
    gpio_set_level(NRF_CSN_PIN, 1);

    if (len > 32) {
        nrf_command(FLUSH_RX);
        nrf_write_register_byte(REG_STATUS, 0x40);
        return 0;
    }

    /* Read payload data */
    gpio_set_level(NRF_CSN_PIN, 0);
    spi_transfer_byte(R_RX_PAYLOAD);
    uint8_t actual_len = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t val = spi_transfer_byte(0xFF);
        if (i < max_len) {
            data[i] = val;
            actual_len++;
        }
    }
    gpio_set_level(NRF_CSN_PIN, 1);

    /* Clear RX_DR flag */
    nrf_write_register_byte(REG_STATUS, 0x40);

    return actual_len;
}