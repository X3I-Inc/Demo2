/**
 * @file nrf24.h
 * @brief NRF24L01+ SPI driver (TX mode focus)
 */

#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ── Hardware Pins ─────────────────────────────────────────────────────── */
#define NRF_CE_PIN      3
#define NRF_CSN_PIN     10
#define NRF_SCK_PIN     12
#define NRF_MOSI_PIN    11
#define NRF_MISO_PIN    13
#define NRF_IRQ_PIN     6

/**
 * @brief Initialise SPI and NRF24L01 in TX mode.
 * @param address 5-byte target address (e.g., {0xE7, 0xE7, 0xE7, 0xE7, 0xE7})
 * @param channel RF channel (0-125)
 * @param payload_size Size of the payload to transmit (1-32 bytes)
 * @return ESP_OK on success
 */
esp_err_t nrf24_init(const uint8_t *address, uint8_t channel, uint8_t payload_size);

/**
 * @brief Send a payload via NRF24L01.
 * @param data Pointer to the payload buffer
 * @param len Length in bytes (must match configured payload_size)
 * @return ESP_OK if ACK received (or successful transmit), ESP_FAIL on max retries.
 */
esp_err_t nrf24_transmit(const uint8_t *data, uint8_t len);

/**
 * @brief Check for and read data from the RX FIFO (e.g. ACK payloads).
 * @param data Buffer to store the received payload
 * @param max_len Maximum length to read
 * @return Number of bytes actually received/read. 0 if no data available.
 */
uint8_t nrf24_receive(uint8_t *data, uint8_t max_len);

#endif /* NRF24_H */
