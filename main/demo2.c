/**
 * @file demo2.c
 * @brief Main application – LCD menu UI with encoder + settings
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lcd.h"
#include "buttons.h"
#include "as5600.h"
#include "nrf24.h"
#include "arm_protocol.h"
#include "ui.h"
#include "encoder_mux.h"
#include "nrf_telemetry_task.h"
#include "io_control.h"

static const char *TAG = "APP";

void app_main(void)
{
    /* Initialise all hardware */
    io_control_init();
    lcd_init();
    buttons_init();

    esp_err_t err = as5600_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AS5600 init failed – encoder page will show errors");
    }

    /* Initialize NRF24L01 in TX mode */
    const uint8_t nrf_tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    /* Using channel 76, payload size 20 bytes (arm_motion_packet_t) */
    err = nrf24_init(nrf_tx_addr, 76, sizeof(arm_motion_packet_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NRF24L01 Init Failed");
    }

    /* Start the multiplexed encoder I2C task (5Hz) */
    encoder_mux_start();

    /* Start the dedicated NRF telemetry link task (50Hz) */
    nrf_telemetry_start();

    /* Start the menu UI (never returns) */
    ui_init();
    ui_run();
}
