/**
 * @file nrf_telemetry_task.h
 * @brief Dedicated task for NRF24 communication (TX/RX) to keep telemetry separate from sensor reading.
 */

#ifndef NRF_TELEMETRY_TASK_H
#define NRF_TELEMETRY_TASK_H

#include "esp_err.h"

/**
 * @brief Start the dedicated NRF24 telemetry FreeRTOS task.
 * @return ESP_OK on success.
 */
esp_err_t nrf_telemetry_start(void);

#endif /* NRF_TELEMETRY_TASK_H */
