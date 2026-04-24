/**
 * @file encoder_mux.h
 * @brief Read 4 × AS5600 encoders via PCA9548A / TCA9548A I2C multiplexer
 *
 * Mux channels used:
 *   Channel 0  (SD0/SC0) – Encoder 0
 *   Channel 1  (SD1/SC1) – Encoder 1
 *   Channel 6  (SD6/SC6) – Encoder 2
 *   Channel 7  (SD7/SC7) – Encoder 3
 *
 * Call encoder_mux_start() once from app_main().  It creates a FreeRTOS
 * task that periodically reads all four encoders and prints the angles
 * to the serial terminal via ESP_LOGI.
 */

#ifndef ENCODER_MUX_H
#define ENCODER_MUX_H

#include "esp_err.h"

/**
 * @brief Initialise the mux + encoder subsystem and start the
 *        background printing task.
 *
 * Uses the same I2C port / pins already configured in as5600.h.
 * The I2C driver must already be installed before calling this
 * (as5600_init() does that).
 *
 * @return ESP_OK on success.
 */
esp_err_t encoder_mux_start(void);

/**
 * @brief Get the most recently read angles from all 4 encoders.
 * @param deg_out Array to hold the 4 angle degrees
 * @param ok_out Array to hold the 4 success flags (true = valid read)
 */
void encoder_mux_get_angles(float deg_out[4], bool ok_out[4]);

#endif /* ENCODER_MUX_H */
