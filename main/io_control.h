/**
 * @file io_control.h
 * @brief Hardware IO abstraction for Potentiometer, Peltier PWM, and Servo on ESP32-S3.
 */

#ifndef IO_CONTROL_H
#define IO_CONTROL_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize all hardware IOs (ADC and LEDC).
 * @return ESP_OK on success.
 */
esp_err_t io_control_init(void);

/**
 * @brief Get the raw 12-bit value from the potentiometer on GPIO4.
 * @return Value between 0-4095. Defaults to 0 if ADC fails.
 */
uint16_t io_get_pot_raw(void);

/**
 * @brief Update the Peltier cooling/heating PWM via GPIO5.
 * @param temp_c Temperature received from PIC ranging 20C-60C.
 */
void io_update_peltier(float temp_c);

/**
 * @brief Update the force feedback servo angle via GPIO7 based on raw pressure.
 * @param pressure_raw PIC raw pressure value (0-4095).
 */
void io_update_ff_servo(uint16_t pressure_raw);

#endif /* IO_CONTROL_H */
