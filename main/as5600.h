/**
 * @file as5600.h
 * @brief AS5600 magnetic rotary encoder driver (I2C) - Single encoder
 */

#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ── I2C Configuration ─────────────────────────────────────────────────── */
#define AS5600_I2C_PORT     0
#define AS5600_SDA_GPIO     42
#define AS5600_SCL_GPIO     41
#define AS5600_I2C_FREQ_HZ  400000      /* 400 kHz */
#define AS5600_I2C_ADDR     0x36        /* 7-bit address */

/**
 * @brief Initialise the I2C bus for the AS5600.
 * @return ESP_OK on success.
 */
esp_err_t as5600_init(void);

/**
 * @brief Read the 12-bit raw angle (0–4095).
 * @param[out] raw_angle  pointer to store the raw angle value.
 * @return ESP_OK on success.
 */
esp_err_t as5600_read_raw_angle(uint16_t *raw_angle);

/**
 * @brief Read the angle in degrees (0.0–359.9).
 * @param[out] degrees  pointer to store the angle in degrees.
 * @return ESP_OK on success.
 */
esp_err_t as5600_read_angle_deg(float *degrees);

/**
 * @brief Check if a magnet is detected.
 * @param[out] detected  true if magnet is detected.
 * @return ESP_OK on success.
 */
esp_err_t as5600_magnet_detected(bool *detected);

#endif /* AS5600_H */
