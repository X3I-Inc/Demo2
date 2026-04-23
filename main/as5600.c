/**
 * @file as5600.c
 * @brief AS5600 magnetic rotary encoder – I2C driver implementation (Single encoder)
 */

#include "as5600.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "AS5600";

/* ── AS5600 Register Addresses ─────────────────────────────────────────── */
#define REG_RAW_ANGLE_H  0x0C   /* 12-bit raw angle [11:8] */
#define REG_RAW_ANGLE_L  0x0D   /* 12-bit raw angle  [7:0] */
#define REG_STATUS       0x0B   /* status register         */

/* Status register bits */
#define STATUS_MD  (1 << 5)     /* magnet detected   */

/* ── I2C helpers ───────────────────────────────────────────────────────── */

static esp_err_t as5600_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        AS5600_I2C_PORT,
        AS5600_I2C_ADDR,
        &reg, 1,
        data, len,
        pdMS_TO_TICKS(100)
    );
}

/* ── Public API ────────────────────────────────────────────────────────── */

esp_err_t as5600_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = AS5600_SDA_GPIO,
        .scl_io_num       = AS5600_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AS5600_I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(AS5600_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(AS5600_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Quick check – try reading status register */
    bool detected = false;
    err = as5600_magnet_detected(&detected);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AS5600 found on I2C bus (magnet %s)",
                 detected ? "DETECTED" : "NOT detected");
    } else {
        ESP_LOGW(TAG, "AS5600 not responding on I2C bus");
    }

    return err;
}

esp_err_t as5600_read_raw_angle(uint16_t *raw_angle)
{
    uint8_t buf[2];
    esp_err_t err = as5600_read_reg(REG_RAW_ANGLE_H, buf, 2);
    if (err != ESP_OK) return err;

    *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return ESP_OK;
}

esp_err_t as5600_read_angle_deg(float *degrees)
{
    uint16_t raw;
    esp_err_t err = as5600_read_raw_angle(&raw);
    if (err != ESP_OK) return err;

    *degrees = (float)raw * 360.0f / 4096.0f;
    return ESP_OK;
}

esp_err_t as5600_magnet_detected(bool *detected)
{
    uint8_t status;
    esp_err_t err = as5600_read_reg(REG_STATUS, &status, 1);
    if (err != ESP_OK) return err;

    *detected = (status & STATUS_MD) != 0;
    return ESP_OK;
}
