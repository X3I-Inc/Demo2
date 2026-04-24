/**
 * @file encoder_mux.c
 * @brief Read 4 × AS5600 encoders through a PCA9548A (TCA9548A) I2C mux
 *
 * Mux address: 0x70  (A0=A1=A2=GND)
 * Encoder channels: 0, 1, 6, 7
 *
 * A FreeRTOS task polls every 200 ms and prints all four angles to the
 * serial monitor.  Nothing else in the project is modified.
 */

#include "encoder_mux.h"
#include "as5600.h"          /* reuse AS5600_I2C_PORT, AS5600_I2C_ADDR */
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static const char *TAG = "ENC_MUX";

/* ── PCA9548A / TCA9548A configuration ────────────────────────────────── */
static uint8_t mux_i2c_addr = 0x70; /* Will auto-detect between 0x70-0x77 */

/* Which mux channels have encoders attached */
#define NUM_ENCODERS        4
static const uint8_t encoder_channels[NUM_ENCODERS] = { 0, 1, 6, 7 };

/* ── AS5600 register addresses (same as in as5600.c) ──────────────────── */
#define REG_RAW_ANGLE_H     0x0C
#define REG_STATUS           0x0B
#define STATUS_MD            (1 << 5)

/* ── Mux helpers ──────────────────────────────────────────────────────── */

/**
 * @brief Select a single channel on the PCA9548A multiplexer.
 * @param channel  0-7
 */
static esp_err_t mux_select_channel(uint8_t channel)
{
    uint8_t data = (uint8_t)(1 << channel);
    return i2c_master_write_to_device(
        AS5600_I2C_PORT,
        mux_i2c_addr,
        &data, 1,
        pdMS_TO_TICKS(100)
    );
}

/**
 * @brief Disable all mux channels (optional – keeps bus clean).
 */
static esp_err_t mux_disable_all(void)
{
    uint8_t data = 0x00;
    return i2c_master_write_to_device(
        AS5600_I2C_PORT,
        mux_i2c_addr,
        &data, 1,
        pdMS_TO_TICKS(100)
    );
}

/* ── Per-encoder read (through selected mux channel) ──────────────────── */

static esp_err_t read_encoder_raw(uint16_t *raw_angle)
{
    uint8_t reg = REG_RAW_ANGLE_H;
    uint8_t buf[2];
    esp_err_t err = i2c_master_write_read_device(
        AS5600_I2C_PORT,
        AS5600_I2C_ADDR,
        &reg, 1,
        buf, 2,
        pdMS_TO_TICKS(100)
    );
    if (err != ESP_OK) return err;

    *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return ESP_OK;
}

/* ── Global storage for other tasks ───────────────────────────────────── */
static float g_angles[NUM_ENCODERS] = {0.0f};
static bool  g_angles_ok[NUM_ENCODERS] = {0};

void encoder_mux_get_angles(float deg_out[4], bool ok_out[4])
{
    if (!deg_out || !ok_out) return;
    for (int i = 0; i < NUM_ENCODERS; i++) {
        deg_out[i] = g_angles[i];
        ok_out[i]  = g_angles_ok[i];
    }
}

/* ── Background task ──────────────────────────────────────────────────── */

#define AVG_SAMPLES 16

static void encoder_mux_task(void *arg)
{
    (void)arg;

    /* Short initial delay so the rest of init finishes */
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Encoder mux task started – reading %d encoders on channels 0,1,6,7",
             NUM_ENCODERS);

    uint16_t raw_history[NUM_ENCODERS][AVG_SAMPLES] = {0};
    uint8_t  hist_idx[NUM_ENCODERS] = {0};
    bool     hist_filled[NUM_ENCODERS] = {false};

    while (1) {
        uint16_t raw[NUM_ENCODERS];
        float    deg[NUM_ENCODERS];
        bool     ok[NUM_ENCODERS];

        for (int i = 0; i < NUM_ENCODERS; i++) {
            esp_err_t err = mux_select_channel(encoder_channels[i]);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Mux select ch%d failed: %s",
                         encoder_channels[i], esp_err_to_name(err));
                ok[i] = false;
                continue;
            }

            /* The TCA9548A switches channels almost instantly. We use a 100us delay instead of a 2ms yield */
            ets_delay_us(100);

            err = read_encoder_raw(&raw[i]);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Encoder ch%d read failed: %s",
                         encoder_channels[i], esp_err_to_name(err));
                ok[i] = false;
            } else {
                /* Add to moving average history */
                raw_history[i][hist_idx[i]] = raw[i];
                hist_idx[i] = (hist_idx[i] + 1) % AVG_SAMPLES;
                if (hist_idx[i] == 0) {
                    hist_filled[i] = true;
                }

                uint32_t sum = 0;
                int count = hist_filled[i] ? AVG_SAMPLES : hist_idx[i];
                if (count > 0) {
                    for (int j = 0; j < count; j++) {
                        sum += raw_history[i][j];
                    }
                    raw[i] = sum / count;
                }

                deg[i] = (float)raw[i] * 360.0f / 4096.0f;
                ok[i]  = true;
            }
        }

        /* Disable mux after round of reads */
        mux_disable_all();

        /* Update global cache for other tasks */
        for (int i = 0; i < NUM_ENCODERS; i++) {
            g_angles[i] = deg[i];
            g_angles_ok[i] = ok[i];
        }

        /* ── Print all four encoder values in one line ──────────────── */
        ESP_LOGI(TAG,
            "ENC0: %s%5.1f deg | ENC1: %s%5.1f deg | ENC2: %s%5.1f deg | ENC3: %s%5.1f deg",
            ok[0] ? "" : "ERR ", ok[0] ? deg[0] : 0,
            ok[1] ? "" : "ERR ", ok[1] ? deg[1] : 0,
            ok[2] ? "" : "ERR ", ok[2] ? deg[2] : 0,
            ok[3] ? "" : "ERR ", ok[3] ? deg[3] : 0
        );

        /* Read encoders at 100Hz+ so NRF always transmits the freshest possible target */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── Public API ───────────────────────────────────────────────────────── */

esp_err_t encoder_mux_start(void)
{
    esp_err_t err = ESP_FAIL;
    
    /* 1. Sweep to find the exact mux address (0x70-0x77) */
    for (uint8_t addr = 0x70; addr <= 0x77; addr++) {
        mux_i2c_addr = addr;
        /* Sending 0x00 to mux turns off all channels, safe & confirms presence */
        if (mux_disable_all() == ESP_OK) {
            ESP_LOGI(TAG, "PCA9548A mux found at 0x%02X !!", addr);
            err = ESP_OK;
            break;
        }
    }

    /* 2. If it is NOT found, run a full I2C scan to help you debug wiring */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9548A mux not responding on any address 0x70-0x77!");
        ESP_LOGI(TAG, "--- I2C BUS SCAN (SDA: 42, SCL: 41) ---");
        int devices_found = 0;
        for (uint8_t i = 1; i < 127; i++) {
            /* Try a 0-byte write to see if any device ACKs the address */
            if (i2c_master_write_to_device(AS5600_I2C_PORT, i, NULL, 0, pdMS_TO_TICKS(10)) == ESP_OK) {
                ESP_LOGI(TAG, "Found unknown I2C device at: 0x%02X", i);
                devices_found++;
            }
        }
        if (devices_found == 0) {
            ESP_LOGE(TAG, "NO I2C DEVICES FOUND! Double-check your SDA(42) / SCL(41) wiring, GND, and 3.3V.");
        } else {
            ESP_LOGI(TAG, "Scan complete - found %d device(s), but none were the multiplexer.", devices_found);
        }
        return err;
    }

    ESP_LOGI(TAG, "Starting encoder task...");

    BaseType_t ret = xTaskCreate(
        encoder_mux_task,
        "enc_mux",
        4096,           /* stack size */
        NULL,
        5,              /* priority */
        NULL
    );

    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}
