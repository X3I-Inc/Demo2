/**
 * @file io_control.c
 * @brief Hardware IO abstraction driver
 */

#include "io_control.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"

static const char *TAG = "IO_CTRL";

/* ADC config */
#define POT_ADC_UNIT        ADC_UNIT_1
#define POT_ADC_CHANNEL     ADC_CHANNEL_3   /* GPIO4 on ESP32-S3 = ADC1_CH3 */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/* LEDC configs */
#define PELTIER_TIMER       LEDC_TIMER_0
#define PELTIER_MODE        LEDC_LOW_SPEED_MODE
#define PELTIER_CHANNEL     LEDC_CHANNEL_0
#define PELTIER_GPIO        5

#define SERVO_TIMER         LEDC_TIMER_1
#define SERVO_MODE          LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL       LEDC_CHANNEL_1
#define SERVO_GPIO          7

esp_err_t io_control_init(void)
{
    /* 1. Init ADC for Potentiometer */
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = POT_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADC unit: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    err = adc_oneshot_config_channel(s_adc_handle, POT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. Init LEDC for Peltier (5kHz, 8-bit resolution) */
    ledc_timer_config_t peltier_timer = {
        .speed_mode       = PELTIER_MODE,
        .timer_num        = PELTIER_TIMER,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&peltier_timer);

    ledc_channel_config_t peltier_chan = {
        .speed_mode     = PELTIER_MODE,
        .channel        = PELTIER_CHANNEL,
        .timer_sel      = PELTIER_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PELTIER_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&peltier_chan);

    /* 3. Init LEDC for Servo (50Hz, 14-bit resolution) */
    ledc_timer_config_t servo_timer = {
        .speed_mode       = SERVO_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&servo_timer);

    ledc_channel_config_t servo_chan = {
        .speed_mode     = SERVO_MODE,
        .channel        = SERVO_CHANNEL,
        .timer_sel      = SERVO_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&servo_chan);

    ESP_LOGI(TAG, "IO Peripherals initialized (ADC1_CH3, Peltier GPIO5, Servo GPIO7)");
    return ESP_OK;
}

uint16_t io_get_pot_raw(void)
{
    if (!s_adc_handle) return 0;
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, POT_ADC_CHANNEL, &raw) == ESP_OK) {
        return (uint16_t)raw;
    }
    return 0;
}

void io_update_peltier(float temp_c)
{
    /* Map 20C to 0% duty (0), 60C to 100% duty (255) */
    float mapped = 0;
    if (temp_c > 20.0f) {
        mapped = ((temp_c - 20.0f) / 40.0f) * 255.0f;
    }
    
    uint32_t duty = 0;
    if (mapped > 255.0f) {
        duty = 255;
    } else if (mapped > 0.0f) {
        duty = (uint32_t)mapped;
    }

    ledc_set_duty(PELTIER_MODE, PELTIER_CHANNEL, duty);
    ledc_update_duty(PELTIER_MODE, PELTIER_CHANNEL);
}

void io_update_ff_servo(uint16_t pressure_raw)
{
    /* Map raw pressure (0-4095) to servo PWM
       For 14-bit resolution at 50Hz (20ms):
       0.5ms (0 deg) = (0.5/20) * 16384 = 409
       2.5ms (180 deg) = (2.5/20) * 16384 = 2048
     */
    if (pressure_raw > 4095) pressure_raw = 4095;
    
    uint32_t duty = 409 + ((uint32_t)pressure_raw * (2048 - 409)) / 4095;
    
    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
}
