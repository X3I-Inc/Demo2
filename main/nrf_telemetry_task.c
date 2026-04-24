/**
 * @file nrf_telemetry_task.c
 * @brief Dedicated background task to handle sending ESP angles and receiving PIC telemetry.
 */

#include "nrf_telemetry_task.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nrf24.h"
#include "arm_protocol.h"
#include "encoder_mux.h"
#include "io_control.h"

static const char *TAG = "NRF_TASK";

static void nrf_telemetry_loop(void *arg)
{
    (void)arg;
    static uint8_t nrf_seq = 0;

    ESP_LOGI(TAG, "Telemetry link started.");

    while (1) {
        /* 1. Fetch the latest cached angles from the mux task */
        float deg[4] = {0.0f};
        bool ok[4] = {false};
        encoder_mux_get_angles(deg, ok);

        /* 2. Build the NRF arm_motion_packet_t */
        arm_motion_packet_t pkt;
        float joints_deg[ARM_MAX_JOINTS] = {0.0f};
        for (int i = 0; i < 4; i++) {
            if (ok[i]) {
                joints_deg[i] = deg[i];
            }
        }
        
        /* Insert the mapped Potentiometer value into Joint 4 */
        uint16_t pot_raw = io_get_pot_raw();
        joints_deg[4] = ((float)pot_raw / 4095.0f) * 360.0f;

        uint8_t pkt_len = arm_build_motion_packet(&pkt, joints_deg, 0, ARM_MOTION_FLAG_ENABLE, nrf_seq++);
        
        /* 3. Transmit the packet */
        esp_err_t res = nrf24_transmit((const uint8_t *)&pkt, pkt_len);

        /* 4. Check for ACK payload containing PIC telemetry */
        float pic_temperature = 0.0f;
        uint16_t pic_pressure = 0;
        bool has_telemetry = false;

        if (res == ESP_OK) {
            arm_sensors_packet_t rx_pkt;
            uint8_t rx_len = nrf24_receive((uint8_t *)&rx_pkt, sizeof(rx_pkt));
            if (rx_len == sizeof(arm_sensors_packet_t)) {
                if (rx_pkt.hdr.magic == ARM_PROTO_MAGIC) {
                    pic_temperature = (float)rx_pkt.temperature_x10 / 10.0f;
                    pic_pressure = rx_pkt.pressure_raw;
                    has_telemetry = true;
                }
            }
        }

        /* 5. Drive the Force Feedback and Temp actuators */
        if (has_telemetry) {
            io_update_peltier(pic_temperature);
            io_update_ff_servo(pic_pressure);
        }

        /* 6. Periodically log the telemetry just for monitoring */
        if (has_telemetry && (nrf_seq % 25 == 0)) {
            /* Print roughly twice a second */
            ESP_LOGI(TAG, "LINK UP -> Pot: %4u | PIC Temp: %5.1f C | PIC P: %4u", 
                     pot_raw, pic_temperature, pic_pressure);
        } else if (res != ESP_OK && (nrf_seq % 50 == 0)) {
            ESP_LOGW(TAG, "Link timeout! Safety shutting off Peltier.");
            io_update_peltier(0.0f);
        }

        /* Sleep 20ms to hit ~50 Hz update rate (ARM_LINK_RATE_HZ) */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t nrf_telemetry_start(void)
{
    BaseType_t ret = xTaskCreate(
        nrf_telemetry_loop,
        "nrf_telemetry",
        4096,
        NULL,
        6,   /* slightly higher priority than the mux task */
        NULL
    );

    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}
