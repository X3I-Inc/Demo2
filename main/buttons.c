/**
 * @file buttons.c
 * @brief Debounced button driver implementation
 *
 * Buttons are assumed active-LOW (pressed = GND) with internal pull-ups enabled.
 * If your buttons are wired active-HIGH, flip the logic in button_is_pressed().
 */

#include "buttons.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

static const char *TAG = "BTN";

static const gpio_num_t btn_pins[BTN_COUNT] = {
    [BTN_UP]     = BTN_UP_GPIO,
    [BTN_DOWN]   = BTN_DOWN_GPIO,
    [BTN_SELECT] = BTN_SELECT_GPIO,
};

static const char *btn_names[BTN_COUNT] = {
    [BTN_UP]     = "UP",
    [BTN_DOWN]   = "DOWN",
    [BTN_SELECT] = "SELECT",
};

void buttons_init(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        gpio_reset_pin(btn_pins[i]);
        gpio_set_direction(btn_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(btn_pins[i], GPIO_PULLUP_ONLY);
    }
    ESP_LOGI(TAG, "Buttons initialised (GPIO %d, %d, %d)",
             BTN_UP_GPIO, BTN_DOWN_GPIO, BTN_SELECT_GPIO);
}

bool button_is_pressed(button_id_t btn)
{
    if (btn >= BTN_COUNT) return false;

    /* Active-LOW: pressed when GPIO reads 0 */
    /* Simple software debounce: read twice with a short delay */
    int first = gpio_get_level(btn_pins[btn]);
    ets_delay_us(5000);  /* 5 ms debounce */
    int second = gpio_get_level(btn_pins[btn]);

    return (first == 0 && second == 0);
}

const char *button_name(button_id_t btn)
{
    if (btn >= BTN_COUNT) return "???";
    return btn_names[btn];
}
