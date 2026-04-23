/**
 * @file buttons.h
 * @brief Simple debounced button driver for 3 UI buttons
 */

#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include <stdint.h>

/* ── Pin Defines ───────────────────────────────────────────────────────── */
#define BTN_UP_GPIO     48
#define BTN_DOWN_GPIO   47
#define BTN_SELECT_GPIO 40

#define BTN_COUNT       3

typedef enum {
    BTN_UP = 0,
    BTN_DOWN,
    BTN_SELECT,
} button_id_t;

/**
 * @brief Initialise button GPIOs with internal pull-ups.
 */
void buttons_init(void);

/**
 * @brief Read the current (debounced) state of a button.
 * @return true if the button is currently pressed.
 */
bool button_is_pressed(button_id_t btn);

/**
 * @brief Get a human-readable name for a button.
 */
const char *button_name(button_id_t btn);

#endif /* BUTTONS_H */
