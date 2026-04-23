/**
 * @file ui.h
 * @brief Menu-driven UI system for 16×2 HD44780 LCD
 */

#ifndef UI_H
#define UI_H

#include <stdint.h>

/**
 * @brief Initialise the UI (must call lcd_init, buttons_init, as5600_init first).
 */
void ui_init(void);

/**
 * @brief Run the UI loop (never returns).
 */
void ui_run(void);

#endif /* UI_H */
