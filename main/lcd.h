/**
 * @file lcd.h
 * @brief HD44780 16×2 LCD driver – 4-bit parallel mode (no I2C)
 */

#ifndef LCD_H
#define LCD_H

#include <stdint.h>

/* ── Pin Defines ───────────────────────────────────────────────────────── */
#define LCD_RS_GPIO  17
#define LCD_E_GPIO   18
#define LCD_D4_GPIO   8
#define LCD_D5_GPIO   9
#define LCD_D6_GPIO  14
#define LCD_D7_GPIO  21

/**
 * @brief Initialise GPIO pins and run the HD44780 4-bit init sequence.
 */
void lcd_init(void);

/**
 * @brief Clear the display and return cursor to home.
 */
void lcd_clear(void);

/**
 * @brief Set the cursor position.
 * @param row  0 or 1
 * @param col  0–15
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Print a null-terminated string at the current cursor position.
 */
void lcd_print(const char *str);

/**
 * @brief Send a single character (data) to the LCD.
 */
void lcd_putchar(char c);

/**
 * @brief Send a raw command byte to the LCD.
 */
void lcd_command(uint8_t cmd);

#endif /* LCD_H */
