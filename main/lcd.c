/**
 * @file lcd.c
 * @brief HD44780 16×2 LCD driver implementation – 4-bit parallel on ESP32-S3
 */

#include "lcd.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

static const char *TAG = "LCD";

/* ── HD44780 commands ──────────────────────────────────────────────────── */
#define CMD_CLEAR        0x01
#define CMD_HOME         0x02
#define CMD_ENTRY_MODE   0x06   /* increment, no shift              */
#define CMD_DISPLAY_ON   0x0C   /* display ON, cursor OFF, blink OFF */
#define CMD_FUNCTION_SET 0x28   /* 4-bit, 2-line, 5×8 font          */
#define CMD_SET_DDRAM    0x80   /* OR with address                   */

/* ── Low-level helpers ─────────────────────────────────────────────────── */

static void lcd_pulse_enable(void)
{
    gpio_set_level(LCD_E_GPIO, 1);
    ets_delay_us(2);
    gpio_set_level(LCD_E_GPIO, 0);
    ets_delay_us(2);
}

static void lcd_write_nibble(uint8_t nibble)
{
    gpio_set_level(LCD_D4_GPIO, (nibble >> 0) & 0x01);
    gpio_set_level(LCD_D5_GPIO, (nibble >> 1) & 0x01);
    gpio_set_level(LCD_D6_GPIO, (nibble >> 2) & 0x01);
    gpio_set_level(LCD_D7_GPIO, (nibble >> 3) & 0x01);
    lcd_pulse_enable();
}

static void lcd_send_byte(uint8_t data, int rs)
{
    gpio_set_level(LCD_RS_GPIO, rs);
    lcd_write_nibble(data >> 4);    /* high nibble */
    lcd_write_nibble(data & 0x0F);  /* low  nibble */
    ets_delay_us(50);               /* most commands need ≥37 µs */
}

/* ── Public API ────────────────────────────────────────────────────────── */

void lcd_command(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
}

void lcd_putchar(char c)
{
    lcd_send_byte((uint8_t)c, 1);
}

void lcd_clear(void)
{
    lcd_command(CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));   /* clear needs ~1.52 ms */
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? col : (0x40 + col);
    lcd_command(CMD_SET_DDRAM | addr);
}

void lcd_print(const char *str)
{
    while (*str) {
        lcd_putchar(*str++);
    }
}

static void lcd_gpio_init(void)
{
    const gpio_num_t pins[] = {
        LCD_RS_GPIO, LCD_E_GPIO,
        LCD_D4_GPIO, LCD_D5_GPIO, LCD_D6_GPIO, LCD_D7_GPIO
    };

    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
}

void lcd_init(void)
{
    lcd_gpio_init();

    /* Wait >40 ms after VCC rises to 2.7 V */
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(LCD_RS_GPIO, 0);

    /* Step 1 – Function set (8-bit), wait >4.1 ms */
    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Step 2 – Function set (8-bit), wait >100 µs */
    lcd_write_nibble(0x03);
    ets_delay_us(150);

    /* Step 3 – Function set (8-bit) */
    lcd_write_nibble(0x03);
    ets_delay_us(150);

    /* Step 4 – Switch to 4-bit mode */
    lcd_write_nibble(0x02);
    ets_delay_us(150);

    /* Now in 4-bit mode — configure the display */
    lcd_command(CMD_FUNCTION_SET);   /* 4-bit, 2 lines, 5×8 */
    lcd_command(CMD_DISPLAY_ON);     /* display ON           */
    lcd_clear();                     /* clear display        */
    lcd_command(CMD_ENTRY_MODE);     /* increment cursor     */

    ESP_LOGI(TAG, "LCD initialised (4-bit parallel)");
}
