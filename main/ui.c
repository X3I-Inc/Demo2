/**
 * @file ui.c
 * @brief Menu-driven UI for 16×2 LCD with 3-button navigation
 *
 * Button mapping:
 *   UP     (GPIO 48) – move cursor up / increment value
 *   DOWN   (GPIO 47) – move cursor down / decrement value
 *   SELECT (GPIO 40) – enter sub-menu / confirm / back
 *
 * Screen layout (16 chars × 2 lines):
 *   Line 0:  ">"  or " " + menu item  (or page header)
 *   Line 1:  ">"  or " " + menu item  (or page data)
 */

#include "ui.h"
#include "lcd.h"
#include "buttons.h"
#include "as5600.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "UI";

/* ── Screen IDs ────────────────────────────────────────────────────────── */
typedef enum {
    SCREEN_MAIN_MENU,
    SCREEN_ENCODER,
    SCREEN_SETTINGS,
    SCREEN_SETTINGS_REFRESH,
    SCREEN_SETTINGS_CONTRAST,
    SCREEN_SETTINGS_BACKLIGHT,
    SCREEN_ABOUT,
} screen_t;

/* ── State ─────────────────────────────────────────────────────────────── */
static screen_t  current_screen = SCREEN_MAIN_MENU;
static int       cursor_pos     = 0;       /* highlighted item index     */
static int       scroll_offset  = 0;       /* first visible item index   */
static bool      needs_redraw   = true;    /* flag to refresh display    */

/* Settings values (mock) */
static int       setting_refresh_hz  = 10;  /* 5, 10, 20, 50 */
static int       setting_contrast    = 5;   /* 0–10           */
static bool      setting_backlight   = true;

static uint16_t  smoothed_raw = 0;
static float     smoothed_deg = 0.0f;

/* ── Helpers ───────────────────────────────────────────────────────────── */

/** Write a 16-char padded string to a given LCD line */
static void lcd_write_line(uint8_t row, const char *text)
{
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", text);
    lcd_set_cursor(row, 0);
    lcd_print(buf);
}

/** Build a menu line: "> Item" or "  Item" */
static void lcd_write_menu_line(uint8_t row, const char *label, bool selected)
{
    char buf[17];
    snprintf(buf, sizeof(buf), "%c %-14s", selected ? '>' : ' ', label);
    lcd_set_cursor(row, 0);
    lcd_print(buf);
}

/** Wait for all buttons to be released before continuing */
static void wait_release(void)
{
    while (button_is_pressed(BTN_UP) ||
           button_is_pressed(BTN_DOWN) ||
           button_is_pressed(BTN_SELECT))
    {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(30));  /* extra settling time */
}

/** Clamp an int between min and max */
static int clamp(int val, int lo, int hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ======================================================================
 *  MAIN MENU
 * ====================================================================== */

static const char *main_menu_items[] = {
    "Encoder",
    "Settings",
    "About",
};
#define MAIN_MENU_COUNT  (sizeof(main_menu_items) / sizeof(main_menu_items[0]))

static void draw_main_menu(void)
{
    /* Show 2 items at a time, with scroll */
    for (int row = 0; row < 2; row++) {
        int idx = scroll_offset + row;
        if (idx < (int)MAIN_MENU_COUNT) {
            lcd_write_menu_line(row, main_menu_items[idx], idx == cursor_pos);
        } else {
            lcd_write_line(row, "");
        }
    }
}

static void handle_main_menu(void)
{
    if (button_is_pressed(BTN_DOWN)) {
        wait_release();
        if (cursor_pos < (int)MAIN_MENU_COUNT - 1) {
            cursor_pos++;
            if (cursor_pos >= scroll_offset + 2) scroll_offset++;
        }
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_UP)) {
        wait_release();
        if (cursor_pos > 0) {
            cursor_pos--;
            if (cursor_pos < scroll_offset) scroll_offset--;
        }
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_SELECT)) {
        wait_release();
        switch (cursor_pos) {
            case 0: current_screen = SCREEN_ENCODER;  break;
            case 1: current_screen = SCREEN_SETTINGS;  break;
            case 2: current_screen = SCREEN_ABOUT;     break;
        }
        cursor_pos = 0;
        scroll_offset = 0;
        needs_redraw = true;
    }
}

/* ======================================================================
 *  ENCODER PAGE
 * ====================================================================== */

static void draw_encoder(void)
{
    char line0[17], line1[17];
    snprintf(line0, sizeof(line0), "Angle: %6.1f\xDF", smoothed_deg);
    snprintf(line1, sizeof(line1), "Raw:%4u  [SEL]", smoothed_raw);
    lcd_write_line(0, line0);
    lcd_write_line(1, line1);
}

#include "nrf24.h"
#include "arm_protocol.h"

static void handle_encoder(void)
{
    /* Old direct I2C polling removed. 
       Encoders are now handled by encoder_mux task. */

    /* SELECT = back to main menu */
    if (button_is_pressed(BTN_SELECT)) {
        wait_release();
        current_screen = SCREEN_MAIN_MENU;
        cursor_pos = 0;
        scroll_offset = 0;
        needs_redraw = true;
        return;
    }
    /* Continuously refresh encoder data */
    needs_redraw = true;
}

/* ======================================================================
 *  SETTINGS MENU
 * ====================================================================== */

static const char *settings_menu_items[] = {
    "Refresh Rate",
    "Contrast",
    "Backlight",
    "Back",
};
#define SETTINGS_MENU_COUNT  (sizeof(settings_menu_items) / sizeof(settings_menu_items[0]))

static void draw_settings(void)
{
    for (int row = 0; row < 2; row++) {
        int idx = scroll_offset + row;
        if (idx < (int)SETTINGS_MENU_COUNT) {
            /* Show current value next to some items */
            char label[15];
            if (idx == 0)
                snprintf(label, sizeof(label), "Refresh: %dHz", setting_refresh_hz);
            else if (idx == 1)
                snprintf(label, sizeof(label), "Contrast: %d", setting_contrast);
            else if (idx == 2)
                snprintf(label, sizeof(label), "Backlit: %s", setting_backlight ? "ON" : "OFF");
            else
                snprintf(label, sizeof(label), "%s", settings_menu_items[idx]);

            lcd_write_menu_line(row, label, idx == cursor_pos);
        } else {
            lcd_write_line(row, "");
        }
    }
}

static void handle_settings(void)
{
    if (button_is_pressed(BTN_DOWN)) {
        wait_release();
        if (cursor_pos < (int)SETTINGS_MENU_COUNT - 1) {
            cursor_pos++;
            if (cursor_pos >= scroll_offset + 2) scroll_offset++;
        }
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_UP)) {
        wait_release();
        if (cursor_pos > 0) {
            cursor_pos--;
            if (cursor_pos < scroll_offset) scroll_offset--;
        }
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_SELECT)) {
        wait_release();
        switch (cursor_pos) {
            case 0: current_screen = SCREEN_SETTINGS_REFRESH;  break;
            case 1: current_screen = SCREEN_SETTINGS_CONTRAST; break;
            case 2: current_screen = SCREEN_SETTINGS_BACKLIGHT; break;
            case 3: /* Back */
                current_screen = SCREEN_MAIN_MENU;
                cursor_pos = 1;     /* return to "Settings" highlighted */
                scroll_offset = 0;
                break;
        }
        needs_redraw = true;
    }
}

/* ======================================================================
 *  SETTINGS VALUE EDITORS
 * ====================================================================== */

/* ── Refresh Rate ── */
static const int refresh_options[] = { 5, 10, 20, 50 };
#define REFRESH_OPT_COUNT  4

static void draw_settings_refresh(void)
{
    char line0[17], line1[17];
    snprintf(line0, sizeof(line0), "Refresh Rate:");
    snprintf(line1, sizeof(line1), "  < %3d Hz >   ", setting_refresh_hz);
    lcd_write_line(0, line0);
    lcd_write_line(1, line1);
}

static void handle_settings_refresh(void)
{
    if (button_is_pressed(BTN_UP)) {
        wait_release();
        /* Find next higher option */
        for (int i = 0; i < REFRESH_OPT_COUNT; i++) {
            if (refresh_options[i] > setting_refresh_hz) {
                setting_refresh_hz = refresh_options[i];
                break;
            }
        }
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_DOWN)) {
        wait_release();
        /* Find next lower option */
        for (int i = REFRESH_OPT_COUNT - 1; i >= 0; i--) {
            if (refresh_options[i] < setting_refresh_hz) {
                setting_refresh_hz = refresh_options[i];
                break;
            }
        }
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_SELECT)) {
        wait_release();
        current_screen = SCREEN_SETTINGS;
        cursor_pos = 0;
        scroll_offset = 0;
        needs_redraw = true;
        ESP_LOGI(TAG, "Refresh rate set to %d Hz", setting_refresh_hz);
    }
}

/* ── Contrast ── */
static void draw_settings_contrast(void)
{
    char line0[17], line1[17];
    snprintf(line0, sizeof(line0), "Contrast:");

    /* Visual bar: filled blocks */
    char bar[11];
    for (int i = 0; i < 10; i++) {
        bar[i] = (i < setting_contrast) ? '\xFF' : '-';  /* 0xFF = filled block on HD44780 */
    }
    bar[10] = '\0';
    snprintf(line1, sizeof(line1), " %s %2d", bar, setting_contrast);

    lcd_write_line(0, line0);
    lcd_write_line(1, line1);
}

static void handle_settings_contrast(void)
{
    if (button_is_pressed(BTN_UP)) {
        wait_release();
        setting_contrast = clamp(setting_contrast + 1, 0, 10);
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_DOWN)) {
        wait_release();
        setting_contrast = clamp(setting_contrast - 1, 0, 10);
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_SELECT)) {
        wait_release();
        current_screen = SCREEN_SETTINGS;
        cursor_pos = 1;
        scroll_offset = 0;
        needs_redraw = true;
        ESP_LOGI(TAG, "Contrast set to %d", setting_contrast);
    }
}

/* ── Backlight ── */
static void draw_settings_backlight(void)
{
    lcd_write_line(0, "Backlight:");
    lcd_write_line(1, setting_backlight ? "  [ ON  ] OFF  " : "   ON  [ OFF ] ");
}

static void handle_settings_backlight(void)
{
    if (button_is_pressed(BTN_UP) || button_is_pressed(BTN_DOWN)) {
        wait_release();
        setting_backlight = !setting_backlight;
        needs_redraw = true;
    }
    if (button_is_pressed(BTN_SELECT)) {
        wait_release();
        current_screen = SCREEN_SETTINGS;
        cursor_pos = 2;
        scroll_offset = 1;
        needs_redraw = true;
        ESP_LOGI(TAG, "Backlight %s", setting_backlight ? "ON" : "OFF");
    }
}

/* ======================================================================
 *  ABOUT PAGE
 * ====================================================================== */

static void draw_about(void)
{
    lcd_write_line(0, "  ESP32-S3 Demo");
    lcd_write_line(1, "  v1.0  [SEL]\xF0");  /* 0xF0 = back arrow-ish on some HD44780 */
}

static void handle_about(void)
{
    if (button_is_pressed(BTN_SELECT)) {
        wait_release();
        current_screen = SCREEN_MAIN_MENU;
        cursor_pos = 2;
        scroll_offset = 1;
        needs_redraw = true;
    }
}

/* ======================================================================
 *  UI ENGINE
 * ====================================================================== */

void ui_init(void)
{
    current_screen = SCREEN_ENCODER;  /* Jump straight to encoding so it prints to PC Terminal instantly */
    cursor_pos     = 0;
    scroll_offset  = 0;
    needs_redraw   = true;

    /* Splash screen */
    lcd_clear();
    lcd_write_line(0, "  ESP32-S3 Demo");
    lcd_write_line(1, "  Loading...");
    vTaskDelay(pdMS_TO_TICKS(1200));

    ESP_LOGI(TAG, "UI initialised");
}

void ui_run(void)
{
    while (1) {
        /* --- Draw current screen --- */
        if (needs_redraw) {
            switch (current_screen) {
                case SCREEN_MAIN_MENU:          draw_main_menu();          break;
                case SCREEN_ENCODER:            draw_encoder();            break;
                case SCREEN_SETTINGS:           draw_settings();           break;
                case SCREEN_SETTINGS_REFRESH:   draw_settings_refresh();   break;
                case SCREEN_SETTINGS_CONTRAST:  draw_settings_contrast();  break;
                case SCREEN_SETTINGS_BACKLIGHT: draw_settings_backlight(); break;
                case SCREEN_ABOUT:              draw_about();              break;
            }
            needs_redraw = false;
        }

        /* --- Handle input for current screen --- */
        switch (current_screen) {
            case SCREEN_MAIN_MENU:          handle_main_menu();          break;
            case SCREEN_ENCODER:            handle_encoder();            break;
            case SCREEN_SETTINGS:           handle_settings();           break;
            case SCREEN_SETTINGS_REFRESH:   handle_settings_refresh();   break;
            case SCREEN_SETTINGS_CONTRAST:  handle_settings_contrast();  break;
            case SCREEN_SETTINGS_BACKLIGHT: handle_settings_backlight(); break;
            case SCREEN_ABOUT:              handle_about();              break;
        }

        /* Tick rate depends on screen (encoder needs faster refresh) */
        int delay_ms = (current_screen == SCREEN_ENCODER)
                       ? (1000 / setting_refresh_hz)
                       : 50;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
