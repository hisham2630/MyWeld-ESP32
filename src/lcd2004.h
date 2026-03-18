/**
 * MyWeld ESP32-S3 — 20×4 Character LCD Driver (I2C, PCF8574)
 *
 * Drives an HD44780-compatible 4-line × 20-character LCD via I2C
 * backpack (PCF8574). Only compiled when DISPLAY_TYPE == DISPLAY_LCD_2004.
 *
 * STATUS: Placeholder — core framework ready.
 */

#ifndef LCD2004_H
#define LCD2004_H

#include "board_config.h"

#if HAS_CHAR_LCD

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize I2C and the HD44780 LCD.
 */
void lcd2004_init(void);

/**
 * Clear the entire display.
 */
void lcd2004_clear(void);

/**
 * Set cursor position.
 * @param row  0–3
 * @param col  0–19
 */
void lcd2004_set_cursor(uint8_t row, uint8_t col);

/**
 * Print a string at the current cursor position.
 */
void lcd2004_print(const char *text);

/**
 * Print a string at a specific row, clearing that row first.
 * @param row   0–3
 * @param text  Up to 20 characters
 */
void lcd2004_print_row(uint8_t row, const char *text);

/**
 * Set backlight on/off.
 * @param on  true = backlight on
 */
void lcd2004_backlight(bool on);

// --- Display HAL interface wrappers ---
void display_hal_init(void);
void display_hal_update(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
);
void display_hal_show_status(const char *message);
void display_hal_weld_fired(void);
void display_hal_set_brightness(uint8_t percent);
void display_hal_start_task(void);

#endif // HAS_CHAR_LCD
#endif // LCD2004_H
