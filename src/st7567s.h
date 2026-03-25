/**
 * MyWeld ESP32-S3 — ST7567S 128×64 COG Graphic LCD Driver (I2C)
 *
 * Framebuffer-based driver for a 128×64 monochrome graphic LCD
 * using the ST7567S controller over I2C (4-pin: SDA/SCL/VCC/GND).
 *
 * Only compiled when DISPLAY_TYPE == DISPLAY_COG_12864.
 *
 * Dashboard Layout (128×64 pixels, landscape):
 *   ┌────────────────────────────────────────────┐ y=0
 *   │ 5.7V  ████████████████  95%   ⚡  B       │ 0–9    voltage bar
 *   ├────────────────────────────────────────────┤ y=11
 *   │ P1:05  T:00  P2:00                        │ 11–19  pulse row 1
 *   │ P3:00  P4:00  S:0.5                        │ 20–28  pulse row 2
 *   ├────────────────────────────────────────────┤ y=30
 *   │     >>> READY <<<                          │ 30–41  status (large)
 *   ├────────────────────────────────────────────┤ y=43
 *   │ AUTO   PR              ⚡ MyWeld           │ 43–51  mode + badge
 *   ├────────────────────────────────────────────┤ y=53
 *   │ ████████████████████████████████           │ 53–63  charge graph
 *   └────────────────────────────────────────────┘ y=64
 *
 * Wiring: SDA → GPIO8, SCL → GPIO9, VCC → 3.3V, GND → GND
 */

#ifndef ST7567S_H
#define ST7567S_H

#include "board_config.h"

#if HAS_COG_LCD

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Display Constants
// ============================================================================
#define COG_WIDTH           128
#define COG_HEIGHT          64
#define COG_PAGES           (COG_HEIGHT / 8)  // 8 pages
#define COG_FB_SIZE         (COG_WIDTH * COG_PAGES)  // 1024 bytes

// ============================================================================
// Low-Level API
// ============================================================================

/**
 * Initialize I2C bus and ST7567S controller.
 * Performs I2C bus scan to auto-detect the display address.
 */
void st7567s_init(void);

/**
 * Clear the entire framebuffer (all black or all white).
 * @param color  0 = black (all pixels off), 1 = white (all pixels on)
 */
void st7567s_clear(uint8_t color);

/**
 * Set display contrast (electronic volume).
 * @param value  0–63 (higher = darker)
 */
void st7567s_set_contrast(uint8_t value);

/**
 * Flush the framebuffer to the display.
 * Uses dirty-page tracking to minimize I2C traffic.
 */
void st7567s_flush(void);

// ============================================================================
// Drawing Primitives (operate on framebuffer, call flush to display)
// ============================================================================

/**
 * Set/clear a single pixel.
 * @param x      0–127
 * @param y      0–63
 * @param color  1 = pixel on (white), 0 = pixel off (black)
 */
void st7567s_pixel(int16_t x, int16_t y, uint8_t color);

/**
 * Draw a horizontal line.
 */
void st7567s_hline(int16_t x, int16_t y, int16_t w, uint8_t color);

/**
 * Draw a vertical line.
 */
void st7567s_vline(int16_t x, int16_t y, int16_t h, uint8_t color);

/**
 * Draw a rectangle outline.
 */
void st7567s_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

/**
 * Draw a filled rectangle.
 */
void st7567s_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

/**
 * Draw text using the built-in 5×7 font.
 * @param x      Pixel X position
 * @param y      Pixel Y position (top of character)
 * @param text   Null-terminated string
 * @param color  1 = white on black, 0 = black on white (inverted)
 */
void st7567s_text(int16_t x, int16_t y, const char *text, uint8_t color);

/**
 * Draw text using double-height characters (10×14).
 * Used for the status line (READY, ARMED, etc.)
 */
void st7567s_text_large(int16_t x, int16_t y, const char *text, uint8_t color);

/**
 * Get the pixel width of a text string in the 5×7 font.
 * @return Width in pixels (6px per char including spacing)
 */
int16_t st7567s_text_width(const char *text);

// ============================================================================
// Display HAL Interface (called by ui_stubs.c / main.c)
// ============================================================================

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

#endif // HAS_COG_LCD
#endif // ST7567S_H
