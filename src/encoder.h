#ifndef ENCODER_H
#define ENCODER_H

/**
 * Rotary Encoder Driver (KY-040 style)
 *
 * Hardware:
 *   S1 (CLK) → IO15  (quadrature A)
 *   S2 (DT)  → IO17  (quadrature B)
 *   KEY (SW) → IO18  (push button, active LOW)
 *   5V       → 5V buck
 *   GND      → common
 *
 * Architecture:
 *   - Rotation: ISR on S1 falling edge, reads S2 for direction
 *   - Button:   Polled from encoder_poll() with software debounce
 *   - Events:   FreeRTOS queue, drained by UI task each frame
 *
 * Thread Safety:
 *   encoder_init() must be called from app_main (before tasks start).
 *   encoder_poll() is called exclusively from ui_task (Core 0).
 */

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ENC_EVENT_CW,          // Clockwise rotation (1 detent)
    ENC_EVENT_CCW,         // Counter-clockwise rotation (1 detent)
    ENC_EVENT_PRESS,       // Short press (<500ms)
    ENC_EVENT_LONG_PRESS   // Long press (≥500ms)
} encoder_event_t;

/**
 * Initialize encoder GPIO pins and ISR.
 * Installs GPIO ISR service if not already installed.
 * Must be called AFTER gpio_init_safe_defaults() in main.c.
 */
void encoder_init(void);

/**
 * Poll for next encoder event (non-blocking).
 *
 * Checks the rotation ISR queue and polls the button state.
 * Call this every UI frame (~10ms) from ui_task.
 *
 * @param event  Output: the next event if available
 * @return true if an event was produced, false if nothing happened
 */
bool encoder_poll(encoder_event_t *event);

#endif // ENCODER_H
