#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Display Module
 *
 * Initializes the JC3248W535 display (AXS15231B, QSPI) and bridges it
 * to LVGL 9.x. Uses a non-blocking flush pattern: flush_cb fires
 * draw_bitmap() async; the DMA-done ISR calls lv_display_flush_ready().
 */

void display_init(void);
void display_set_brightness(uint8_t percent);
uint8_t display_get_brightness(void);

/**
 * Lock/unlock LVGL mutex for thread-safe UI updates from other tasks.
 * MUST be used when calling any lv_* function from outside the UI task.
 */
bool display_lock(uint32_t timeout_ms);
void display_unlock(void);

/**
 * No-op: kept for backward compatibility only.
 * The non-blocking flush architecture no longer requires a task handle —
 * the DMA-done ISR calls lv_display_flush_ready() directly.
 */
void display_set_flush_task(void *task_handle);

/**
 * Returns the SPI LCD panel IO handle (for advanced use).
 */
void *display_get_io_handle(void);

#endif // DISPLAY_H
