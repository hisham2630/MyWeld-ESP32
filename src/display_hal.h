/**
 * MyWeld ESP32-S3 — Display Hardware Abstraction Layer
 *
 * Provides a unified interface for all display types.
 * The actual implementation is selected at compile time based on DISPLAY_TYPE.
 *
 * For QSPI TFT:   display.c + ui.c (LVGL-based)
 * For Nextion:     nextion.c (UART-based, display handles rendering)
 * For LCD 20×4:    lcd2004.c (I2C character LCD)
 * For COG 128×64:  st7567s.c (I2C graphic LCD, framebuffer)
 */

#ifndef DISPLAY_HAL_H
#define DISPLAY_HAL_H

#include "board_config.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Unified Display Interface
// ============================================================================

/**
 * Initialize the display subsystem.
 * Automatically selects the correct driver based on DISPLAY_TYPE.
 */
void display_hal_init(void);

/**
 * Update the display with current welding parameters and status.
 * Called periodically from the UI task (or main loop for non-LVGL variants).
 *
 * @param voltage_v      Current supercap voltage
 * @param charge_pct     Charge percentage (0–100)
 * @param p1_ms          Pulse 1 duration (ms)
 * @param t_ms           Pause duration (ms)
 * @param p2_ms          Pulse 2 duration (ms)
 * @param p3_ms          Pulse 3 duration (ms)
 * @param p4_ms          Pulse 4 duration (ms)
 * @param auto_mode      true=AUTO, false=MAN
 * @param s_delay        AUTO mode contact delay (seconds)
 * @param status_text    Status text (e.g., "READY", "CHARGING", "ARMED")
 * @param ble_connected  true if BLE client is connected
 */
void display_hal_update(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
);

/**
 * Show a status message/error on the display.
 * For LVGL: uses the status label.
 * For Nextion: sends text to a designated text field.
 * For LCD: writes to a specific row.
 *
 * @param message  Status text to display
 */
void display_hal_show_status(const char *message);

/**
 * Show a weld-fired feedback (flash, animation, etc).
 * Implementation varies by display type.
 */
void display_hal_weld_fired(void);

/**
 * Set display brightness (if supported).
 * @param percent 0–100
 */
void display_hal_set_brightness(uint8_t percent);

/**
 * Start the UI task (for LVGL) or periodic update loop.
 * Should be called from main after all init is complete.
 */
void display_hal_start_task(void);

// ============================================================================
// Route to correct implementation header
// ============================================================================

#if (DISPLAY_TYPE == DISPLAY_QSPI_TFT)
  // LVGL-based — uses existing display.h + ui.h
  #include "display.h"
  #include "ui.h"
#elif (DISPLAY_TYPE == DISPLAY_NEXTION)
  #include "nextion.h"
#elif (DISPLAY_TYPE == DISPLAY_LCD_2004)
  #include "lcd2004.h"
#elif (DISPLAY_TYPE == DISPLAY_COG_12864)
  #include "st7567s.h"
#endif

#endif // DISPLAY_HAL_H
