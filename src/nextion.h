/**
 * MyWeld ESP32-S3 — Nextion HMI Display Driver
 *
 * UART-based driver for Nextion intelligent displays.
 * The Nextion display handles its own UI rendering — the ESP32 only
 * sends data updates and receives touch events via serial protocol.
 *
 * Protocol: Nextion Instruction Set (each command terminated by 0xFF 0xFF 0xFF)
 *
 * Only compiled when DISPLAY_TYPE == DISPLAY_NEXTION.
 *
 * STATUS: Placeholder — implement after user confirms Nextion model.
 */

#ifndef NEXTION_H
#define NEXTION_H

#include "board_config.h"

#if HAS_NEXTION

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize UART for Nextion communication.
 */
void nextion_init(void);

/**
 * Send a raw command string to the Nextion display.
 * The 0xFF 0xFF 0xFF terminator is appended automatically.
 *
 * @param cmd  Nextion instruction string (e.g., "t0.txt=\"Hello\"")
 */
void nextion_send_cmd(const char *cmd);

/**
 * Update a text field on the Nextion display.
 *
 * @param obj_name  Object name (e.g., "t0", "tStatus")
 * @param text      Text to display
 */
void nextion_set_text(const char *obj_name, const char *text);

/**
 * Update a numeric value on the Nextion display.
 *
 * @param obj_name  Object name (e.g., "n0", "nVoltage")
 * @param value     Integer value
 */
void nextion_set_value(const char *obj_name, int32_t value);

/**
 * Update a progress bar on the Nextion display.
 *
 * @param obj_name  Object name (e.g., "j0")
 * @param percent   0–100
 */
void nextion_set_progress(const char *obj_name, uint8_t percent);

/**
 * Switch to a different page on the Nextion display.
 *
 * @param page_id  Page number (0, 1, 2, ...)
 */
void nextion_goto_page(uint8_t page_id);

/**
 * Process incoming touch events from the Nextion display.
 * Should be called periodically (e.g., from a FreeRTOS task).
 * Returns true if an event was processed.
 */
bool nextion_process_events(void);

/**
 * Full status update — pushes all welding parameters to the display.
 * Called periodically from the welding/UI loop.
 */
void nextion_update_status(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
);

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

#endif // HAS_NEXTION
#endif // NEXTION_H
