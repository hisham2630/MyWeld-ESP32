#ifndef UI_H
#define UI_H

#include "config.h"

/**
 * LVGL UI Module
 * 
 * Full dashboard interface for the MyWeld spot welder.
 * 
 * Main Screen:
 *   - Status bar (voltage, mode, BLE status)
 *   - P1/T/P2 parameter cards with +/- buttons (0.5ms steps)
 *   - S value (AUTO mode only)
 *   - Color-coded voltage bar (0–5.7V)
 *   - Live voltage history graph (30 seconds)
 *   - Weld counter (session / total)
 *   - Status indicator (READY / CHARGING / LOW / ERROR)
 *   - Settings gear icon
 * 
 * Settings Screen:
 *   - Brightness slider
 *   - Sound toggle
 *   - AUTO/MAN mode switch
 *   - Preset selector (10 slots)
 *   - Dark/Light theme toggle
 *   - ADC calibration
 *   - Weld counter reset
 *   - About / version
 */

/**
 * Initialize LVGL UI: create all screens, widgets, and register callbacks.
 * Must be called AFTER display_init().
 */
void ui_init(void);

/**
 * FreeRTOS task: LVGL tick handler and screen refresh.
 * Runs on Core 0.
 */
void ui_task(void *pvParameters);

/**
 * Update the voltage display (bar + number).
 * Called from ADC task via thread-safe LVGL messaging.
 * @param voltage Current supercap voltage (V)
 */
void ui_update_voltage(float voltage);

/**
 * Update the protection rail voltage display.
 * @param voltage Current gate drive rail voltage (V)
 */
void ui_update_protection(float voltage);

/**
 * Update the weld state indicator on screen.
 * @param state Current welding state
 */
void ui_update_weld_state(uint8_t state);

/**
 * Update the weld counter display.
 * @param session Session count
 * @param total   Lifetime count
 */
void ui_update_weld_count(uint32_t session, uint32_t total);

/**
 * Add a voltage data point to the history graph.
 * @param voltage Voltage reading (V)
 */
void ui_graph_add_point(float voltage);

/**
 * Switch between dark and light theme.
 * @param theme 0 = dark, 1 = light
 */
void ui_set_theme(uint8_t theme);

/**
 * Re-sync all parameter card labels (P1/T/P2/S), preset name, and mode
 * indicator from g_settings. Thread-safe: posts a message to the UI queue.
 * Call this after any BLE write that modifies g_settings fields.
 */
void ui_refresh_params(void);

/**
 * Show a notification toast on screen.
 * @param message Text to display
 * @param duration_ms How long to show (0 = until dismissed)
 */
void ui_show_notification(const char *message, uint16_t duration_ms);

/**
 * Trigger factory-reset countdown: shows a full-screen overlay with
 * "Factory Reset — Rebooting in... 3 2 1" then calls esp_restart().
 * Thread-safe: posts a message to the UI queue.
 */
void ui_trigger_reboot_countdown(void);

#endif // UI_H
