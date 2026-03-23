/**
 * MyWeld ESP32-S3 — Status LED Driver (WS2812 NeoPixel)
 *
 * Drives the onboard WS2812 RGB LED with animated patterns
 * for system state feedback. Uses ESP-IDF RMT peripheral.
 *
 * Only compiled on variants with HAS_RGB_LED = 1 (DevKit boards).
 * The JC3248W535 uses GPIO48 for the QSPI display, so no RGB LED.
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "board_config.h"

#if HAS_RGB_LED

#include <stdint.h>

// ============================================================================
// LED Events
// ============================================================================
typedef enum {
    // --- Transient overlays (play once, auto-return to base state) ---
    LED_EVT_STARTUP,            // Rainbow sweep → settle
    LED_EVT_BLE_CONNECTED,      // Blue triple-blink
    LED_EVT_BLE_DISCONNECTED,   // Orange/amber fade-out
    LED_EVT_WELD_FIRE,          // White flash → orange afterglow
    LED_EVT_CONTACT_DETECT,     // Yellow blink

    // --- Base states (persistent until another base state is set) ---
    LED_EVT_IDLE,               // Dim blue-white breathing
    LED_EVT_CHARGING,           // Green breathing
    LED_EVT_FULLY_CHARGED,      // Solid dim green
    LED_EVT_LOW_VOLTAGE,        // Red pulsing
    LED_EVT_PROTECTION_FAULT,   // Fast red strobe
    LED_EVT_OTA_ACTIVE,         // Cyan breathing (progress-based)
    LED_EVT_OTA_COMPLETE,       // Green flash → idle (transient)
} led_event_t;

/**
 * Initialize the WS2812 RMT driver and create the event queue.
 * Must be called before status_led_start_task().
 */
void status_led_init(void);

/**
 * Create and start the LED animation task on Core 0.
 */
void status_led_start_task(void);

/**
 * Send a state event to the LED task (non-blocking, ISR-safe from task context).
 * If the queue is full, the event is silently dropped.
 */
void status_led_set_event(led_event_t event);

/**
 * Update OTA progress for the LED animation (0–100%).
 * Only meaningful when LED_EVT_OTA_ACTIVE is the base state.
 */
void status_led_set_ota_progress(uint8_t percent);

#else

// ============================================================================
// No-op stubs for boards without RGB LED
// ============================================================================
#define status_led_init()              ((void)0)
#define status_led_start_task()        ((void)0)
#define status_led_set_event(e)        ((void)0)
#define status_led_set_ota_progress(p) ((void)0)

#endif // HAS_RGB_LED
#endif // STATUS_LED_H
