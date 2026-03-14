#ifndef WELDING_H
#define WELDING_H

#include "config.h"

/**
 * Welding state machine states
 */
typedef enum {
    WELD_STATE_IDLE = 0,     // Ready, waiting for trigger
    WELD_STATE_ARMED = 1,    // Contact detected (AUTO), waiting for S delay
    WELD_STATE_PRE_FIRE = 2, // Disabling charger, settling
    WELD_STATE_FIRING_P1 = 3,// Firing pulse 1
    WELD_STATE_PAUSE_1 = 4,  // Pause after P1 (before P2)
    WELD_STATE_FIRING_P2 = 5,// Firing pulse 2
    WELD_STATE_PAUSE_2 = 6,  // Pause after P2 (before P3)
    WELD_STATE_FIRING_P3 = 7,// Firing pulse 3
    WELD_STATE_PAUSE_3 = 8,  // Pause after P3 (before P4)
    WELD_STATE_FIRING_P4 = 9,// Firing pulse 4
    WELD_STATE_COOLDOWN = 10,// Re-enabling charger, settling
    WELD_STATE_BLOCKED = 11, // Blocked (low voltage or protection fault)
    WELD_STATE_ERROR = 12,   // Error state (hardware fault detected)
} weld_state_t;

/**
 * Protection status flags
 */
typedef struct {
    float supercap_voltage;       // Current supercap voltage (V)
    float protection_voltage;     // Current gate drive rail voltage (V)
    float contact_voltage;        // Current contact pin voltage (V)
    bool  low_voltage_warn;       // Below LOW_VOLTAGE_WARN
    bool  low_voltage_block;      // Below LOW_VOLTAGE_BLOCK — welding disabled
    bool  protection_fault;       // Gate drive rail out of range
    bool  contact_detected;       // Electrode contact detected (AUTO mode)
} weld_status_t;

// Global welding status (read by UI task on Core 0)
extern volatile weld_status_t g_weld_status;
extern volatile weld_state_t  g_weld_state;

/**
 * Initialize welding GPIO and state machine.
 */
void welding_init(void);

/**
 * FreeRTOS task: welding state machine + button monitoring.
 * Runs on Core 1 at highest priority.
 */
void welding_task(void *pvParameters);

/**
 * FreeRTOS task: ADC sampling for voltage monitoring.
 * Runs on Core 1, feeds data to welding logic and UI.
 */
void adc_task(void *pvParameters);

/**
 * Fire the weld pulse sequence (up to P1/T/P2/T/P3/T/P4).
 * Called internally by the state machine — NOT directly from UI.
 * BLOCKING during pulse sequence (max ~650ms with all 4 pulses).
 */
void welding_fire_pulse(void);

/**
 * Reset trigger flags and armed state.
 * Must be called after any parameter/preset change to prevent
 * stuck one-shot flags from blocking the trigger path.
 */
void welding_reset_state(void);

/**
 * Get the current weld state as a human-readable string.
 */
const char* welding_state_str(weld_state_t state);

/**
 * Check if the supercap charger is currently active.
 * @return true if charger is enabled (PIN_CHARGER_EN = LOW)
 */
bool welding_is_charging(void);

#endif // WELDING_H
