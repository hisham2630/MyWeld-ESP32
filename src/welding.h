#ifndef WELDING_H
#define WELDING_H

#include "config.h"

/**
 * Welding state machine states
 */
typedef enum {
    WELD_STATE_IDLE,         // Ready, waiting for trigger
    WELD_STATE_ARMED,        // Contact detected (AUTO), waiting for S delay
    WELD_STATE_PRE_FIRE,     // Disabling charger, settling
    WELD_STATE_FIRING_P1,    // Firing pulse 1
    WELD_STATE_PAUSE,        // Pause between pulses
    WELD_STATE_FIRING_P2,    // Firing pulse 2
    WELD_STATE_COOLDOWN,     // Re-enabling charger, settling
    WELD_STATE_BLOCKED,      // Blocked (low voltage or protection fault)
    WELD_STATE_ERROR,        // Error state (hardware fault detected)
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
 * Fire the weld pulse sequence (P1/T/P2).
 * Called internally by the state machine — NOT directly from UI.
 * BLOCKING during pulse (max ~100ms).
 */
void welding_fire_pulse(void);

/**
 * Get the current weld state as a human-readable string.
 */
const char* welding_state_str(weld_state_t state);

#endif // WELDING_H
