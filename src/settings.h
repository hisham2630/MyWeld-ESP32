#ifndef SETTINGS_H
#define SETTINGS_H

#include "config.h"

/**
 * Application settings stored in NVS.
 * Loaded on boot, saved on change (debounced to prevent flash wear).
 */
typedef struct {
    // Active weld parameters (4-pulse system)
    float p1;                    // Pulse 1 (ms) — always active, 5–50
    float t;                     // Shared pause between pulses (ms), 0 = single pulse
    float p2;                    // Pulse 2 (ms), 0 = disabled
    float p3;                    // Pulse 3 (ms), 0 = disabled
    float p4;                    // Pulse 4 (ms), 0 = disabled
    float s_value;               // AUTO mode delay (s)
    bool  auto_mode;             // true = AUTO, false = MAN

    // UI settings
    uint8_t brightness;          // Display brightness (0–100)
    uint8_t volume;              // Master sound volume (0–100%)
    bool    sound_on;            // Audio enable/disable
    uint8_t theme;               // 0 = dark, 1 = light

    // Counters
    uint32_t session_welds;      // Session counter (resets on boot)
    uint32_t total_welds;        // Lifetime counter (persists)

    // Presets
    uint8_t       active_preset; // Currently selected preset index
    weld_preset_t presets[MAX_PRESETS];

    // ADC calibration factors
    float adc_cal_voltage;       // Multiplier for supercap ADC
    float adc_cal_protection;    // Multiplier for protection ADC

    // BLE
    char ble_name[32];           // BLE device name
    char pin[PIN_MAX_LEN];       // Connection PIN (4 ASCII digits, null-terminated)
} app_settings_t;

// Global settings instance
extern app_settings_t g_settings;

/**
 * Initialize settings: load from NVS or set factory defaults.
 */
void settings_init(void);

/**
 * Initialize calibration from dedicated NVS partition.
 * Migrates from legacy settings if needed. Call AFTER settings_init().
 */
void calibration_init(void);

/**
 * Save current calibration factors to dedicated NVS partition.
 * Call after ADC calibration is performed.
 */
void calibration_save(void);

/**
 * Save current settings to NVS (debounced).
 * Call this after any parameter change.
 */
void settings_save(void);

/**
 * Force-save settings immediately (bypass debounce).
 * Use before shutdown or after weld counter increment.
 */
void settings_save_now(void);

/**
 * Reset all settings to factory defaults.
 * Erases NVS and reloads defaults.
 */
void settings_factory_reset(void);

/**
 * Load a preset's values into the active parameters.
 * @param index Preset index (0 to MAX_PRESETS-1)
 */
void settings_load_preset(uint8_t index);

/**
 * Switch to User Defined mode (pulses editable from dashboard).
 * Current pulse values are preserved; only active_preset changes.
 */
void settings_set_user_defined(void);

/**
 * Save explicit parameters into a preset slot.
 * @param index    Preset index (0 to MAX_PRESETS-1)
 * @param name     Preset name (max PRESET_NAME_LEN-1 chars)
 * @param p1       P1 / warm-up (ms), P1 ≤ P2
 * @param t        Pause duration (ms), 20–150
 * @param p2       P2 / main weld (ms), peak
 * @param p3       P3 / forge (ms), P3 ≤ P2
 * @param p4       P4 / temper (ms), P4 ≤ P3
 */
void settings_save_preset(uint8_t index, const char *name,
                          float p1, float t, float p2,
                          float p3, float p4);

/**
 * Verify a PIN string against the stored PIN.
 * @return true if match
 */
bool settings_verify_pin(const char *pin);

/**
 * Change the stored PIN.
 * @param new_pin 4 ASCII digits, null-terminated
 */
void settings_change_pin(const char *new_pin);

/**
 * Increment weld counter (session + total).
 * Auto-saves total to NVS.
 */
void settings_increment_weld_count(void);

#endif // SETTINGS_H
