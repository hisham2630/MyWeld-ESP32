#ifndef CONFIG_H
#define CONFIG_H

/**
 * MyWeld ESP32-S3 — Hardware Configuration
 * Board: JC3248W535 (Guition ESP32-S3, AXS15231B display)
 * 
 * Original MyWeld V2.0 PRO designed by Aka Kasyan (YouTube)
 * https://www.youtube.com/@akakasyan
 * This project is an ESP32-S3 adaptation of his original Arduino Nano design.
 * Thank you Aka Kasyan for the inspiration and the excellent welding logic!
 * 
 * All pin assignments VERIFIED from:
 *   - Official JC3248W535 GPIO割当表 (2025/02/01)
 *   - Board schematic (JC3248W535 V1.0, 深圳市晶彩智能有限公司)
 */

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// GPIO Pin Definitions (VERIFIED from schematic + GPIO table)
// ============================================================================

// --- Welding Outputs ---
#define PIN_OUTPUT      GPIO_NUM_46  // MOSFET bank fire signal (P2 pin 6)
#define PIN_CHARGER_EN  GPIO_NUM_16  // Supercap charger KEY control (P2 pin 5)

// --- Physical Input ---
#define PIN_START       GPIO_NUM_14  // External weld button, MAN mode (P2 pin 8)

// --- ADC Inputs (all ADC1 channels for reliability) ---
#define PIN_VOLTAGE     GPIO_NUM_5   // Supercap voltage — ADC1_CH4 (P2 pin 1)
#define PIN_PROTECTION  GPIO_NUM_6   // Gate drive rail — ADC1_CH5 (P2 pin 2), safe up to ~25V
#define PIN_CONTACT     GPIO_NUM_7   // Electrode contact detect — ADC1_CH6 (P2 pin 3)

// --- Display (QSPI — internal, no user wiring) ---
#define PIN_LCD_CLK     GPIO_NUM_47
#define PIN_LCD_CS      GPIO_NUM_45
#define PIN_LCD_TE      GPIO_NUM_38
#define PIN_LCD_BL      GPIO_NUM_1   // Backlight PWM
#define PIN_LCD_D0      GPIO_NUM_21
#define PIN_LCD_D1      GPIO_NUM_48
#define PIN_LCD_D2      GPIO_NUM_40
#define PIN_LCD_D3      GPIO_NUM_39

// --- Touch I2C (from schematic: IO8=TP_SCL, IO4=TP_SDA) ---
#define PIN_TOUCH_SCL   GPIO_NUM_8   // SCL — confirmed from reference esp_bsp.h
#define PIN_TOUCH_SDA   GPIO_NUM_4   // SDA — confirmed from reference esp_bsp.h
#define TOUCH_I2C_ADDR  0x3B

// --- I2S Audio (built-in amplifier → speaker on P6) ---
#define PIN_I2S_LRCLK   GPIO_NUM_2   // I2S Word Select (WS)
#define PIN_I2S_BCLK    GPIO_NUM_42  // I2S Bit Clock
#define PIN_I2S_DOUT    GPIO_NUM_41  // I2S Data Out

// --- Rotary Encoder (KY-040 style, 5-pin module) ---
#define PIN_ENC_S1      GPIO_NUM_15  // CLK (quadrature A) — P2 pin 4
#define PIN_ENC_S2      GPIO_NUM_17  // DT  (quadrature B) — P3/P4
#define PIN_ENC_KEY     GPIO_NUM_18  // SW  (push button, active LOW) — P3/P4

// --- Spare GPIOs (available for future expansion) ---
#define PIN_SPARE_1     GPIO_NUM_9   // P2 pin 7 (ADC1_CH8) — reserved for temp sensor

// ============================================================================
// Display Configuration
// ============================================================================
#define DISPLAY_WIDTH   480
#define DISPLAY_HEIGHT  320

// ============================================================================
// ADC Configuration
// ============================================================================
#define ADC_RESOLUTION_BITS  12        // ESP32-S3: 12-bit (0–4095)
#define ADC_MAX_VALUE        4095
#define ADC_VREF             3.3f
#define ADC_NUM_SAMPLES      16         // Multi-sample averaging (noise reduction √16 = 4×)
#define ADC_EMA_ALPHA        0.2f       // EMA smoothing factor (lower = smoother, slower)

// Voltage divider multipliers (calculate real voltage from ADC reading)
#define SUPERCAP_V_MULT      (25.0f / 15.0f)   // 10k+15k divider: Vreal = Vadc × (R1+R2)/R2
#define PROTECTION_V_MULT    (115.0f / 15.0f)   // 100k+15k divider (safe up to ~25V)

// ============================================================================
// Supercap Bank (2S2P, 3.0V 3000F per cell)
// ============================================================================
#define SUPERCAP_MAX_V       5.7f     // Max charge voltage (derated from 6.0V for longevity)
#define SUPERCAP_FULL_V      5.5f     // UI: considered "fully charged"
#define LOW_VOLTAGE_WARN     4.0f     // UI: low voltage warning
#define LOW_VOLTAGE_BLOCK    3.0f     // Safety: refuse to weld below this
#define SUPERCAP_CAPACITY_F  3000.0f  // Bank capacitance in Farads

// ============================================================================
// Protection Thresholds
// ============================================================================
#define CONTACT_THRESHOLD_V  1.5f     // Contact detection voltage threshold
#define PROTECT_RAIL_MIN_V   10.0f    // Gate drive rail minimum (13.5V nominal)
#define PROTECT_RAIL_MAX_V   18.0f    // Gate drive rail maximum
// Below this level the protection rail is treated as "not connected / bench mode".
// A floating 47k+15k divider with no gate-drive reads 0.5–4V noise; the real
// 13.5V rail reads ≥10V. Anything between 8–10V would indicate a real fault.
#define PROTECT_RAIL_PLAUSIBLE_V 8.0f

// ============================================================================
// Pulse Parameters (4-pulse system: P1 → T → P2 → T → P3 → T → P4)
// ============================================================================
#define PULSE_HW_MIN_MS      1.0f     // Absolute hardware floor (BLE validation)
#define PULSE_MIN_MS         5.0f     // UI slider minimum (ms)
#define PULSE_MAX_MS         50.0f    // Maximum pulse duration (ms)
#define PULSE_STEP_MS        5.0f     // Step size for pulse adjustment
#define PULSE_OFF_VALUE      0.0f     // Special "disabled" value for P2/P3/P4

// Shared pause (T) between consecutive pulses
#define PAUSE_MIN_MS         20.0f    // Minimum pause when active (ms)
#define PAUSE_MAX_MS         150.0f   // Maximum pause (ms)
#define PAUSE_STEP_MS        5.0f     // Step size for pause adjustment

// Factory defaults
#define PULSE_DEFAULT_P1     5.0f     // Default P1 (ms) — always active
#define PULSE_DEFAULT_T      0.0f     // Default T (ms) — 0 = single-pulse mode
#define PULSE_DEFAULT_P2     0.0f     // Default P2 (ms) — 0 = disabled
#define PULSE_DEFAULT_P3     0.0f     // Default P3 (ms) — 0 = disabled
#define PULSE_DEFAULT_P4     0.0f     // Default P4 (ms) — 0 = disabled

// AUTO mode delay (contact-to-fire)
#define S_VALUE_MIN          0.3f     // Minimum contact delay (seconds)
#define S_VALUE_MAX          2.0f     // Maximum contact delay (seconds)
#define S_VALUE_STEP         0.1f     // Step size (seconds)
#define S_VALUE_DEFAULT      0.5f     // Factory default (seconds)

// ============================================================================
// Timing Constants
// ============================================================================
#define DEBOUNCE_MS          50       // Button debounce delay (ms) — requires 50/10+1=6
                                      // consecutive stable readings at 10ms polling
#define MAN_RETRIGGER_LOCKOUT_MS 300   // Post-weld lockout: ignore new triggers (EMI protection)
#define LOW_V_CONFIRM_MS     1500     // Confirm low voltage before blocking
#define PROTECT_CONFIRM_MS   1500     // Confirm protection fault before blocking
#define CHARGER_SETTLE_US    500      // Settle time before/after pulse (microseconds)
#define POST_PULSE_CHARGE_DELAY_MS 500 // Charge hold-off after pulse ends (ms)
#define NVS_SAVE_DEBOUNCE_MS 2000     // Debounce NVS writes (prevent flash wear)
#define ADC_SAMPLE_INTERVAL  500      // ADC sampling interval (ms) for voltage graph

// Encoder tuning
#define ENC_STEPS_PER_DETENT     5      // Gray-code micro-steps per physical detent click
#define ENC_KEY_DEBOUNCE_MS      30     // Button debounce (30ms: fast taps, still filters bounce)
#define ENC_LONG_PRESS_MS        500    // Long press threshold

// ============================================================================
// I2S Audio Configuration
// ============================================================================
#define AUDIO_SAMPLE_RATE    44100
#define AUDIO_BITS           16
#define AUDIO_CHANNELS       1        // Mono

// Tone frequencies (Hz) for audio feedback
#define TONE_BEEP            880      // Parameter change confirmation
#define TONE_WELD_FIRE       1200     // Pulse fired
#define TONE_READY_LOW       660      // Ready chime — first note
#define TONE_READY_HIGH      880      // Ready chime — second note
#define TONE_ERROR_HIGH      440      // Error alert — first note
#define TONE_ERROR_LOW       220      // Error alert — second note
#define TONE_CONTACT         1000     // AUTO mode contact detected
#define TONE_STARTUP_C       523      // Startup melody — C5
#define TONE_STARTUP_E       659      // Startup melody — E5
#define TONE_STARTUP_G       784      // Startup melody — G5
#define TONE_BLE_NOTE_1      587      // BLE connect chime — D5 (soft, welcoming)
#define TONE_BLE_NOTE_2      880      // BLE connect chime — A5 (ascending resolve)

// ============================================================================
// Preset Configuration
// ============================================================================
#define MAX_PRESETS           20   // 7 factory (read-only) + 13 user-custom
#define PRESET_USER_DEFINED   0xFF // Sentinel: pulse cards are user-editable (no preset loaded)
#define PRESET_NAME_LEN      20

// Preset data structure — pulse parameters only.
// S delay and auto_mode are device-level user preferences, NOT per-preset.
typedef struct {
    char  name[PRESET_NAME_LEN]; 
    float p1;                    // Pulse 1 / warm-up (ms) — 5–50, P1 ≤ P2
    float t;                     // Shared pause between pulses (ms), 20–150
    float p2;                    // Pulse 2 / main weld (ms) — 5–50, peak
    float p3;                    // Pulse 3 / forge (ms) — 5–50, P3 ≤ P2
    float p4;                    // Pulse 4 / temper (ms) — 5–50, P4 ≤ P3
} weld_preset_t;

// ============================================================================
// BLE Configuration
// ============================================================================
#define BLE_DEVICE_NAME      "MyWeld"
#define BLE_SERVICE_UUID     0x1234
#define BLE_CHAR_PARAMS_UUID 0x1235
#define BLE_CHAR_STATUS_UUID 0x1236
#define BLE_CHAR_CMD_UUID    0x1237

// ============================================================================
// PIN Authentication
// ============================================================================
#define PIN_DEFAULT          "1234"   // Factory default PIN (4 ASCII digits)
#define PIN_MAX_LEN          5        // 4 digits + null terminator
#define BLE_AUTH_MAX_ATTEMPTS 5       // Wrong PINs before lockout
#define BLE_AUTH_LOCKOUT_SEC  60      // Lockout duration (seconds)

// ============================================================================
// FreeRTOS Task Configuration
// ============================================================================
#define TASK_UI_STACK_SIZE       20480  // LVGL full-frame render needs ~16–20KB minimum
#define TASK_UI_PRIORITY         5
#define TASK_UI_CORE             0      // Core 0: UI rendering

#define TASK_WELDING_STACK_SIZE  4096
#define TASK_WELDING_PRIORITY    10     // Highest priority for pulse timing
#define TASK_WELDING_CORE        1      // Core 1: welding logic

#define TASK_ADC_STACK_SIZE      4096
#define TASK_ADC_PRIORITY        3
#define TASK_ADC_CORE            1      // Core 1: ADC sampling

#define TASK_AUDIO_STACK_SIZE    4096
#define TASK_AUDIO_PRIORITY      2
#define TASK_AUDIO_CORE          1      // Core 1: audio output

#define TASK_BLE_STACK_SIZE      4096
#define TASK_BLE_PRIORITY        1
#define TASK_BLE_CORE            0      // Core 0: BLE (shares with UI)

// ============================================================================
// Software Version
// ============================================================================
#define FW_VERSION_MAJOR     1
#define FW_VERSION_MINOR     0
#define FW_VERSION_PATCH     0
#define FW_VERSION_STRING    "1.0.0"
#define FW_BUILD_DATE        __DATE__
#define FW_BUILD_TIME        __TIME__

#endif // CONFIG_H
