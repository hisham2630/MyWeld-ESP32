#ifndef CONFIG_H
#define CONFIG_H

/**
 * MyWeld ESP32-S3 — Hardware Configuration
 * Board: JC3248W535 (Guition ESP32-S3, AXS15231B display)
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
#define PIN_PROTECTION  GPIO_NUM_6   // 13.5V gate drive rail — ADC1_CH5 (P2 pin 2)
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

// --- Spare GPIOs (available for future expansion) ---
#define PIN_SPARE_1     GPIO_NUM_15  // P2 pin 4
#define PIN_SPARE_2     GPIO_NUM_9   // P2 pin 7 (ADC1_CH8)
#define PIN_SPARE_3     GPIO_NUM_17  // P3/P4
#define PIN_SPARE_4     GPIO_NUM_18  // P3/P4

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

// Voltage divider multipliers (calculate real voltage from ADC reading)
#define SUPERCAP_V_MULT      (25.0f / 15.0f)   // 10k+15k divider: Vreal = Vadc × (R1+R2)/R2
#define PROTECTION_V_MULT    (62.0f / 15.0f)    // 47k+15k divider

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
// Pulse Parameters
// ============================================================================
#define PULSE_MIN_MS         0.0f     // Minimum pulse duration (0 = OFF for P2)
#define PULSE_MAX_MS         50.0f    // Maximum pulse duration
#define PULSE_STEP_MS        0.5f     // Step size for touch +/- buttons
#define PULSE_DEFAULT_P1     5.0f     // Factory default P1 (ms)
#define PULSE_DEFAULT_T      8.0f     // Factory default T (ms)
#define PULSE_DEFAULT_P2     8.0f     // Factory default P2 (ms)

// AUTO mode delay (contact-to-fire)
#define S_VALUE_MIN          0.3f     // Minimum contact delay (seconds)
#define S_VALUE_MAX          2.0f     // Maximum contact delay (seconds)
#define S_VALUE_STEP         0.1f     // Step size (seconds)
#define S_VALUE_DEFAULT      0.5f     // Factory default (seconds)

// ============================================================================
// Timing Constants
// ============================================================================
#define DEBOUNCE_MS          5        // Button debounce delay
#define LOW_V_CONFIRM_MS     1500     // Confirm low voltage before blocking
#define PROTECT_CONFIRM_MS   1500     // Confirm protection fault before blocking
#define CHARGER_SETTLE_US    500      // Settle time before/after pulse (microseconds)
#define NVS_SAVE_DEBOUNCE_MS 2000     // Debounce NVS writes (prevent flash wear)
#define ADC_SAMPLE_INTERVAL  500      // ADC sampling interval (ms) for voltage graph

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
#define MAX_PRESETS           10
#define PRESET_NAME_LEN      20

// Preset data structure
typedef struct {
    char name[PRESET_NAME_LEN]; 
    float p1;                    // Pulse 1 duration (ms)
    float t;                     // Pause duration (ms)
    float p2;                    // Pulse 2 duration (ms), 0 = single pulse
    float s_value;               // AUTO mode contact delay (seconds)
    bool  auto_mode;             // true = AUTO, false = MAN
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
