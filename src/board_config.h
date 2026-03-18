/**
 * MyWeld ESP32-S3 — Board & Variant Configuration
 *
 * Central hardware abstraction header. All variant-specific decisions
 * (display type, audio type, pin remapping) are resolved HERE based
 * on build-time defines set in platformio.ini.
 *
 * Build defines expected from platformio.ini:
 *   -DDISPLAY_TYPE=<n>   (1=QSPI TFT, 2=Nextion, 3=LCD 20×4)
 *   -DAUDIO_TYPE=<n>     (1=I2S, 2=LEDC Buzzer)
 *   -DBOARD_VARIANT=<n>  (1=JC3248W535, 2=DevKit)
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// ============================================================================
// Display type constants
// ============================================================================
#define DISPLAY_QSPI_TFT     1  // JC3248W535 built-in 480×320 QSPI (AXS15231B + LVGL)
#define DISPLAY_NEXTION       2  // Nextion HMI via UART (display handles UI rendering)
#define DISPLAY_LCD_2004      3  // 20×4 character LCD via I2C (HD44780 + PCF8574)

// ============================================================================
// Audio type constants
// ============================================================================
#define AUDIO_I2S             1  // I2S DAC → amplifier → speaker (JC3248W535 built-in)
#define AUDIO_BUZZER          2  // Passive buzzer via LEDC PWM (any GPIO)

// ============================================================================
// Board variant constants
// ============================================================================
#define BOARD_JC3248W535      1  // Guition JC3248W535 all-in-one (display+touch+speaker)
#define BOARD_DEVKIT           2  // Generic ESP32-S3 N16R8 DevKit-C

// ============================================================================
// Defaults (if not set by platformio.ini — fallback to current board)
// ============================================================================
#ifndef DISPLAY_TYPE
#define DISPLAY_TYPE   DISPLAY_QSPI_TFT
#endif

#ifndef AUDIO_TYPE
#define AUDIO_TYPE     AUDIO_I2S
#endif

#ifndef BOARD_VARIANT
#define BOARD_VARIANT  BOARD_JC3248W535
#endif

// ============================================================================
// Feature flags (derived from variant selection)
// ============================================================================

// Does this variant use LVGL for graphical UI?
#if (DISPLAY_TYPE == DISPLAY_QSPI_TFT)
  #define HAS_LVGL           1
  #define HAS_TOUCH          1
  #define HAS_BACKLIGHT      1
#else
  #define HAS_LVGL           0
  #define HAS_TOUCH          0
  #define HAS_BACKLIGHT      0
#endif

// Does this variant use Nextion HMI?
#if (DISPLAY_TYPE == DISPLAY_NEXTION)
  #define HAS_NEXTION        1
#else
  #define HAS_NEXTION        0
#endif

// Does this variant use a character LCD?
#if (DISPLAY_TYPE == DISPLAY_LCD_2004)
  #define HAS_CHAR_LCD       1
#else
  #define HAS_CHAR_LCD       0
#endif

// Audio backend
#if (AUDIO_TYPE == AUDIO_I2S)
  #define HAS_I2S_AUDIO      1
  #define HAS_BUZZER         0
#elif (AUDIO_TYPE == AUDIO_BUZZER)
  #define HAS_I2S_AUDIO      0
  #define HAS_BUZZER         1
#else
  #error "AUDIO_TYPE must be AUDIO_I2S (1) or AUDIO_BUZZER (2)"
#endif

// ============================================================================
// Pin Definitions — DevKit variant overrides
// ============================================================================
// For the JC3248W535, pins are fixed by the board design (see config.h).
// For the generic DevKit, we remap display/audio pins to user-accessible GPIOs.

#if (BOARD_VARIANT == BOARD_DEVKIT)

  // GPIO_NUM_* constants used below come from ESP-IDF's gpio_types.h
  #include "driver/gpio.h"

  // --- Nextion UART pins (when DISPLAY_TYPE == DISPLAY_NEXTION) ---
  #define PIN_NEXTION_TX     GPIO_NUM_17  // DevKit TX → Nextion RX
  #define PIN_NEXTION_RX     GPIO_NUM_18  // DevKit RX → Nextion TX
  #define NEXTION_UART_NUM   UART_NUM_1
  #define NEXTION_BAUD_RATE  115200

  // --- I2C LCD pins (when DISPLAY_TYPE == DISPLAY_LCD_2004) ---
  #define PIN_LCD_SDA        GPIO_NUM_8   // I2C SDA
  #define PIN_LCD_SCL        GPIO_NUM_9   // I2C SCL
  #define LCD_I2C_ADDR       0x27         // PCF8574 default address
  #define LCD_I2C_NUM        I2C_NUM_0

  // --- I2S Audio pins (MAX98357 amplifier module) ---
  // Override JC3248W535 defaults (GPIO 41/42 conflict with JTAG on WROOM)
  #define PIN_I2S_BCLK       GPIO_NUM_12  // I2S Bit Clock  → MAX98357 BCLK
  #define PIN_I2S_LRCLK      GPIO_NUM_13  // I2S Word Select → MAX98357 LRC
  #define PIN_I2S_DOUT       GPIO_NUM_11  // I2S Data Out    → MAX98357 DIN

  // --- Encoder pins (may differ on DevKit wiring) ---
  #ifndef PIN_ENC_S1
    #define PIN_ENC_S1       GPIO_NUM_15  // Same as JC3248W535 default
  #endif
  #ifndef PIN_ENC_S2
    #define PIN_ENC_S2       GPIO_NUM_16  // Remapped — GPIO17 is Nextion TX
  #endif
  #ifndef PIN_ENC_KEY
    #define PIN_ENC_KEY      GPIO_NUM_21  // Remapped — GPIO18 is Nextion RX
  #endif

#endif // BOARD_DEVKIT

// ============================================================================
// Board Name String (for boot banner)
// ============================================================================
#if (BOARD_VARIANT == BOARD_JC3248W535)
  #define BOARD_NAME_STRING   "JC3248W535 (Guition)"
#elif (BOARD_VARIANT == BOARD_DEVKIT)
  #if (DISPLAY_TYPE == DISPLAY_NEXTION)
    #define BOARD_NAME_STRING "ESP32-S3 DevKit + Nextion"
  #elif (DISPLAY_TYPE == DISPLAY_LCD_2004)
    #define BOARD_NAME_STRING "ESP32-S3 DevKit + LCD 20x4"
  #else
    #define BOARD_NAME_STRING "ESP32-S3 DevKit"
  #endif
#else
  #define BOARD_NAME_STRING   "Unknown Board"
#endif

// ============================================================================
// Validation
// ============================================================================
#if (DISPLAY_TYPE == DISPLAY_QSPI_TFT) && (BOARD_VARIANT != BOARD_JC3248W535)
  #error "QSPI TFT display is only available on the JC3248W535 board"
#endif

// I2S audio is now supported on both JC3248W535 (built-in) and DevKit (MAX98357)

#endif // BOARD_CONFIG_H
