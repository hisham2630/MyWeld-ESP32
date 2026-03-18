/**
 * MyWeld ESP32-S3 — Audio Hardware Abstraction Layer
 *
 * Provides a unified audio interface regardless of the backend.
 * - I2S backend: sine wave tones via I2S DAC → amplifier → speaker
 * - Buzzer backend: square wave tones via LEDC PWM → passive buzzer
 *
 * Both backends expose the same API. The correct implementation is
 * linked at compile time based on AUDIO_TYPE.
 */

#ifndef AUDIO_HAL_H
#define AUDIO_HAL_H

#include "board_config.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Unified Audio Interface
// ============================================================================
// These functions are implemented by EITHER audio.c (I2S) or buzzer.c (LEDC).
// The function names match the existing audio.h API so minimal refactoring
// is needed in calling code.

/**
 * Initialize the audio subsystem.
 */
void audio_init(void);

/**
 * Play a tone at a specific frequency.
 * @param freq_hz    Frequency in Hz
 * @param duration_ms Duration in milliseconds
 */
void audio_play_tone(uint16_t freq_hz, uint16_t duration_ms);

// --- Convenience wrappers (same API as existing audio.h) ---
void audio_play_startup(void);
void audio_play_beep(void);
void audio_play_weld_fire(void);
void audio_play_ready(void);
void audio_play_error(void);
void audio_play_contact(void);
void audio_play_ble_connect(void);

bool audio_is_muted(void);
void audio_set_muted(bool muted);
void audio_set_volume(uint8_t volume);
uint8_t audio_get_volume(void);

#endif // AUDIO_HAL_H
