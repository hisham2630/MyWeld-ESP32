#ifndef AUDIO_H
#define AUDIO_H

#include "config.h"

/**
 * I2S Audio Module
 * 
 * Uses the built-in I2S audio amplifier on the JC3248W535 board.
 * Generates sine wave tones for pleasant audio feedback.
 * Speaker connects to P6 connector (-SP, -VAT).
 * 
 * I2S Pins: LRCLK=GPIO2, BCLK=GPIO42, DOUT=GPIO41
 */

/**
 * Initialize I2S peripheral for audio output.
 */
void audio_init(void);

/**
 * Play a tone at a specific frequency for a given duration.
 * Non-blocking: queues the tone for the audio task.
 * 
 * @param freq_hz  Frequency in Hz
 * @param duration_ms Duration in milliseconds
 */
void audio_play_tone(uint16_t freq_hz, uint16_t duration_ms);

/**
 * Play startup melody (C5→E5→G5 arpeggio).
 * Called once during boot.
 */
void audio_play_startup(void);

/**
 * Short confirmation beep (880Hz, 50ms).
 * Used for parameter changes, button presses.
 */
void audio_play_beep(void);

/**
 * Weld pulse fired notification (1200Hz, 30ms).
 */
void audio_play_weld_fire(void);

/**
 * Caps fully charged chime (660Hz→880Hz ascending).
 */
void audio_play_ready(void);

/**
 * Error/warning alert (440Hz→220Hz descending).
 */
void audio_play_error(void);

/**
 * Contact detected blip (1000Hz, 20ms).
 * Used in AUTO mode when electrodes touch workpiece.
 */
void audio_play_contact(void);

/**
 * BLE client connected welcome chime (D5→A5, soft ascending 2-note).
 * Low volume, brief — plays when a phone connects for the first time
 * or after re-authentication. Respects the sound-on setting.
 */
void audio_play_ble_connect(void);

/**
 * Check if audio is currently muted (from settings).
 * @return true if sound is disabled
 */
bool audio_is_muted(void);

/**
 * Set audio mute state.
 * @param muted true to mute, false to unmute
 */
void audio_set_muted(bool muted);

#endif // AUDIO_H
