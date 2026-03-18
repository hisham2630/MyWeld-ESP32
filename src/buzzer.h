/**
 * MyWeld ESP32-S3 — Passive Buzzer Driver (LEDC PWM)
 *
 * Alternative audio backend for boards without I2S amplifier.
 * Uses a passive buzzer connected to a GPIO pin, driven by LEDC PWM.
 * Only compiled when AUDIO_TYPE == AUDIO_BUZZER.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "board_config.h"

#if HAS_BUZZER

#include <stdbool.h>
#include <stdint.h>

void buzzer_init(void);
void buzzer_play_tone(uint16_t freq_hz, uint16_t duration_ms);
void buzzer_stop(void);

#endif // HAS_BUZZER
#endif // BUZZER_H
