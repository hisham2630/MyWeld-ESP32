/**
 * MyWeld ESP32-S3 — Passive Buzzer Driver (LEDC PWM)
 *
 * Drives a passive buzzer via LEDC PWM for audio feedback.
 * This is the alternative audio backend for boards without a built-in
 * I2S amplifier (e.g., generic ESP32-S3 DevKit).
 *
 * The API mirrors audio.c so that calling code (welding.c, ble_serial.c)
 * doesn't need to know which backend is active.
 *
 * Wiring: GPIO → passive buzzer → GND
 */

#include "board_config.h"

#if HAS_BUZZER

#include "buzzer.h"
#include "config.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdbool.h>

static const char *TAG = "buzzer";

// ---------- State ----------
static bool s_muted = false;
static uint8_t s_volume = 80;           // 0–100 (maps to LEDC duty cycle)
static SemaphoreHandle_t s_mutex = NULL;

// ============================================================================
// Low-level LEDC helpers
// ============================================================================

static void buzzer_ledc_start(uint16_t freq_hz, uint8_t duty_pct) {
    if (freq_hz == 0 || duty_pct == 0) return;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = BUZZER_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,    // 0–1023
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    uint32_t duty = (1023 * duty_pct) / 100;
    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BUZZER_LEDC_CHANNEL,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .gpio_num   = PIN_BUZZER,
        .duty       = duty,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch_cfg);
}

static void buzzer_ledc_stop(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL);
    ledc_stop(LEDC_LOW_SPEED_MODE, BUZZER_LEDC_CHANNEL, 0);
}

// ============================================================================
// Public API (matches audio.h interface)
// ============================================================================

void buzzer_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Passive buzzer initialized on GPIO%d", PIN_BUZZER);
}

void buzzer_play_tone(uint16_t freq_hz, uint16_t duration_ms) {
    if (s_muted || freq_hz == 0 || duration_ms == 0) return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    buzzer_ledc_start(freq_hz, s_volume / 2);   // Buzzer: cap at 50% duty max
    xSemaphoreGive(s_mutex);

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        buzzer_ledc_stop();
        xSemaphoreGive(s_mutex);
    }
}

void buzzer_stop(void) {
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        buzzer_ledc_stop();
        xSemaphoreGive(s_mutex);
    }
}

// ============================================================================
// audio_* wrappers — same names as audio.c so the rest of the
// firmware doesn't need #ifdefs everywhere.
// ============================================================================

void audio_init(void) {
    buzzer_init();
}

void audio_play_tone(uint16_t freq_hz, uint16_t duration_ms) {
    buzzer_play_tone(freq_hz, duration_ms);
}

void audio_play_startup(void) {
    buzzer_play_tone(TONE_STARTUP_C, 80);
    vTaskDelay(pdMS_TO_TICKS(30));
    buzzer_play_tone(TONE_STARTUP_E, 80);
    vTaskDelay(pdMS_TO_TICKS(30));
    buzzer_play_tone(TONE_STARTUP_G, 120);
}

void audio_play_beep(void) {
    buzzer_play_tone(TONE_BEEP, 50);
}

void audio_play_weld_fire(void) {
    buzzer_play_tone(TONE_WELD_FIRE, 30);
}

void audio_play_ready(void) {
    buzzer_play_tone(TONE_READY_LOW, 80);
    vTaskDelay(pdMS_TO_TICKS(20));
    buzzer_play_tone(TONE_READY_HIGH, 100);
}

void audio_play_error(void) {
    buzzer_play_tone(TONE_ERROR_HIGH, 100);
    vTaskDelay(pdMS_TO_TICKS(30));
    buzzer_play_tone(TONE_ERROR_LOW, 150);
}

void audio_play_contact(void) {
    buzzer_play_tone(TONE_CONTACT, 20);
}

void audio_play_ble_connect(void) {
    buzzer_play_tone(TONE_BLE_NOTE_1, 60);
    vTaskDelay(pdMS_TO_TICKS(20));
    buzzer_play_tone(TONE_BLE_NOTE_2, 80);
}

bool audio_is_muted(void) {
    return s_muted;
}

void audio_set_muted(bool muted) {
    s_muted = muted;
    if (muted) buzzer_stop();
}

void audio_set_volume(uint8_t volume) {
    s_volume = (volume > 100) ? 100 : volume;
}

uint8_t audio_get_volume(void) {
    return s_volume;
}

#endif // HAS_BUZZER
