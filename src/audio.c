/**
 * I2S Audio Module — Sine Wave Tone Generator
 * 
 * Uses ESP32-S3 I2S peripheral → built-in amplifier → P6 speaker.
 * Generates clean sine wave tones for pleasant audio feedback.
 */

#include "audio.h"
#include "settings.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>
#include <string.h>

static const char *TAG = "Audio";

// I2S handle
static i2s_chan_handle_t s_tx_handle = NULL;
static bool s_muted = false;

// Audio command queue
typedef struct {
    uint16_t freq_hz;
    uint16_t duration_ms;
    bool     quiet;       // true = 25% amplitude (soft welcome chime)
} audio_cmd_t;

#define AUDIO_QUEUE_SIZE 8
static QueueHandle_t s_audio_queue = NULL;

// Pre-computed sine table (256 entries, 16-bit signed)
#define SINE_TABLE_SIZE 256
static int16_t sine_table[SINE_TABLE_SIZE];

static void generate_sine_table(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        sine_table[i] = (int16_t)(sinf(2.0f * M_PI * i / SINE_TABLE_SIZE) * 16000);
    }
}

/**
 * Generate and play a sine wave tone.
 * This is blocking — called from the audio task.
 * @param amplitude Peak sample value (0–32767). Use 16000 for normal, 4000 for quiet.
 */
static void play_sine_tone(uint16_t freq_hz, uint16_t duration_ms, int16_t amplitude)
{
    if (!s_tx_handle || freq_hz == 0 || duration_ms == 0) return;

    const int sample_rate = AUDIO_SAMPLE_RATE;
    const int total_samples = (sample_rate * duration_ms) / 1000;
    const float phase_increment = (float)freq_hz * SINE_TABLE_SIZE / sample_rate;

    #define CHUNK_SAMPLES 256
    int16_t buffer[CHUNK_SAMPLES * 2]; // stereo interleaved (L, R)
    float phase = 0.0f;
    int samples_written = 0;
    size_t bytes_written;

    while (samples_written < total_samples) {
        int chunk = total_samples - samples_written;
        if (chunk > CHUNK_SAMPLES) chunk = CHUNK_SAMPLES;

        for (int i = 0; i < chunk; i++) {
            int idx = (int)phase & (SINE_TABLE_SIZE - 1);
            // Scale stored sine table (amplitude 16000) to target amplitude
            int16_t sample = (int16_t)((int32_t)sine_table[idx] * amplitude / 16000);
            buffer[i * 2]     = sample;
            buffer[i * 2 + 1] = sample;
            phase += phase_increment;
            if (phase >= SINE_TABLE_SIZE) phase -= SINE_TABLE_SIZE;
        }

        i2s_channel_write(s_tx_handle, buffer, chunk * 4, &bytes_written, pdMS_TO_TICKS(100));
        samples_written += chunk;
    }

    // Short silence gap to prevent click
    memset(buffer, 0, sizeof(buffer));
    i2s_channel_write(s_tx_handle, buffer, CHUNK_SAMPLES * 4, &bytes_written, pdMS_TO_TICKS(50));
}

/**
 * Audio processing task (runs on Core 1).
 * Dequeues tone commands and plays them.
 */
static void audio_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Audio task started on Core %d", xPortGetCoreID());
    audio_cmd_t cmd;

    while (1) {
        if (xQueueReceive(s_audio_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd.freq_hz == 0) {
                // Silence/pause command — just wait
                vTaskDelay(pdMS_TO_TICKS(cmd.duration_ms));
            } else if (!s_muted) {
                int16_t amp = cmd.quiet ? 10000 : 16000;  // ~62% vs full amplitude
                play_sine_tone(cmd.freq_hz, cmd.duration_ms, amp);
            }
        }
    }
}

static void queue_tone(uint16_t freq, uint16_t duration)
{
    if (!s_audio_queue || s_muted) return;
    audio_cmd_t cmd = { .freq_hz = freq, .duration_ms = duration, .quiet = false };
    xQueueSend(s_audio_queue, &cmd, 0);
}

static void queue_tone_quiet(uint16_t freq, uint16_t duration)
{
    if (!s_audio_queue || s_muted) return;
    audio_cmd_t cmd = { .freq_hz = freq, .duration_ms = duration, .quiet = true };
    xQueueSend(s_audio_queue, &cmd, 0);
}

// ============================================================================
// Public API
// ============================================================================

void audio_init(void)
{
    generate_sine_table();

    // Create command queue
    s_audio_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_cmd_t));

    // Configure I2S channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)PIN_I2S_BCLK,
            .ws   = (gpio_num_t)PIN_I2S_LRCLK,
            .dout = (gpio_num_t)PIN_I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_handle));

    s_muted = !g_settings.sound_on;

    // Start audio task on Core 1
    xTaskCreatePinnedToCore(audio_task, "audio_task",
                            TASK_AUDIO_STACK_SIZE, NULL,
                            TASK_AUDIO_PRIORITY, NULL,
                            TASK_AUDIO_CORE);

    ESP_LOGI(TAG, "I2S audio initialized (LRCLK=%d, BCLK=%d, DOUT=%d), muted=%d",
             PIN_I2S_LRCLK, PIN_I2S_BCLK, PIN_I2S_DOUT, s_muted);
}

void audio_play_tone(uint16_t freq_hz, uint16_t duration_ms)
{
    queue_tone(freq_hz, duration_ms);
}

void audio_play_startup(void)
{
    if (s_muted) return;
    queue_tone(TONE_STARTUP_C, 120); // C5
    queue_tone(TONE_STARTUP_E, 120); // E5
    queue_tone(TONE_STARTUP_G, 200); // G5 (longer)
}

void audio_play_beep(void)
{
    queue_tone(TONE_BEEP, 50);
}

void audio_play_weld_fire(void)
{
    queue_tone(TONE_WELD_FIRE, 30);
}

void audio_play_ready(void)
{
    if (s_muted) return;
    queue_tone(TONE_READY_LOW, 100);
    queue_tone(TONE_READY_HIGH, 150);
}

void audio_play_error(void)
{
    if (s_muted) return;
    queue_tone(TONE_ERROR_HIGH, 150);
    queue_tone(TONE_ERROR_LOW, 250);
}

void audio_play_contact(void)
{
    queue_tone(TONE_CONTACT, 20);
}

void audio_play_ble_connect(void)
{
    if (s_muted) return;
    // Soft 2-note ascending welcome chime at 25% amplitude
    // D5 (587Hz, 80ms) → brief gap → A5 (880Hz, 120ms)
    // Similar character to JK BMS welcome tone: warm, brief, low volume
    queue_tone_quiet(TONE_BLE_NOTE_1, 80);
    queue_tone_quiet(0, 30);          // 30ms silence gap (0Hz = silence)
    queue_tone_quiet(TONE_BLE_NOTE_2, 120);
    ESP_LOGD("Audio", "BLE connect chime queued");
}

bool audio_is_muted(void)
{
    return s_muted;
}

void audio_set_muted(bool muted)
{
    s_muted = muted;
    ESP_LOGI(TAG, "Audio %s", muted ? "muted" : "unmuted");
}
