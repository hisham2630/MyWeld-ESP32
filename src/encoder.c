/**
 * Rotary Encoder Driver — KY-040 compatible
 *
 * Rotation: ISR-driven on S1 (CLK) falling edge.
 *           Reads S2 (DT) level to determine direction.
 *           Events queued via FreeRTOS xQueueSendFromISR.
 *
 * Button:   Polled in encoder_poll() with 50ms debounce.
 *           Short press (<500ms) and long press (≥500ms) detection.
 *
 * Safety:   Runs entirely on Core 0. Cannot affect welding (Core 1).
 *           Cannot trigger a weld — only IO14 (physical button) can fire.
 */

#include "encoder.h"
#include "config.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "ENCODER";

// ============================================================================
// Internal state
// ============================================================================

#define ENC_QUEUE_DEPTH 8

static QueueHandle_t s_enc_queue = NULL;

// ISR debounce — ignore edges within ENC_DEBOUNCE_MS of each other
static volatile int64_t s_last_isr_us = 0;

// Button state machine
static bool    s_key_down       = false;   // Physical key state (debounced)
static bool    s_key_raw_prev   = false;   // Previous raw reading
static int64_t s_key_change_ms  = 0;       // When raw state last changed
static int64_t s_key_press_ms   = 0;       // When debounced press started
static bool    s_long_sent      = false;   // Long-press event already emitted

// ============================================================================
// Rotation ISR
// ============================================================================

static void IRAM_ATTR enc_rotation_isr(void *arg)
{
    (void)arg;

    // Software debounce — ignore bouncy edges
    int64_t now_us = esp_timer_get_time();
    if ((now_us - s_last_isr_us) < (ENC_DEBOUNCE_MS * 1000)) {
        return;
    }
    s_last_isr_us = now_us;

    // On falling edge of S1 (CLK):
    //   S2 (DT) HIGH → clockwise
    //   S2 (DT) LOW  → counter-clockwise
    // (Swap CW/CCW here if your encoder is reversed)
    int dt = gpio_get_level(PIN_ENC_S2);
    encoder_event_t evt = dt ? ENC_EVENT_CW : ENC_EVENT_CCW;

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_enc_queue, &evt, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

// ============================================================================
// Public API
// ============================================================================

void encoder_init(void)
{
    // Create event queue
    s_enc_queue = xQueueCreate(ENC_QUEUE_DEPTH, sizeof(encoder_event_t));
    if (!s_enc_queue) {
        ESP_LOGE(TAG, "Failed to create encoder queue!");
        return;
    }

    // Configure S1 (CLK) — input with pull-up, falling edge interrupt
    gpio_config_t s1_conf = {
        .pin_bit_mask  = (1ULL << PIN_ENC_S1),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&s1_conf);

    // Configure S2 (DT) — input with pull-up, no interrupt
    gpio_config_t s2_conf = {
        .pin_bit_mask  = (1ULL << PIN_ENC_S2),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&s2_conf);

    // Configure KEY (SW) — input with pull-up, no interrupt (polled)
    gpio_config_t key_conf = {
        .pin_bit_mask  = (1ULL << PIN_ENC_KEY),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&key_conf);

    // Install GPIO ISR service (safe to call if already installed)
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(err));
        return;
    }

    // Attach rotation ISR to S1
    gpio_isr_handler_add(PIN_ENC_S1, enc_rotation_isr, NULL);

    ESP_LOGI(TAG, "Encoder initialized: S1=IO%d S2=IO%d KEY=IO%d",
             PIN_ENC_S1, PIN_ENC_S2, PIN_ENC_KEY);
}

bool encoder_poll(encoder_event_t *event)
{
    if (!s_enc_queue) return false;

    // 1) Check rotation queue first (ISR-produced events)
    if (xQueueReceive(s_enc_queue, event, 0) == pdTRUE) {
        return true;
    }

    // 2) Poll button with debounce state machine
    int64_t now_ms = esp_timer_get_time() / 1000;
    bool key_raw = (gpio_get_level(PIN_ENC_KEY) == 0);  // active LOW

    // Detect raw state change → start debounce timer
    if (key_raw != s_key_raw_prev) {
        s_key_change_ms = now_ms;
        s_key_raw_prev = key_raw;
    }

    // Apply debounce: only accept state if stable for ENC_KEY_DEBOUNCE_MS
    if ((now_ms - s_key_change_ms) >= ENC_KEY_DEBOUNCE_MS) {
        if (key_raw && !s_key_down) {
            // Debounced press detected
            s_key_down = true;
            s_key_press_ms = now_ms;
            s_long_sent = false;
        } else if (!key_raw && s_key_down) {
            // Debounced release
            s_key_down = false;
            if (!s_long_sent) {
                // Short press (released before long-press threshold)
                *event = ENC_EVENT_PRESS;
                return true;
            }
            // If long was already sent, release is a no-op
        }
    }

    // Long-press detection while held
    if (s_key_down && !s_long_sent) {
        if ((now_ms - s_key_press_ms) >= ENC_LONG_PRESS_MS) {
            s_long_sent = true;
            *event = ENC_EVENT_LONG_PRESS;
            return true;
        }
    }

    return false;
}
