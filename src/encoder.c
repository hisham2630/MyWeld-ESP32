/**
 * Rotary Encoder Driver — KY-040 compatible (Quadrature State Machine)
 *
 * Rotation: ISR fires on ANY edge of BOTH S1 (CLK) and S2 (DT).
 *           A Gray-code state machine tracks valid transitions only.
 *           One CW/CCW event is emitted per full detent (4 micro-steps).
 *           Contact bounce produces no false events because invalid
 *           state transitions (e.g. 11→00) are silently discarded.
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

// ----------------------------------------------------------------------------
// Quadrature state machine
//
// State = (S1_level | (S2_level << 1)), giving 4 possible values: 0–3.
// At rest (both pulled HIGH) the state is 3.
//
// Valid Gray-code sequences (KY-040 with mechanical detents):
//   CW  detent: 3 → 2 → 0 → 1 → 3  (4 micro-steps)
//   CCW detent: 3 → 1 → 0 → 2 → 3  (4 micro-steps)
//
// The lookup table is indexed by (old_state << 2 | new_state):
//   +1 = valid CW  micro-step
//   -1 = valid CCW micro-step
//    0 = no change  OR  invalid skip (bounce/noise) → ignored
//
// The accumulator counts valid micro-steps. Only when it reaches
// ±ENC_STEPS_PER_DETENT (typically 4) is a single CW/CCW event emitted.
// This makes the decoder inherently immune to contact bounce because:
//   - Bounce on one pin oscillates between two adjacent states (e.g. 3↔2),
//     producing +1 then -1, so the accumulator stays near zero.
//   - Electrical noise that skips a state (e.g. 3→0) produces 0 (invalid)
//     from the lookup table and is silently discarded.
// ----------------------------------------------------------------------------

static volatile uint8_t s_enc_state = 3;   // Rest: both pins HIGH (pull-up)
static volatile int8_t  s_enc_accum = 0;   // Micro-step accumulator

// Must live in DRAM for ISR access (not flash)
static const DRAM_ATTR int8_t s_quad_table[16] = {
//         new state →  0   1   2   3
/* old=0 (S1=0,S2=0) */ 0, +1, -1,  0,
/* old=1 (S1=1,S2=0) */-1,  0,  0, +1,
/* old=2 (S1=0,S2=1) */+1,  0,  0, -1,
/* old=3 (S1=1,S2=1) */ 0, -1, +1,  0,
};

// Button state machine
static volatile bool s_key_isr_flag = false;  // Set by ISR on falling edge
static bool    s_key_down       = false;   // Physical key state (debounced)
static int64_t s_key_press_ms   = 0;       // When debounced press started
static bool    s_long_sent      = false;   // Long-press event already emitted
static int64_t s_key_release_ms = 0;       // When release was first detected
static bool    s_key_releasing  = false;   // Release debounce in progress

// ============================================================================
// ISR handlers
// ============================================================================

// Quadrature ISR (shared handler for S1 and S2, fires on ANY edge)
static void IRAM_ATTR enc_quadrature_isr(void *arg)
{
    (void)arg;

    // Read current 2-bit state
    uint8_t new_state = (uint8_t)(gpio_get_level(PIN_ENC_S1)
                                | (gpio_get_level(PIN_ENC_S2) << 1));

    // Lookup direction from old→new transition
    uint8_t idx = (uint8_t)((s_enc_state << 2) | new_state);
    int8_t dir = s_quad_table[idx];

    s_enc_state = new_state;

    if (dir == 0) return;  // Invalid or no-change → ignore

    s_enc_accum += dir;

    // Full detent reached?
    if (s_enc_accum >= ENC_STEPS_PER_DETENT) {
        s_enc_accum = 0;
        encoder_event_t evt = ENC_EVENT_CW;
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(s_enc_queue, &evt, &woken);
        if (woken) portYIELD_FROM_ISR();
    } else if (s_enc_accum <= -ENC_STEPS_PER_DETENT) {
        s_enc_accum = 0;
        encoder_event_t evt = ENC_EVENT_CCW;
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(s_enc_queue, &evt, &woken);
        if (woken) portYIELD_FROM_ISR();
    }
}

// Button ISR: fires on falling edge (button press, active LOW)
static void IRAM_ATTR enc_key_isr(void *arg)
{
    (void)arg;
    s_key_isr_flag = true;  // Latch — cleared by encoder_poll()
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

    // Configure S1 (CLK) — input with pull-up, ANY edge interrupt
    gpio_config_t s1_conf = {
        .pin_bit_mask  = (1ULL << PIN_ENC_S1),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&s1_conf);

    // Configure S2 (DT) — input with pull-up, ANY edge interrupt
    gpio_config_t s2_conf = {
        .pin_bit_mask  = (1ULL << PIN_ENC_S2),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&s2_conf);

    // Configure KEY (SW) — input with pull-up, FALLING edge interrupt (press)
    gpio_config_t key_conf = {
        .pin_bit_mask  = (1ULL << PIN_ENC_KEY),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_NEGEDGE,  // Interrupt on press (active LOW)
    };
    gpio_config(&key_conf);

    // Install GPIO ISR service (safe to call if already installed)
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(err));
        return;
    }

    // Attach quadrature ISR to BOTH encoder pins
    gpio_isr_handler_add(PIN_ENC_S1, enc_quadrature_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_S2, enc_quadrature_isr, NULL);

    // Attach button ISR
    gpio_isr_handler_add(PIN_ENC_KEY, enc_key_isr, NULL);

    // Seed initial state from actual pin levels
    s_enc_state = (uint8_t)(gpio_get_level(PIN_ENC_S1)
                          | (gpio_get_level(PIN_ENC_S2) << 1));

    ESP_LOGI(TAG, "Encoder initialized (quadrature+key ISR): S1=IO%d S2=IO%d KEY=IO%d",
             PIN_ENC_S1, PIN_ENC_S2, PIN_ENC_KEY);
}

bool encoder_poll(encoder_event_t *event)
{
    if (!s_enc_queue) return false;

    // 1) Check rotation queue first (ISR-produced events)
    if (xQueueReceive(s_enc_queue, event, 0) == pdTRUE) {
        return true;
    }

    // 2) Button state machine — ISR-assisted press detection
    int64_t now_ms = esp_timer_get_time() / 1000;
    bool key_raw = (gpio_get_level(PIN_ENC_KEY) == 0);  // active LOW

    // ISR flagged a press — accept immediately (no debounce needed for press;
    // the ISR fires on the clean falling edge and bounce only produces
    // additional falling edges which just re-set the same flag).
    if (s_key_isr_flag && !s_key_down) {
        s_key_isr_flag = false;
        s_key_down = true;
        s_key_press_ms = now_ms;
        s_long_sent = false;
        s_key_releasing = false;
    } else if (s_key_isr_flag) {
        // Already pressed — clear stale ISR flag
        s_key_isr_flag = false;
    }

    // Release detection with debounce (prevents contact bounce on release)
    if (s_key_down && !key_raw) {
        if (!s_key_releasing) {
            s_key_releasing = true;
            s_key_release_ms = now_ms;
        } else if ((now_ms - s_key_release_ms) >= ENC_KEY_DEBOUNCE_MS) {
            // Debounced release confirmed
            s_key_down = false;
            s_key_releasing = false;
            if (!s_long_sent) {
                *event = ENC_EVENT_PRESS;
                return true;
            }
            // If long was already sent, release is a no-op
        }
    } else if (s_key_down && key_raw) {
        // Button still held — cancel any release debounce (was just bounce)
        s_key_releasing = false;
    }

    // Long-press detection while held
    if (s_key_down && !s_long_sent && !s_key_releasing) {
        if ((now_ms - s_key_press_ms) >= ENC_LONG_PRESS_MS) {
            s_long_sent = true;
            *event = ENC_EVENT_LONG_PRESS;
            return true;
        }
    }

    return false;
}

