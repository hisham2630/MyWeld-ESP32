/**
 * MyWeld ESP32-S3 — Status LED Driver (WS2812 NeoPixel)
 *
 * Pattern engine with smooth animations for system events.
 * Uses ESP-IDF RMT peripheral (new driver, ESP-IDF 5.x).
 *
 * Architecture:
 *   - RMT TX channel drives a single WS2812 on PIN_RGB_LED
 *   - FreeRTOS queue receives led_event_t from any task/core
 *   - Animation loop runs at 50Hz (20ms ticks) for smooth transitions
 *   - Base states are persistent; overlay events play once and return
 *
 * Only compiled when HAS_RGB_LED == 1.
 */

#include "board_config.h"

#if HAS_RGB_LED

#include "status_led.h"
#include "config.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "StatusLED";

// ============================================================================
// RMT Configuration
// ============================================================================
#define RMT_RESOLUTION_HZ  10000000   // 10 MHz → 100 ns per tick
#define LED_UPDATE_MS       20        // 50 Hz animation rate
#define LED_QUEUE_DEPTH     8

// WS2812 timing (ticks at 10 MHz):
//   bit-0: 0.4µs high + 0.8µs low  →  4 ticks + 8 ticks
//   bit-1: 0.8µs high + 0.4µs low  →  8 ticks + 4 ticks

static rmt_channel_handle_t  s_led_chan    = NULL;
static rmt_encoder_handle_t  s_led_encoder = NULL;
static QueueHandle_t         s_led_queue   = NULL;
static volatile uint8_t      s_ota_percent = 0;

// ============================================================================
// WS2812 Custom RMT Encoder
// ============================================================================
// Combines a bytes encoder (pixel data) + copy encoder (reset signal).

typedef struct {
    rmt_encoder_t      base;
    rmt_encoder_t     *bytes_encoder;
    rmt_encoder_t     *copy_encoder;
    int                state;
    rmt_symbol_word_t  reset_code;
} ws2812_encoder_t;

static size_t ws2812_encode(rmt_encoder_t *encoder,
                            rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size,
                            rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encode_state_t session_state = 0;
    rmt_encode_state_t state = 0;
    size_t encoded = 0;

    switch (enc->state) {
    case 0: // Encode pixel data (GRB bytes)
        encoded += enc->bytes_encoder->encode(
            enc->bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = 1; // Move to reset phase
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto done;
        }
        // fall through
    case 1: // Encode reset signal (>280µs low)
        encoded += enc->copy_encoder->encode(
            enc->copy_encoder, channel,
            &enc->reset_code, sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = 0; // Ready for next frame
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto done;
        }
    }
done:
    *ret_state = state;
    return encoded;
}

static esp_err_t ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(enc->bytes_encoder);
    rmt_encoder_reset(enc->copy_encoder);
    enc->state = 0;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_del(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(enc->bytes_encoder);
    rmt_del_encoder(enc->copy_encoder);
    free(enc);
    return ESP_OK;
}

static esp_err_t ws2812_encoder_create(rmt_encoder_handle_t *ret_encoder)
{
    ws2812_encoder_t *enc = calloc(1, sizeof(ws2812_encoder_t));
    if (!enc) return ESP_ERR_NO_MEM;

    enc->base.encode = ws2812_encode;
    enc->base.reset  = ws2812_encoder_reset;
    enc->base.del    = ws2812_encoder_del;

    // Bytes encoder: maps each bit to WS2812 high/low timing
    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = { .level0 = 1, .duration0 = 4,    // 0.4µs high
                   .level1 = 0, .duration1 = 8 },  // 0.8µs low
        .bit1 = { .level0 = 1, .duration0 = 8,    // 0.8µs high
                   .level1 = 0, .duration1 = 4 },  // 0.4µs low
        .flags.msb_first = 1,
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes_cfg, &enc->bytes_encoder);
    if (err != ESP_OK) { free(enc); return err; }

    // Copy encoder: outputs the reset symbol verbatim
    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &enc->copy_encoder);
    if (err != ESP_OK) {
        rmt_del_encoder(enc->bytes_encoder);
        free(enc);
        return err;
    }

    // Reset signal: 280µs low (2800 ticks at 10MHz)
    enc->reset_code = (rmt_symbol_word_t){
        .level0 = 0, .duration0 = 2800,
        .level1 = 0, .duration1 = 2800,
    };

    *ret_encoder = &enc->base;
    return ESP_OK;
}

// ============================================================================
// Low-Level LED Control
// ============================================================================

static void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    // WS2812 expects GRB byte order
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    rmt_transmit(s_led_chan, s_led_encoder, grb, sizeof(grb), &tx_cfg);
    rmt_tx_wait_all_done(s_led_chan, pdMS_TO_TICKS(50));
}

// ============================================================================
// Color Helpers
// ============================================================================

/**
 * HSV to RGB conversion.
 * @param h Hue 0–359
 * @param s Saturation 0–255
 * @param v Value (brightness) 0–255
 */
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s == 0) {
        *r = *g = *b = v;
        return;
    }
    uint8_t region = h / 60;
    uint16_t remainder = (h - (region * 60)) * 255 / 60;

    uint8_t p = (uint8_t)((uint16_t)v * (255 - s) / 255);
    uint8_t q = (uint8_t)((uint16_t)v * (255 - ((uint16_t)s * remainder / 255)) / 255);
    uint8_t t = (uint8_t)((uint16_t)v * (255 - ((uint16_t)s * (255 - remainder) / 255)) / 255);

    switch (region) {
    case 0:  *r = v; *g = t; *b = p; break;
    case 1:  *r = q; *g = v; *b = p; break;
    case 2:  *r = p; *g = v; *b = t; break;
    case 3:  *r = p; *g = q; *b = v; break;
    case 4:  *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}

/**
 * Breathing wave — smooth sine-like intensity oscillation.
 * Uses quadratic approximation: (triangle)² / 255 for a smooth curve.
 *
 * @param ms     Current elapsed time in milliseconds
 * @param period Breathing cycle period in milliseconds
 * @param lo     Minimum brightness (0–255)
 * @param hi     Maximum brightness (0–255)
 * @return       Brightness value between lo and hi
 */
static uint8_t breathing(uint32_t ms, uint32_t period, uint8_t lo, uint8_t hi)
{
    uint32_t phase = (ms % period) * 512 / period; // 0–511

    // Triangle wave: 0→255→0 over one period
    uint16_t tri;
    if (phase < 256) {
        tri = phase;
    } else {
        tri = 511 - phase;
    }

    // Quadratic gamma for natural breathing feel
    uint16_t gamma = (tri * tri) >> 8; // 0–255

    return (uint8_t)(lo + ((uint16_t)(hi - lo) * gamma / 255));
}

/**
 * Linear interpolation between two values.
 */
static uint8_t lerp8(uint8_t a, uint8_t b, uint8_t t) // t: 0–255
{
    return (uint8_t)(a + ((int16_t)(b - a) * t / 255));
}

// ============================================================================
// Pattern Engine
// ============================================================================

// Overlay durations (ms)
#define DUR_STARTUP          3000
#define DUR_BLE_CONNECT      1200  // 3 blinks × (200 on + 200 off)
#define DUR_BLE_DISCONNECT   800
#define DUR_WELD_FIRE        800
#define DUR_CONTACT_DETECT   400
#define DUR_OTA_COMPLETE     1500

// Internal "no overlay" sentinel
#define OVERLAY_NONE         0xFF

typedef struct {
    led_event_t  base_state;       // Current persistent state
    uint8_t      overlay;          // Current transient overlay (led_event_t or OVERLAY_NONE)
    uint32_t     overlay_start_ms; // Tick when overlay started
    uint32_t     boot_ms;          // Tick at init (for absolute time ref)
} led_engine_t;

static led_engine_t s_engine;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/**
 * Render one frame of the current overlay animation.
 * Returns true if the overlay is still active, false if it has finished.
 */
static bool render_overlay(uint32_t elapsed_ms, uint8_t *r, uint8_t *g, uint8_t *b)
{
    switch (s_engine.overlay) {

    // ── Startup: Rainbow hue sweep + brightness ramp ──
    case LED_EVT_STARTUP:
        if (elapsed_ms < DUR_STARTUP) {
            // Two full hue rotations over 2.5s, then fade to idle
            if (elapsed_ms < 2500) {
                uint16_t hue = (uint16_t)((uint32_t)elapsed_ms * 720 / 2500) % 360;
                uint8_t  val = (uint8_t)((uint32_t)elapsed_ms * 80 / 2500);
                if (val < 10) val = 10;
                hsv_to_rgb(hue, 255, val, r, g, b);
            } else {
                // Fade from last hue toward idle blue
                uint8_t fade = (uint8_t)((elapsed_ms - 2500) * 255 / 500);
                uint8_t hr, hg, hb;
                hsv_to_rgb(0, 255, 60, &hr, &hg, &hb); // Red end
                *r = lerp8(hr, 5, fade);
                *g = lerp8(hg, 8, fade);
                *b = lerp8(hb, 25, fade);
            }
            return true;
        }
        return false;

    // ── BLE Connected: 3 quick blue blinks ──
    case LED_EVT_BLE_CONNECTED:
        if (elapsed_ms < DUR_BLE_CONNECT) {
            // 200ms on, 200ms off × 3
            uint32_t phase = elapsed_ms % 400;
            if (phase < 200) {
                *r = 0; *g = 15; *b = 80;
            } else {
                *r = 0; *g = 0; *b = 0;
            }
            return true;
        }
        return false;

    // ── BLE Disconnected: Amber fade-out ──
    case LED_EVT_BLE_DISCONNECTED:
        if (elapsed_ms < DUR_BLE_DISCONNECT) {
            uint8_t val = (uint8_t)(60 - (uint32_t)elapsed_ms * 60 / DUR_BLE_DISCONNECT);
            *r = val;
            *g = val / 4;
            *b = 0;
            return true;
        }
        return false;

    // ── Weld Fire: Bright white flash → orange afterglow → fade ──
    case LED_EVT_WELD_FIRE:
        if (elapsed_ms < DUR_WELD_FIRE) {
            if (elapsed_ms < 80) {
                // Bright hot white
                *r = 255; *g = 200; *b = 120;
            } else if (elapsed_ms < 300) {
                // Orange afterglow
                uint8_t fade = (uint8_t)((elapsed_ms - 80) * 255 / 220);
                *r = lerp8(255, 100, fade);
                *g = lerp8(200, 40, fade);
                *b = lerp8(120, 0, fade);
            } else {
                // Fade to black
                uint8_t fade = (uint8_t)((elapsed_ms - 300) * 255 / 500);
                *r = lerp8(100, 0, fade);
                *g = lerp8(40, 0, fade);
                *b = 0;
            }
            return true;
        }
        return false;

    // ── Contact Detect: Yellow blink ──
    case LED_EVT_CONTACT_DETECT:
        if (elapsed_ms < DUR_CONTACT_DETECT) {
            if (elapsed_ms < 200) {
                *r = 60; *g = 50; *b = 0;
            } else {
                *r = 0; *g = 0; *b = 0;
            }
            return true;
        }
        return false;

    // ── OTA Complete: Green flash → fade ──
    case LED_EVT_OTA_COMPLETE:
        if (elapsed_ms < DUR_OTA_COMPLETE) {
            if (elapsed_ms < 300) {
                *r = 0; *g = 80; *b = 10;
            } else {
                uint8_t fade = (uint8_t)((elapsed_ms - 300) * 255 / 1200);
                *r = 0;
                *g = lerp8(80, 0, fade);
                *b = lerp8(10, 0, fade);
            }
            return true;
        }
        return false;

    default:
        return false;
    }
}

/**
 * Render one frame of the current base state.
 */
static void render_base(uint32_t abs_ms, uint8_t *r, uint8_t *g, uint8_t *b)
{
    switch (s_engine.base_state) {

    // ── Idle: Very subtle cool-white breathing ──
    case LED_EVT_IDLE: {
        uint8_t val = breathing(abs_ms, 4000, 10, 60);
        *r = val / 3;
        *g = val / 2;
        *b = val;
        break;
    }

    // ── Charging: Green breathing ──
    case LED_EVT_CHARGING: {
        uint8_t val = breathing(abs_ms, 3000, 15, 120);
        *r = val / 8;
        *g = val;
        *b = val / 6;
        break;
    }

    // ── Fully Charged: Solid dim green ──
    case LED_EVT_FULLY_CHARGED:
        *r = 0; *g = 35; *b = 3;
        break;

    // ── Low Voltage: Red pulsing ──
    case LED_EVT_LOW_VOLTAGE: {
        uint8_t val = breathing(abs_ms, 1500, 8, 80);
        *r = val;
        *g = 0;
        *b = 0;
        break;
    }

    // ── Protection Fault: Fast red strobe ──
    case LED_EVT_PROTECTION_FAULT: {
        uint32_t phase = abs_ms % 300;
        if (phase < 150) {
            *r = 90; *g = 0; *b = 0;
        } else {
            *r = 5; *g = 0; *b = 0;
        }
        break;
    }

    // ── OTA Active: Cyan breathing, intensity ∝ progress ──
    case LED_EVT_OTA_ACTIVE: {
        uint8_t pct = s_ota_percent;
        uint8_t hi = 20 + (uint8_t)((uint16_t)pct * 60 / 100);
        uint8_t val = breathing(abs_ms, 2000, 5, hi);
        *r = 0;
        *g = val;
        *b = val;
        break;
    }

    default:
        *r = 0; *g = 0; *b = 0;
        break;
    }
}

// ============================================================================
// Public API
// ============================================================================

void status_led_init(void)
{
    // Create event queue
    s_led_queue = xQueueCreate(LED_QUEUE_DEPTH, sizeof(led_event_t));
    if (!s_led_queue) {
        ESP_LOGE(TAG, "Failed to create LED event queue");
        return;
    }

    // Configure RMT TX channel
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = PIN_RGB_LED,
        .clk_src  = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_led_chan));

    // Create custom WS2812 encoder
    ESP_ERROR_CHECK(ws2812_encoder_create(&s_led_encoder));

    // Enable channel
    ESP_ERROR_CHECK(rmt_enable(s_led_chan));

    // Init engine state
    s_engine.base_state       = LED_EVT_IDLE;
    s_engine.overlay          = OVERLAY_NONE;
    s_engine.overlay_start_ms = 0;
    s_engine.boot_ms          = now_ms();

    ESP_LOGI(TAG, "Status LED initialized on GPIO%d", PIN_RGB_LED);
}

void status_led_set_event(led_event_t event)
{
    if (!s_led_queue) return;
    // Non-blocking send — drop if queue is full
    xQueueSend(s_led_queue, &event, 0);
}

void status_led_set_ota_progress(uint8_t percent)
{
    s_ota_percent = percent;
}

// ============================================================================
// Animation Task
// ============================================================================

static bool is_overlay_event(led_event_t evt)
{
    return (evt == LED_EVT_STARTUP ||
            evt == LED_EVT_BLE_CONNECTED ||
            evt == LED_EVT_BLE_DISCONNECTED ||
            evt == LED_EVT_WELD_FIRE ||
            evt == LED_EVT_CONTACT_DETECT ||
            evt == LED_EVT_OTA_COMPLETE);
}

static void led_task(void *arg)
{
    ESP_LOGI(TAG, "LED animation task started on Core %d", xPortGetCoreID());

    while (1) {
        // Drain all pending events from the queue
        led_event_t evt;
        while (xQueueReceive(s_led_queue, &evt, 0) == pdTRUE) {
            if (is_overlay_event(evt)) {
                // Start transient overlay
                s_engine.overlay          = (uint8_t)evt;
                s_engine.overlay_start_ms = now_ms();
            } else {
                // Update base state
                s_engine.base_state = evt;
            }
        }

        // Calculate current color
        uint8_t r = 0, g = 0, b = 0;
        uint32_t t = now_ms();

        if (s_engine.overlay != OVERLAY_NONE) {
            uint32_t elapsed = t - s_engine.overlay_start_ms;
            if (!render_overlay(elapsed, &r, &g, &b)) {
                // Overlay finished — clear and fall through to base
                s_engine.overlay = OVERLAY_NONE;
                render_base(t, &r, &g, &b);
            }
        } else {
            render_base(t, &r, &g, &b);
        }

        // Push color to LED
        ws2812_set_rgb(r, g, b);

        // 50 Hz update rate
        vTaskDelay(pdMS_TO_TICKS(LED_UPDATE_MS));
    }
}

void status_led_start_task(void)
{
    xTaskCreatePinnedToCore(
        led_task, "led_task",
        TASK_LED_STACK_SIZE, NULL,
        TASK_LED_PRIORITY, NULL,
        TASK_LED_CORE
    );
}

#endif // HAS_RGB_LED
