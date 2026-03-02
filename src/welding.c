/**
 * Welding Module — Core Logic + State Machine + ADC
 * 
 * State Machine (runs on Core 1, highest priority):
 *   IDLE → ARMED → PRE_FIRE → FIRING_P1 → PAUSE → FIRING_P2 → COOLDOWN → IDLE
 *        → BLOCKED (low voltage or protection fault)
 *        → ERROR (hardware fault)
 * 
 * ADC Task (runs on Core 1):
 *   Samples supercap voltage, protection rail, and contact pin.
 *   Feeds data to UI (via thread-safe updates) and welding state machine.
 * 
 * Safety:
 *   - OUTPUT_PIN defaults LOW (set in main.c before this module loads)
 *   - CHARGER_EN defaults LOW (charger enabled)
 *   - Weld only fires when physical START button pressed (MAN)
 *     or electrode contact held for S seconds (AUTO)
 *   - Protection checks run every cycle
 */

#include "welding.h"
#include "settings.h"
#include "audio.h"
#include "ui.h"
#include "ble_serial.h"
#include "ota.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <rom/ets_sys.h>

static const char *TAG = "Welding";

// ============================================================================
// Global State
// ============================================================================

volatile weld_status_t g_weld_status = {0};
volatile weld_state_t  g_weld_state  = WELD_STATE_IDLE;

// ADC handles
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali = NULL;

// Timing
static int64_t s_contact_start_us = 0;
static int64_t s_low_v_start_us = 0;
static int64_t s_protect_start_us = 0;
static bool s_start_btn_prev = false; // For edge detection
static uint8_t s_debounce_count = 0;  // Button debounce counter

// AUTO mode: one-shot flag — prevents re-firing while contact is held
// Set TRUE after weld fires, cleared only when contact is RELEASED
static bool s_auto_fired = false;

// MAN mode: one-shot flag — prevents re-firing while trigger button is held
// Set TRUE after weld fires, cleared only when button is RELEASED
static bool s_man_fired = false;

// Post-pulse charger hold-off: charger stays disabled for POST_PULSE_CHARGE_DELAY_MS
// This is INDEPENDENT of the weld state — welding is allowed during hold-off
static int64_t s_cooldown_start_us = 0;
static bool s_charger_holdoff = false;

// Was caps charged on last check?
static bool s_was_ready = false;

// ============================================================================
// ADC Initialization
// ============================================================================

static void adc_hw_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    // Configure all ADC channels
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_4, &chan_cfg)); // GPIO5 = voltage
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_5, &chan_cfg)); // GPIO6 = protection
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_6, &chan_cfg)); // GPIO7 = contact

    // Calibration (curve fitting for ESP32-S3)
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_4,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
        ESP_LOGI(TAG, "ADC calibration initialized");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
        s_adc_cali = NULL;
    }

    ESP_LOGI(TAG, "ADC initialized (CH4=voltage, CH5=protection, CH6=contact)");
}

/**
 * Read ADC channel with multi-sample averaging, then convert to voltage.
 * Takes ADC_NUM_SAMPLES readings and averages for noise reduction.
 */
static float adc_read_voltage(adc_channel_t channel, float multiplier, float cal_factor)
{
    int32_t sum = 0;
    int valid = 0;

    for (int i = 0; i < ADC_NUM_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc_handle, channel, &raw) == ESP_OK) {
            sum += raw;
            valid++;
        }
    }

    if (valid == 0) return 0.0f;
    int avg_raw = (int)(sum / valid);

    float voltage;
    if (s_adc_cali) {
        int mv = 0;
        adc_cali_raw_to_voltage(s_adc_cali, avg_raw, &mv);
        voltage = (float)mv / 1000.0f;
    } else {
        voltage = (float)avg_raw * ADC_VREF / ADC_MAX_VALUE;
    }

    return voltage * multiplier * cal_factor;
}

// ============================================================================
// Welding Pulse Generation
// ============================================================================

void welding_fire_pulse(void)
{
    // Safety: refuse to fire if P1 is 0 (no valid pulse possible)
    if (g_settings.p1 < PULSE_STEP_MS) {
        ESP_LOGW(TAG, "FIRE aborted: P1=%.1fms is below minimum", g_settings.p1);
        return;
    }

    ESP_LOGI(TAG, "⚡ FIRING: P1=%.1fms T=%.1fms P2=%.1fms",
             g_settings.p1, g_settings.t, g_settings.p2);

    // Snapshot parameters to prevent mid-pulse changes via BLE
    float snap_p1 = g_settings.p1;
    float snap_t  = g_settings.t;
    float snap_p2 = g_settings.p2;

    g_weld_state = WELD_STATE_PRE_FIRE;

    // 1. Disable charger
    gpio_set_level(PIN_CHARGER_EN, 1);
    ets_delay_us(CHARGER_SETTLE_US);

    // 2. Fire Pulse 1 (interrupts disabled for timing accuracy)
    g_weld_state = WELD_STATE_FIRING_P1;
    portDISABLE_INTERRUPTS();
    gpio_set_level(PIN_OUTPUT, 1);
    ets_delay_us((uint32_t)(snap_p1 * 1000.0f));
    gpio_set_level(PIN_OUTPUT, 0);
    portENABLE_INTERRUPTS();

    // 3. Pause + Pulse 2 (if P2 > 0)
    if (snap_p2 > 0.0f) {
        g_weld_state = WELD_STATE_PAUSE;
        ets_delay_us((uint32_t)(snap_t * 1000.0f));

        g_weld_state = WELD_STATE_FIRING_P2;
        portDISABLE_INTERRUPTS();
        gpio_set_level(PIN_OUTPUT, 1);
        ets_delay_us((uint32_t)(snap_p2 * 1000.0f));
        gpio_set_level(PIN_OUTPUT, 0);
        portENABLE_INTERRUPTS();
    }

    // 4. Start charger hold-off timer (charger stays DISABLED)
    //    The welding_task loop will re-enable it after POST_PULSE_CHARGE_DELAY_MS
    //    This does NOT block new welds — the state goes to IDLE immediately
    s_cooldown_start_us = esp_timer_get_time();
    s_charger_holdoff = true;
    g_weld_state = WELD_STATE_IDLE;

    // 5. Post-pulse: update counters and feedback
    settings_increment_weld_count();
    audio_play_weld_fire();
    ui_update_weld_state(WELD_STATE_IDLE);
    ui_update_weld_count(g_settings.session_welds, g_settings.total_welds);

    ESP_LOGI(TAG, "⚡ Pulse complete, charger hold-off %dms. Welds: %lu / %lu",
             POST_PULSE_CHARGE_DELAY_MS,
             (unsigned long)g_settings.session_welds,
             (unsigned long)g_settings.total_welds);
}

// ============================================================================
// Welding State Machine Task
// ============================================================================

void welding_init(void)
{
    // GPIOs already configured in gpio_init_safe_defaults()
    g_weld_state = WELD_STATE_IDLE;
    ESP_LOGI(TAG, "Welding state machine initialized");
}

void welding_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Welding task started on Core %d", xPortGetCoreID());

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // ==============================================
        // Protection Check (runs every cycle)
        // ==============================================

        // Check gate drive rail.
        // Guard: only start the fault timer if the reading is in a plausible
        // "hardware connected" range (> PROTECT_RAIL_PLAUSIBLE_V).
        //
        // A floating GPIO6 with a 100k+15k divider (no gate-drive connected)
        // reads 0.5–4V of resistive/capacitive noise. A real 13.5V rail through
        // the divider reads ~1.76V at the ADC pin → ~13.5V after the ×(115/15)
        // multiplier. So any reading below PROTECT_RAIL_PLAUSIBLE_V is treated
        // as "not connected / bench mode".
        float prot_v = g_weld_status.protection_voltage;
        if (prot_v > PROTECT_RAIL_PLAUSIBLE_V &&
            (prot_v < PROTECT_RAIL_MIN_V || prot_v > PROTECT_RAIL_MAX_V)) {
            if (s_protect_start_us == 0) {
                s_protect_start_us = esp_timer_get_time();
            } else if ((esp_timer_get_time() - s_protect_start_us) > (PROTECT_CONFIRM_MS * 1000LL)) {
                if (g_weld_state != WELD_STATE_BLOCKED) {
                    g_weld_status.protection_fault = true;
                    g_weld_state = WELD_STATE_BLOCKED;
                    ui_update_weld_state(WELD_STATE_BLOCKED);
                    audio_play_error();
                    ESP_LOGW(TAG, "PROTECTION FAULT: Rail=%.1fV (expected %.1f-%.1fV)",
                             prot_v, PROTECT_RAIL_MIN_V, PROTECT_RAIL_MAX_V);
                }
            }
        } else {
            s_protect_start_us = 0;
            if (g_weld_status.protection_fault) {
                g_weld_status.protection_fault = false;
                if (g_weld_state == WELD_STATE_BLOCKED && !g_weld_status.low_voltage_block) {
                    g_weld_state = WELD_STATE_IDLE;
                    ui_update_weld_state(WELD_STATE_IDLE);
                    ESP_LOGI(TAG, "Protection fault cleared");
                }
            }
        }

        // Check supercap voltage
        float cap_v = g_weld_status.supercap_voltage;
        g_weld_status.low_voltage_warn = (cap_v < LOW_VOLTAGE_WARN && cap_v > 0.1f);

        if (cap_v < LOW_VOLTAGE_BLOCK && cap_v > 0.1f) {
            if (s_low_v_start_us == 0) {
                s_low_v_start_us = esp_timer_get_time();
            } else if ((esp_timer_get_time() - s_low_v_start_us) > (LOW_V_CONFIRM_MS * 1000LL)) {
                g_weld_status.low_voltage_block = true;
                if (g_weld_state == WELD_STATE_IDLE) {
                    g_weld_state = WELD_STATE_BLOCKED;
                    ui_update_weld_state(WELD_STATE_BLOCKED);
                    audio_play_error();
                    ESP_LOGW(TAG, "LOW VOLTAGE: %.1fV < %.1fV", cap_v, LOW_VOLTAGE_BLOCK);
                }
            }
        } else {
            s_low_v_start_us = 0;
            if (g_weld_status.low_voltage_block) {
                g_weld_status.low_voltage_block = false;
                if (g_weld_state == WELD_STATE_BLOCKED && !g_weld_status.protection_fault) {
                    g_weld_state = WELD_STATE_IDLE;
                    ui_update_weld_state(WELD_STATE_IDLE);
                    ESP_LOGI(TAG, "Low voltage cleared");
                }
            }
        }

        // "Ready" notification when caps fully charged
        if (cap_v >= SUPERCAP_FULL_V && !s_was_ready) {
            s_was_ready = true;
            audio_play_ready();
            ESP_LOGI(TAG, "Supercaps fully charged: %.1fV", cap_v);
        } else if (cap_v < SUPERCAP_FULL_V - 0.5f) {
            s_was_ready = false; // Hysteresis
        }

        // ==============================================
        // Charger Hold-off Timer (non-blocking)
        // Charger stays disabled for POST_PULSE_CHARGE_DELAY_MS after pulse,
        // but this does NOT prevent arming or welding.
        // ==============================================
        if (s_charger_holdoff) {
            int64_t elapsed_us = esp_timer_get_time() - s_cooldown_start_us;
            if (elapsed_us >= (int64_t)POST_PULSE_CHARGE_DELAY_MS * 1000LL) {
                gpio_set_level(PIN_CHARGER_EN, 0); // Re-enable charger
                s_charger_holdoff = false;
                s_cooldown_start_us = 0;
                ESP_LOGI(TAG, "Charger hold-off complete, charger re-enabled");
            }
        }

        // ==============================================
        // Welding Logic (only if not blocked and NOT in OTA)
        // ==============================================

        // SAFETY: Block ALL welding during OTA firmware updates.
        // Firing during flash writes could corrupt the update AND is dangerous.
        if (ota_is_active()) {
            // Force IDLE state and skip all trigger logic during OTA
            if (g_weld_state == WELD_STATE_ARMED) {
                g_weld_state = WELD_STATE_IDLE;
                ui_update_weld_state(WELD_STATE_IDLE);
            }
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
            continue;
        }

        if (g_weld_state == WELD_STATE_IDLE || g_weld_state == WELD_STATE_ARMED) {
            // --- MAN Mode: Physical button trigger (with debounce + one-shot) ---
            if (!g_settings.auto_mode) {
                bool btn_raw = (gpio_get_level(PIN_START) == 0); // Active LOW

                // Debounce: require stable reading for DEBOUNCE_MS / 10ms = N cycles
                if (btn_raw != s_start_btn_prev) {
                    s_debounce_count++;
                    if (s_debounce_count >= (DEBOUNCE_MS / 10 + 1)) {
                        bool btn_pressed = btn_raw;
                        bool was_pressed = s_start_btn_prev;
                        s_start_btn_prev = btn_pressed;
                        s_debounce_count = 0;

                        if (btn_pressed && !was_pressed) {
                            // FALLING EDGE — button just pressed
                            if (!s_man_fired) { // One-shot: ignore if already fired this press
                                if (!g_weld_status.low_voltage_block && !g_weld_status.protection_fault) {
                                    welding_fire_pulse();
                                    s_man_fired = true; // Block re-fire until button released
                                    ESP_LOGI(TAG, "MAN: Weld fired, waiting for button release");
                                } else {
                                    audio_play_error();
                                    ESP_LOGW(TAG, "MAN trigger blocked: lowV=%d protFault=%d",
                                             g_weld_status.low_voltage_block,
                                             g_weld_status.protection_fault);
                                }
                            }
                        } else if (!btn_pressed && was_pressed) {
                            // RISING EDGE — button released: clear one-shot lock
                            if (s_man_fired) {
                                s_man_fired = false;
                                ESP_LOGI(TAG, "MAN: Button released, ready for next weld");
                            }
                        }
                    }
                } else {
                    s_debounce_count = 0;
                }
            }

            // --- AUTO Mode: Contact detection ---
            if (g_settings.auto_mode) {
                bool contact = g_weld_status.contact_detected;
                if (contact) {
                    // If we already fired on this contact, ignore until released
                    if (s_auto_fired) {
                        // Do nothing — waiting for user to lift electrodes
                    } else if (g_weld_state == WELD_STATE_IDLE) {
                        // Contact just detected — start timing
                        g_weld_state = WELD_STATE_ARMED;
                        s_contact_start_us = esp_timer_get_time();
                        audio_play_contact();
                        ui_update_weld_state(WELD_STATE_ARMED);
                        ESP_LOGI(TAG, "AUTO: Contact detected, arming (S=%.1fs)", g_settings.s_value);
                    } else if (g_weld_state == WELD_STATE_ARMED) {
                        // Contact held — check if S delay has elapsed
                        int64_t elapsed_us = esp_timer_get_time() - s_contact_start_us;
                        float elapsed_s = (float)elapsed_us / 1000000.0f;
                        if (elapsed_s >= g_settings.s_value) {
                            if (!g_weld_status.low_voltage_block && !g_weld_status.protection_fault) {
                                welding_fire_pulse();
                                s_auto_fired = true; // Block re-fire until contact released
                                ESP_LOGI(TAG, "AUTO: Weld complete, waiting for contact release");
                            }
                            s_contact_start_us = 0;
                        }
                    }
                } else {
                    // Contact RELEASED — clear one-shot lock and disarm
                    if (s_auto_fired) {
                        s_auto_fired = false;
                        ESP_LOGI(TAG, "AUTO: Contact released, ready for next weld");
                    }
                    if (g_weld_state == WELD_STATE_ARMED) {
                        g_weld_state = WELD_STATE_IDLE;
                        s_contact_start_us = 0;
                        ui_update_weld_state(WELD_STATE_IDLE);
                    }
                }
            }
        }

        // 100Hz polling (10ms period)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// ADC Sampling Task
// ============================================================================

void adc_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ADC task started on Core %d", xPortGetCoreID());

    adc_hw_init();

    // EMA filter state
    float ema_cap = 0.0f;
    float ema_prot = 0.0f;
    bool ema_initialized = false;

    uint32_t graph_counter = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Read all ADC channels (multi-sampled internally)
        float v_cap_raw = adc_read_voltage(ADC_CHANNEL_4, SUPERCAP_V_MULT, g_settings.adc_cal_voltage);
        float v_prot_raw = adc_read_voltage(ADC_CHANNEL_5, PROTECTION_V_MULT, g_settings.adc_cal_protection);
        float v_contact = adc_read_voltage(ADC_CHANNEL_6, 1.0f, 1.0f); // Raw 0–3.3V (no EMA — needs fast response)

        // Apply EMA smoothing to voltage channels (not contact — it needs instant response)
        if (!ema_initialized) {
            ema_cap = v_cap_raw;
            ema_prot = v_prot_raw;
            ema_initialized = true;
        } else {
            ema_cap = ADC_EMA_ALPHA * v_cap_raw + (1.0f - ADC_EMA_ALPHA) * ema_cap;
            ema_prot = ADC_EMA_ALPHA * v_prot_raw + (1.0f - ADC_EMA_ALPHA) * ema_prot;
        }

        float v_cap = ema_cap;
        float v_prot = ema_prot;

        // Update global status (read by welding task)
        g_weld_status.supercap_voltage = v_cap;
        g_weld_status.protection_voltage = v_prot;
        g_weld_status.contact_voltage = v_contact;
        g_weld_status.contact_detected = (v_contact > CONTACT_THRESHOLD_V);

        // Update UI voltage display (every sample)
        ui_update_voltage(v_cap);
        ui_update_protection(v_prot);

        // Add point to voltage graph (every ADC_SAMPLE_INTERVAL)
        graph_counter++;
        if (graph_counter >= (ADC_SAMPLE_INTERVAL / 100)) { // 500ms / 100ms = every 5 samples
            ui_graph_add_point(v_cap);
            graph_counter = 0;
        }

        // BLE status update (every 1 second)
        static uint32_t ble_counter = 0;
        ble_counter++;
        if (ble_counter >= 10) { // 10 × 100ms = 1s
            ble_serial_send_status();
            ble_counter = 0;
        }

        // 10Hz sampling (100ms period)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

const char* welding_state_str(weld_state_t state)
{
    switch (state) {
        case WELD_STATE_IDLE:       return "READY";
        case WELD_STATE_ARMED:      return "ARMED";
        case WELD_STATE_PRE_FIRE:   return "PRE-FIRE";
        case WELD_STATE_FIRING_P1:  return "FIRING P1";
        case WELD_STATE_PAUSE:      return "PAUSE";
        case WELD_STATE_FIRING_P2:  return "FIRING P2";
        case WELD_STATE_COOLDOWN:   return "COOLDOWN";
        case WELD_STATE_BLOCKED:    return "BLOCKED";
        case WELD_STATE_ERROR:      return "ERROR";
        default:                    return "UNKNOWN";
    }
}
