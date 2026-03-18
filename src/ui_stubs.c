/**
 * MyWeld ESP32-S3 — UI Stub Implementations
 *
 * When LVGL is NOT active (Nextion / LCD 2004 / LCD 1604 variants),
 * these stubs receive ui_* calls from welding.c and other modules,
 * cache the latest values into static variables.
 *
 * The actual LCD writes happen in lcd_update_task (lcd2004.c), which
 * periodically reads these cached values and calls display_hal_update().
 * This avoids blocking the ADC/welding tasks with slow I2C transactions.
 *
 * Data flow:
 *   adc_task (Core 1) → ui_update_voltage()  → caches s_voltage
 *   welding  (Core 1) → ui_update_weld_state() → caches s_weld_state
 *   lcd_update_task (Core 0) → reads caches → display_hal_update() → I2C → LCD
 */

#include "board_config.h"

#if !HAS_LVGL

#include "ui.h"
#include "display_hal.h"
#include "settings.h"
#include "welding.h"
#include "ble_serial.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

static const char *TAG = "ui_stub";

// ============================================================================
// Cached display state — written by ui_* calls (any core),
//                        read by lcd_update_task (Core 0)
// ============================================================================
static volatile float    s_voltage     = 0.0f;
static volatile uint8_t  s_weld_state  = WELD_STATE_IDLE;
static volatile uint32_t s_session     = 0;
static volatile uint32_t s_total       = 0;
static volatile bool     s_dirty       = true;  // Force initial refresh
static volatile bool     s_paused      = false; // Pause refresh during reboot

// OTA overlay state — written by ui_show/hide_ota_progress (any core),
//                     rendered by lcd_update_task (Core 0) only
static volatile bool    s_ota_overlay = false;
static volatile uint8_t s_ota_percent = 0;

// ============================================================================
// Public: called by lcd_update_task to push cached data to display
// ============================================================================

void ui_stub_refresh_display(void)
{
    // ── OTA overlay takes priority over normal dashboard ──
    if (s_ota_overlay) {
        char buf[21];

        lcd2004_print_row(0, "==  OTA UPDATING  ==");

        if (s_ota_percent == 0) {
            snprintf(buf, sizeof(buf), "   Preparing...     ");
        } else {
            snprintf(buf, sizeof(buf), "    Progress: %3u%%  ", s_ota_percent);
        }
        lcd2004_print_row(1, buf);

        // Visual progress bar [################]
        int filled = (s_ota_percent * 16 + 50) / 100;
        if (filled > 16) filled = 16;
        char bar[21];
        bar[0] = '[';
        for (int i = 0; i < 16; i++) {
            bar[1 + i] = (i < filled) ? '#' : '.';
        }
        bar[17] = ']';
        bar[18] = ' ';
        bar[19] = ' ';
        bar[20] = '\0';
        lcd2004_print_row(2, bar);

        lcd2004_print_row(3, " DO NOT POWER OFF!  ");

        s_dirty = false;
        return;
    }

    // ── Normal dashboard ──
    // Calculate charge percentage (0–100) from voltage
    float v = s_voltage;
    float max_v = settings_get_max_voltage();
    uint8_t pct = 0;
    if (max_v > 0.1f) {
        float ratio = v / max_v;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < 0.0f) ratio = 0.0f;
        pct = (uint8_t)(ratio * 100.0f);
    }

    // Build status text from weld state
    const char *status = welding_state_str((weld_state_t)s_weld_state);

    // Check BLE connection
    bool ble = ble_serial_is_connected();

    display_hal_update(
        v, pct,
        g_settings.p1, g_settings.t, g_settings.p2,
        g_settings.p3, g_settings.p4,
        g_settings.auto_mode, g_settings.s_value,
        status, ble
    );

    s_dirty = false;
}

bool ui_stub_is_dirty(void)
{
    return s_dirty && !s_paused;
}

// ============================================================================
// UI interface implementations — cache only, NO I2C writes
// ============================================================================

void ui_init(void) {
    ESP_LOGI(TAG, "ui_init (non-LVGL — forwarding to display_hal)");
}

void ui_task(void *pvParameters) {
    ESP_LOGI(TAG, "ui_task stub — not used for LCD (lcd_update_task handles it)");
    // This task is NOT started for LCD variants (main.c calls
    // display_hal_start_task() instead). If somehow started, just idle.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ui_update_voltage(float voltage) {
    s_voltage = voltage;
    s_dirty = true;
}

void ui_update_protection(float voltage) {
    (void)voltage;  // Not shown on character LCD
}

void ui_update_weld_state(uint8_t state) {
    s_weld_state = state;
    s_dirty = true;
}

void ui_update_weld_count(uint32_t session, uint32_t total) {
    s_session = session;
    s_total   = total;
    s_dirty = true;
}

void ui_graph_add_point(float voltage) {
    (void)voltage;  // No graph on character LCD
}

void ui_set_theme(uint8_t theme) {
    (void)theme;
}

void ui_refresh_params(void) {
    s_dirty = true;  // BLE changed parameters — mark for refresh
}

void ui_show_notification(const char *message, uint16_t duration_ms) {
    ESP_LOGI(TAG, "Notification: %s (%ums)", message, duration_ms);
    // Notification will be shown on next lcd_update_task cycle via status row
    display_hal_show_status(message);
}

void ui_trigger_reboot_countdown(bool is_factory_reset) {
    const char *title = is_factory_reset ? "FACTORY RESET" : "REBOOTING";
    ESP_LOGW(TAG, "%s — rebooting in 3s...", title);

    // Pause lcd_update_task so it doesn't overwrite our message
    s_paused = true;

    // Full-screen reboot message
    lcd2004_print_row(0, "                    ");
    lcd2004_print_row(1, is_factory_reset
        ? "  >> FACTORY RESET  "
        : "  >> REBOOTING...   ");

    // 3-2-1 countdown
    for (int i = 3; i > 0; i--) {
        char buf[21];
        snprintf(buf, sizeof(buf), "      %d ...         ", i);
        lcd2004_print_row(2, buf);
        lcd2004_print_row(3, "                    ");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    lcd2004_print_row(2, "    Restarting...   ");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

/**
 * Show OTA progress overlay on LCD.
 *
 * IMPORTANT: This function is called from multiple cores (BLE callback on
 * Core 0, ota_flush_task on Core 1). It does NOT write to I2C directly.
 * Instead, it sets volatile state that lcd_update_task (Core 0) will render
 * on its next cycle, avoiding I2C bus contention and Interrupt WDT crashes.
 */
void ui_show_ota_progress(uint8_t percent) {
    ESP_LOGI(TAG, "OTA progress: %u%%", percent);
    s_ota_overlay = true;
    s_ota_percent = percent;
    s_dirty = true;
}

void ui_hide_ota_progress(void) {
    ESP_LOGI(TAG, "OTA complete — resuming normal display");
    s_ota_overlay = false;
    s_dirty = true;
}

#endif // !HAS_LVGL
