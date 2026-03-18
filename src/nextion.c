/**
 * MyWeld ESP32-S3 — Nextion HMI Display Driver
 *
 * Placeholder implementation. Core UART framework is in place;
 * the UI update logic will be implemented after the user confirms
 * the exact Nextion model and designs the HMI layout in Nextion Editor.
 *
 * Only compiled when DISPLAY_TYPE == DISPLAY_NEXTION.
 */

#include "board_config.h"

#if HAS_NEXTION

#include "nextion.h"
#include "config.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "nextion";

// Nextion command terminator (3 × 0xFF)
static const uint8_t NEXTION_TERM[] = {0xFF, 0xFF, 0xFF};

#define NEXTION_TX_BUF_SIZE  256
#define NEXTION_RX_BUF_SIZE  256

// ============================================================================
// Low-level UART
// ============================================================================

void nextion_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate  = NEXTION_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(NEXTION_UART_NUM, NEXTION_RX_BUF_SIZE * 2,
                        NEXTION_TX_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(NEXTION_UART_NUM, &uart_cfg);
    uart_set_pin(NEXTION_UART_NUM, PIN_NEXTION_TX, PIN_NEXTION_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "Nextion UART%d initialized (TX=IO%d, RX=IO%d, %d baud)",
             NEXTION_UART_NUM, PIN_NEXTION_TX, PIN_NEXTION_RX, NEXTION_BAUD_RATE);

    // Send initial page command
    vTaskDelay(pdMS_TO_TICKS(500));  // Wait for Nextion to boot
    nextion_goto_page(0);
}

void nextion_send_cmd(const char *cmd) {
    uart_write_bytes(NEXTION_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(NEXTION_UART_NUM, (const char *)NEXTION_TERM, 3);
}

void nextion_set_text(const char *obj_name, const char *text) {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj_name, text);
    nextion_send_cmd(buf);
}

void nextion_set_value(const char *obj_name, int32_t value) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s.val=%ld", obj_name, (long)value);
    nextion_send_cmd(buf);
}

void nextion_set_progress(const char *obj_name, uint8_t percent) {
    if (percent > 100) percent = 100;
    nextion_set_value(obj_name, percent);
}

void nextion_goto_page(uint8_t page_id) {
    char buf[16];
    snprintf(buf, sizeof(buf), "page %d", page_id);
    nextion_send_cmd(buf);
}

bool nextion_process_events(void) {
    uint8_t buf[16];
    int len = uart_read_bytes(NEXTION_UART_NUM, buf, sizeof(buf),
                              pdMS_TO_TICKS(10));
    if (len <= 0) return false;

    // TODO: Parse Nextion return codes (0x65 = touch event, 0x66 = page ID, etc.)
    // Will be implemented after Nextion model is confirmed and HMI designed.
    ESP_LOGD(TAG, "Received %d bytes from Nextion", len);
    return true;
}

void nextion_update_status(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
) {
    // TODO: Map these values to specific Nextion UI objects
    // Example (object names depend on HMI design):
    char buf[32];

    snprintf(buf, sizeof(buf), "%.1fV", voltage_v);
    nextion_set_text("tVolt", buf);

    nextion_set_progress("jCharge", charge_pct);

    snprintf(buf, sizeof(buf), "%.0f", p1_ms);
    nextion_set_text("tP1", buf);

    nextion_set_text("tStatus", status_text);
    nextion_set_text("tMode", auto_mode ? "AUTO" : "MAN");

    // BLE indicator
    nextion_set_value("vBLE", ble_connected ? 1 : 0);
}

// ============================================================================
// Display HAL interface implementation
// ============================================================================

void display_hal_init(void) {
    nextion_init();
}

void display_hal_update(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
) {
    nextion_update_status(voltage_v, charge_pct,
                          p1_ms, t_ms, p2_ms, p3_ms, p4_ms,
                          auto_mode, s_delay,
                          status_text, ble_connected);
}

void display_hal_show_status(const char *message) {
    nextion_set_text("tStatus", message);
}

void display_hal_weld_fired(void) {
    // Flash a visual indicator on the Nextion
    nextion_send_cmd("vis pFlash,1");    // Show flash overlay
    vTaskDelay(pdMS_TO_TICKS(100));
    nextion_send_cmd("vis pFlash,0");    // Hide flash overlay
}

void display_hal_set_brightness(uint8_t percent) {
    char buf[16];
    snprintf(buf, sizeof(buf), "dim=%d", percent);
    nextion_send_cmd(buf);
}

static void nextion_task(void *pvParams) {
    (void)pvParams;
    while (1) {
        nextion_process_events();
        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz event polling
    }
}

void display_hal_start_task(void) {
    xTaskCreatePinnedToCore(nextion_task, "nextion_task", 4096, NULL,
                            TASK_UI_PRIORITY, NULL, TASK_UI_CORE);
    ESP_LOGI(TAG, "Nextion event task started");
}

#endif // HAS_NEXTION
