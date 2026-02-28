/**
 * MyWeld ESP32-S3 — Main Entry Point
 *
 * Supercap Spot Welder Controller
 * Board: JC3248W535 (Guition ESP32-S3, 480×320 TFT)
 *
 * Architecture:
 *   Core 0: LVGL UI rendering + BLE serial
 *   Core 1: Welding logic + ADC sampling + I2S audio
 *
 * Safety: OUTPUT_PIN and CHARGER_EN are set to safe defaults
 *         BEFORE any other initialization.
 */

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>


#include "audio.h"
#include "ble_serial.h"
#include "config.h"
#include "display.h"
#include "ota.h"
#include "settings.h"
#include "ui.h"
#include "welding.h"


static const char *TAG = "MyWeld";

/**
 * Initialize GPIO pins to safe defaults.
 * MUST be called FIRST before any other init.
 */
static void gpio_init_safe_defaults(void) {
  // OUTPUT_PIN (MOSFET fire) → LOW immediately
  gpio_config_t output_conf = {
      .pin_bit_mask = (1ULL << PIN_OUTPUT),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&output_conf);
  gpio_set_level(PIN_OUTPUT, 0);

  // CHARGER_EN → LOW (charger enabled by default)
  gpio_config_t charger_conf = {
      .pin_bit_mask = (1ULL << PIN_CHARGER_EN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&charger_conf);
  gpio_set_level(PIN_CHARGER_EN, 0);

  // START_PIN (weld button) → input with pull-up
  gpio_config_t start_conf = {
      .pin_bit_mask = (1ULL << PIN_START),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&start_conf);

  ESP_LOGI(TAG, "GPIO safe defaults initialized");
  ESP_LOGI(TAG, "  OUTPUT_PIN (IO%d) = LOW", PIN_OUTPUT);
  ESP_LOGI(TAG, "  CHARGER_EN (IO%d) = LOW (charger ON)", PIN_CHARGER_EN);
  ESP_LOGI(TAG, "  START_PIN  (IO%d) = INPUT_PULLUP", PIN_START);
}

/**
 * Initialize NVS (Non-Volatile Storage)
 */
static void nvs_init(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition truncated, erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGI(TAG, "NVS initialized");
}

/**
 * Print boot banner
 */
static void print_banner(void) {
  printf("\n");
  printf("╔══════════════════════════════════════════╗\n");
  printf("║  ⚡ MyWeld ESP32-S3 Spot Welder v%s  ║\n", FW_VERSION_STRING);
  printf("║  Board: JC3248W535 (Guition)             ║\n");
  printf("║  Built: %s %s              ║\n", FW_BUILD_DATE, FW_BUILD_TIME);
  printf("╚══════════════════════════════════════════╝\n");
  printf("\n");

  ESP_LOGI(TAG, "Free heap: %lu bytes",
           (unsigned long)esp_get_free_heap_size());
  ESP_LOGI(TAG, "Free PSRAM: %lu bytes",
           (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void app_main(void) {
  // ========================================
  // PHASE 1: Safety-critical initialization
  // ========================================
  gpio_init_safe_defaults(); // MUST be first!

  // ========================================
  // PHASE 2: System initialization
  // ========================================
  nvs_init();
  print_banner();

  // Load saved settings from NVS
  settings_init();

  // Load ADC calibration from dedicated partition (survives factory reset)
  calibration_init();

  // OTA rollback check — mark firmware valid if we booted successfully
  ota_init();

  // ========================================
  // PHASE 3: Peripheral initialization
  // ========================================

  // Silence IDF's internal lcd_panel.io.i2c error logger.
  // The AXS15231B touch controller does not ACK I2C when idle (no finger),
  // causing the IDF driver to log ESP_LOGE before returning the error to us.
  // We already handle the error silently in touch_axs15231b_read_data(),
  // but we can't suppress the IDF's own log from our wrapper — so we do it
  // here.
  esp_log_level_set("lcd_panel.io.i2c", ESP_LOG_NONE);

  // Initialize I2S audio (speaker on P6)
  audio_init();

  // Initialize display (QSPI + LVGL)
  display_init();

  // ========================================
  // PHASE 4: Application initialization
  // ========================================

  // Build LVGL UI
  ui_init();

  // Initialize welding state machine
  welding_init();

  // Initialize BLE serial
  ble_serial_init();

  // ========================================
  // PHASE 5: Start FreeRTOS tasks
  // ========================================

  // Core 0: LVGL UI tick + rendering
  xTaskCreatePinnedToCore(ui_task, "ui_task", TASK_UI_STACK_SIZE, NULL,
                          TASK_UI_PRIORITY, NULL, TASK_UI_CORE);

  // Core 1: ADC voltage sampling
  xTaskCreatePinnedToCore(adc_task, "adc_task", TASK_ADC_STACK_SIZE, NULL,
                          TASK_ADC_PRIORITY, NULL, TASK_ADC_CORE);

  // Core 1: Welding state machine
  xTaskCreatePinnedToCore(welding_task, "welding_task", TASK_WELDING_STACK_SIZE,
                          NULL, TASK_WELDING_PRIORITY, NULL, TASK_WELDING_CORE);

  // Play startup melody
  audio_play_startup();

  ESP_LOGI(TAG, "All tasks started. MyWeld is ready! ⚡");
}
