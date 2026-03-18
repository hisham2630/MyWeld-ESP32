/**
 * MyWeld ESP32-S3 — 20×4 Character LCD Driver (I2C, PCF8574)
 *
 * Drives an HD44780-compatible character LCD via PCF8574 I2C backpack.
 * Only compiled when DISPLAY_TYPE == DISPLAY_LCD_2004.
 *
 * LCD Layout (4 rows × 20 cols):
 *   Row 0: "5.7V  ████████  95%"    ← Voltage + charge bar + percentage
 *   Row 1: "P1:05 T:00 P2:00    "   ← Pulse parameters
 *   Row 2: "P3:00 P4:00     AUTO"   ← More pulses + mode
 *   Row 3: ">>> READY <<<      B"   ← Status + BLE indicator
 *
 * Wiring: SDA → GPIO8, SCL → GPIO9, VCC → 5V (USB VBUS), GND → GND
 *   Note: On DevKitC-1 with dual USB-C, 5V comes from the USB port (not COM port)
 */

#include "board_config.h"

#if HAS_CHAR_LCD

#include "lcd2004.h"
#include "config.h"
#include "ui.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "lcd2004";

// HD44780 commands
#define LCD_CMD_CLEAR       0x01
#define LCD_CMD_HOME        0x02
#define LCD_CMD_ENTRY_MODE  0x06  // Increment cursor, no shift
#define LCD_CMD_DISPLAY_ON  0x0C  // Display ON, cursor OFF, blink OFF
#define LCD_CMD_FUNCTION    0x28  // 4-bit mode, 2-line, 5×8 dots
#define LCD_CMD_SET_DDRAM   0x80

// PCF8574 bit mapping (directly wired to HD44780)
#define LCD_RS   (1 << 0)  // Register select
#define LCD_RW   (1 << 1)  // Read/Write
#define LCD_EN   (1 << 2)  // Enable
#define LCD_BL   (1 << 3)  // Backlight

// Row start addresses for a 20×4 display
// (differs from 16×4 which uses 0x00, 0x40, 0x10, 0x50)
static const uint8_t ROW_ADDR[4] = {0x00, 0x40, 0x14, 0x54};

static bool s_backlight = true;

// ============================================================================
// I2C helpers
// ============================================================================

static esp_err_t lcd_i2c_write_byte(uint8_t data) {
    uint8_t buf = data | (s_backlight ? LCD_BL : 0);
    return i2c_master_write_to_device(LCD_I2C_NUM, LCD_I2C_ADDR,
                                       &buf, 1, pdMS_TO_TICKS(100));
}

static void lcd_pulse_enable(uint8_t data) {
    lcd_i2c_write_byte(data | LCD_EN);
    esp_rom_delay_us(1);
    lcd_i2c_write_byte(data & ~LCD_EN);
    esp_rom_delay_us(50);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode;
    lcd_pulse_enable(data);
}

static void lcd_send_byte(uint8_t byte, uint8_t mode) {
    lcd_send_nibble(byte & 0xF0, mode);           // High nibble
    lcd_send_nibble((byte << 4) & 0xF0, mode);    // Low nibble
}

static void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, 0);  // RS=0 for command
}

static void lcd_data(uint8_t ch) {
    lcd_send_byte(ch, LCD_RS);  // RS=1 for data
}

// ============================================================================
// Public API
// ============================================================================

void lcd2004_init(void) {
    // Initialize I2C
    i2c_config_t i2c_cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_LCD_SDA,
        .scl_io_num       = PIN_LCD_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,  // 100kHz standard I2C
    };
    i2c_param_config(LCD_I2C_NUM, &i2c_cfg);
    i2c_driver_install(LCD_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);

    ESP_LOGI(TAG, "I2C initialized (SDA=IO%d, SCL=IO%d)", PIN_LCD_SDA, PIN_LCD_SCL);

    // --- I2C bus scan: detect all devices ---
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int found = 0;
    uint8_t found_addr = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t probe = 0;
        esp_err_t ret = i2c_master_write_to_device(
            LCD_I2C_NUM, addr, &probe, 1, pdMS_TO_TICKS(50));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  I2C device at 0x%02X", addr);
            found_addr = addr;
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGE(TAG, "No I2C devices found! Check: SDA=IO%d, SCL=IO%d, VCC=5V, GND",
                 PIN_LCD_SDA, PIN_LCD_SCL);
        return;
    }

    // Warn if found address doesn't match configured
    if (found == 1 && found_addr != LCD_I2C_ADDR) {
        ESP_LOGE(TAG, "ADDRESS MISMATCH: configured=0x%02X, found=0x%02X — update board_config.h",
                 LCD_I2C_ADDR, found_addr);
    }

    // Turn on backlight
    s_backlight = true;
    uint8_t bl = LCD_BL;
    i2c_master_write_to_device(LCD_I2C_NUM, LCD_I2C_ADDR, &bl, 1, pdMS_TO_TICKS(100));

    // HD44780 initialization sequence (4-bit mode)
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for LCD power-up

    // Send 0x30 three times to reliably enter 8-bit mode first
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(5));   // Wait >4.1ms
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(2));   // Wait >100us
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(2));   // Switch to 4-bit

    lcd_command(LCD_CMD_FUNCTION);    // 4-bit, 2-line, 5×8
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_command(LCD_CMD_DISPLAY_ON);  // Display ON
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_command(LCD_CMD_CLEAR);       // Clear
    vTaskDelay(pdMS_TO_TICKS(3));     // Clear needs 1.52ms
    lcd_command(LCD_CMD_ENTRY_MODE);  // Increment, no shift
    vTaskDelay(pdMS_TO_TICKS(1));

    // Show welcome message
    lcd2004_print_row(0, "    MyWeld v1.0     ");
    lcd2004_print_row(1, "   Spot Welder      ");
    lcd2004_print_row(2, "   Initializing...  ");
    lcd2004_print_row(3, "                    ");

    ESP_LOGI(TAG, "LCD 20x4 initialized (addr=0x%02X, SDA=IO%d, SCL=IO%d)",
             LCD_I2C_ADDR, PIN_LCD_SDA, PIN_LCD_SCL);
}

void lcd2004_clear(void) {
    lcd_command(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd2004_set_cursor(uint8_t row, uint8_t col) {
    if (row > 3) row = 3;
    if (col > 19) col = 19;
    lcd_command(LCD_CMD_SET_DDRAM | (ROW_ADDR[row] + col));
}

void lcd2004_print(const char *text) {
    while (*text) {
        lcd_data((uint8_t)*text++);
    }
}

void lcd2004_print_row(uint8_t row, const char *text) {
    lcd2004_set_cursor(row, 0);
    // Pad to 20 chars to clear any previous content
    char buf[21];
    snprintf(buf, sizeof(buf), "%-20.20s", text);
    lcd2004_print(buf);
}

void lcd2004_backlight(bool on) {
    s_backlight = on;
    lcd_i2c_write_byte(0);  // Refresh backlight state
}

// ============================================================================
// Display HAL interface implementation
// ============================================================================

void display_hal_init(void) {
    lcd2004_init();
}

void display_hal_update(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
) {
    char line[21];  // 20 chars + null

    // Row 0: Voltage + charge bar + percentage
    // "5.7V  ████████  95%"
    int bars = (charge_pct * 8) / 100;  // 8 bar chars max (more room on 20-col)
    char bar_str[9] = "        ";
    for (int i = 0; i < bars && i < 8; i++) bar_str[i] = 0xFF;  // Full block
    snprintf(line, sizeof(line), "%4.1fV %s %3d%%", voltage_v, bar_str, charge_pct);
    lcd2004_print_row(0, line);

    // Row 1: Pulse parameters (more room now)
    // "P1:05 T:00 P2:00    "
    snprintf(line, sizeof(line), "P1:%02.0f T:%02.0f P2:%02.0f",
             p1_ms, t_ms, p2_ms);
    lcd2004_print_row(1, line);

    // Row 2: More pulses + S delay (mode moved to Row 3 for clarity)
    // "P3:00 P4:00 S:0     "
    snprintf(line, sizeof(line), "P3:%02.0f P4:%02.0f S:%.1f",
             p3_ms, p4_ms, s_delay);
    lcd2004_print_row(2, line);

    // Row 3: Status + Mode + BLE indicator
    // "BLOCKED      AUTO  B"
    snprintf(line, sizeof(line), "%-13.13s%4s %c",
             status_text, auto_mode ? "AUTO" : " MAN",
             ble_connected ? 'B' : ' ');
    lcd2004_print_row(3, line);
}

void display_hal_show_status(const char *message) {
    lcd2004_print_row(3, message);
}

void display_hal_weld_fired(void) {
    // Flash backlight as visual feedback
    lcd2004_backlight(false);
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd2004_backlight(true);
}

void display_hal_set_brightness(uint8_t percent) {
    lcd2004_backlight(percent > 0);
}

static void lcd_update_task(void *pvParams) {
    (void)pvParams;
    // Wait for the init screen to show for 1 second before overwriting
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "lcd_update_task running — 4Hz refresh");
    while (1) {
        // Only write to LCD when data has changed (dirty flag)
        if (ui_stub_is_dirty()) {
            ui_stub_refresh_display();
        }
        vTaskDelay(pdMS_TO_TICKS(250));  // 4Hz max refresh — safe for I2C on Core 0
    }
}

void display_hal_start_task(void) {
    xTaskCreatePinnedToCore(lcd_update_task, "lcd_task", 4096, NULL,
                            TASK_UI_PRIORITY, NULL, TASK_UI_CORE);
    ESP_LOGI(TAG, "LCD update task started");
}

#endif // HAS_CHAR_LCD
