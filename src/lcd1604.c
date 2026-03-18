/**
 * MyWeld ESP32-S3 — 4×16 Character LCD Driver (I2C, PCF8574)
 *
 * Drives an HD44780-compatible character LCD via PCF8574 I2C backpack.
 * Only compiled when DISPLAY_TYPE == DISPLAY_LCD_1604.
 *
 * LCD Layout (4 rows × 16 cols):
 *   Row 0: "5.7V  ██████ 95%"    ← Voltage + charge bar + percentage
 *   Row 1: "P1:05 T:00 P2:00"   ← Pulse parameters
 *   Row 2: "P3:00 P4:00  MAN"   ← More pulses + mode
 *   Row 3: ">>> READY <<<  B"   ← Status + BLE indicator
 *
 * Wiring: SDA → GPIO8, SCL → GPIO9, VCC → 3.3V, GND → GND
 */

#include "board_config.h"

#if HAS_CHAR_LCD

#include "lcd1604.h"
#include "config.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "lcd1604";

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

// Row start addresses for a 4×16 display
static const uint8_t ROW_ADDR[4] = {0x00, 0x40, 0x10, 0x50};

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

void lcd1604_init(void) {
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

    // HD44780 initialization sequence (4-bit mode)
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for LCD power-up

    // Send 0x30 three times to ensure 8-bit mode, then switch to 4-bit
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(1));  // 4-bit mode

    lcd_command(LCD_CMD_FUNCTION);    // 4-bit, 2-line, 5×8
    lcd_command(LCD_CMD_DISPLAY_ON);  // Display ON
    lcd_command(LCD_CMD_CLEAR);       // Clear
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_command(LCD_CMD_ENTRY_MODE);  // Increment, no shift

    ESP_LOGI(TAG, "LCD 4x16 initialized (I2C addr=0x%02X, SDA=IO%d, SCL=IO%d)",
             LCD_I2C_ADDR, PIN_LCD_SDA, PIN_LCD_SCL);
}

void lcd1604_clear(void) {
    lcd_command(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd1604_set_cursor(uint8_t row, uint8_t col) {
    if (row > 3) row = 3;
    if (col > 15) col = 15;
    lcd_command(LCD_CMD_SET_DDRAM | (ROW_ADDR[row] + col));
}

void lcd1604_print(const char *text) {
    while (*text) {
        lcd_data((uint8_t)*text++);
    }
}

void lcd1604_print_row(uint8_t row, const char *text) {
    lcd1604_set_cursor(row, 0);
    // Pad to 16 chars to clear any previous content
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16.16s", text);
    lcd1604_print(buf);
}

void lcd1604_backlight(bool on) {
    s_backlight = on;
    lcd_i2c_write_byte(0);  // Refresh backlight state
}

// ============================================================================
// Display HAL interface implementation
// ============================================================================

void display_hal_init(void) {
    lcd1604_init();
}

void display_hal_update(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
) {
    char line[17];  // 16 chars + null

    // Row 0: Voltage + charge bar + percentage
    // "5.7V ██████  95%"
    int bars = (charge_pct * 6) / 100;  // 6 bar chars max
    char bar_str[7] = "      ";
    for (int i = 0; i < bars && i < 6; i++) bar_str[i] = 0xFF;  // Full block
    snprintf(line, sizeof(line), "%4.1fV %s%3d%%", voltage_v, bar_str, charge_pct);
    lcd1604_print_row(0, line);

    // Row 1: Pulse parameters
    // "P1:05 T:00 P2:00"
    snprintf(line, sizeof(line), "P1:%02.0f T:%02.0f P2:%02.0f",
             p1_ms, t_ms, p2_ms);
    lcd1604_print_row(1, line);

    // Row 2: More pulses + mode
    // "P3:00 P4:00  MAN"
    snprintf(line, sizeof(line), "P3:%02.0f P4:%02.0f %s",
             p3_ms, p4_ms, auto_mode ? "AUTO" : " MAN");
    lcd1604_print_row(2, line);

    // Row 3: Status + BLE indicator
    // ">>> READY <<<  B"
    snprintf(line, sizeof(line), "%-14.14s %c",
             status_text, ble_connected ? 'B' : ' ');
    lcd1604_print_row(3, line);
}

void display_hal_show_status(const char *message) {
    lcd1604_print_row(3, message);
}

void display_hal_weld_fired(void) {
    // Flash backlight as visual feedback
    lcd1604_backlight(false);
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd1604_backlight(true);
}

void display_hal_set_brightness(uint8_t percent) {
    lcd1604_backlight(percent > 0);
}

static void lcd_update_task(void *pvParams) {
    (void)pvParams;
    while (1) {
        // The actual display update is done by display_hal_update()
        // called from the welding/status loop. This task just provides
        // a heartbeat — not strictly needed for char LCD.
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void display_hal_start_task(void) {
    xTaskCreatePinnedToCore(lcd_update_task, "lcd_task", 2048, NULL,
                            TASK_UI_PRIORITY, NULL, TASK_UI_CORE);
    ESP_LOGI(TAG, "LCD update task started");
}

#endif // HAS_CHAR_LCD
