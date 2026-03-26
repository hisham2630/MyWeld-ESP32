/**
 * MyWeld ESP32-S3 — ST7567S 128×64 COG Graphic LCD Driver (I2C)
 *
 * Framebuffer-based driver for ST7567S controller over I2C.
 * Only compiled when DISPLAY_TYPE == DISPLAY_COG_12864.
 *
 * Wiring: SDA → GPIO8, SCL → GPIO9, VCC → 3.3V, GND → GND
 */

#include "board_config.h"

#if HAS_COG_LCD

#include "st7567s.h"
#include "config.h"
#include "ui.h"
#include "encoder.h"
#include "settings.h"
#include "audio_hal.h"

#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "st7567s";

// ============================================================================
// ST7567S Commands
// ============================================================================
#define CMD_DISPLAY_OFF      0xAE
#define CMD_DISPLAY_ON       0xAF
#define CMD_SET_START_LINE   0x40
#define CMD_SET_PAGE         0xB0
#define CMD_SET_COL_HI       0x10
#define CMD_SET_COL_LO       0x00
#define CMD_SEG_NORMAL       0xA0
#define CMD_SEG_REVERSE      0xA1
#define CMD_COM_NORMAL       0xC0
#define CMD_COM_REVERSE      0xC8
#define CMD_BIAS_1_9         0xA2
#define CMD_POWER_CTRL       0x2F
#define CMD_REG_RATIO        0x24
#define CMD_SET_EV           0x81
#define CMD_DISPLAY_NORMAL   0xA6
#define CMD_ALL_ON_RESUME    0xA4
#define CMD_RESET            0xE2
#define CMD_BOOSTER_ON       0x2C
#define CMD_REG_ON           0x2E
#define CMD_FOLLOWER_ON      0x2F

// I2C control bytes
#define I2C_CMD              0x00  // Co=0, A0=0 → command
#define I2C_DATA             0x40  // Co=0, A0=1 → data

// ============================================================================
// Framebuffer & State
// ============================================================================
static uint8_t s_fb[COG_FB_SIZE];           // 1024-byte framebuffer
static uint8_t s_page_dirty;                // Bitmask: which pages need flush
static uint8_t s_dev_addr = 0x3F;           // Detected I2C address

// ============================================================================
// I2C Helpers
// ============================================================================

static esp_err_t i2c_write_cmd(uint8_t cmd) {
    uint8_t buf[2] = {I2C_CMD, cmd};
    return i2c_master_write_to_device(LCD_I2C_NUM, s_dev_addr,
                                       buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_write_data(const uint8_t *data, size_t len) {
    // I2C can send up to ~128 bytes per transaction; page = 128 bytes
    uint8_t buf[129];
    buf[0] = I2C_DATA;
    if (len > 128) len = 128;
    memcpy(buf + 1, data, len);
    return i2c_master_write_to_device(LCD_I2C_NUM, s_dev_addr,
                                       buf, len + 1, pdMS_TO_TICKS(200));
}

// ============================================================================
// 5×7 Font (ASCII 32–126, compact)
// ============================================================================

static const uint8_t FONT_5X7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x00,0x00,0x5F,0x00,0x00}, // 33 '!'
    {0x00,0x07,0x00,0x07,0x00}, // 34 '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 '$'
    {0x23,0x13,0x08,0x64,0x62}, // 37 '%'
    {0x36,0x49,0x55,0x22,0x50}, // 38 '&'
    {0x00,0x05,0x03,0x00,0x00}, // 39 '''
    {0x00,0x1C,0x22,0x41,0x00}, // 40 '('
    {0x00,0x41,0x22,0x1C,0x00}, // 41 ')'
    {0x08,0x2A,0x1C,0x2A,0x08}, // 42 '*'
    {0x08,0x08,0x3E,0x08,0x08}, // 43 '+'
    {0x00,0x50,0x30,0x00,0x00}, // 44 ','
    {0x08,0x08,0x08,0x08,0x08}, // 45 '-'
    {0x00,0x60,0x60,0x00,0x00}, // 46 '.'
    {0x20,0x10,0x08,0x04,0x02}, // 47 '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 '0'
    {0x00,0x42,0x7F,0x40,0x00}, // 49 '1'
    {0x42,0x61,0x51,0x49,0x46}, // 50 '2'
    {0x21,0x41,0x45,0x4B,0x31}, // 51 '3'
    {0x18,0x14,0x12,0x7F,0x10}, // 52 '4'
    {0x27,0x45,0x45,0x45,0x39}, // 53 '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 '6'
    {0x01,0x71,0x09,0x05,0x03}, // 55 '7'
    {0x36,0x49,0x49,0x49,0x36}, // 56 '8'
    {0x06,0x49,0x49,0x29,0x1E}, // 57 '9'
    {0x00,0x36,0x36,0x00,0x00}, // 58 ':'
    {0x00,0x56,0x36,0x00,0x00}, // 59 ';'
    {0x00,0x08,0x14,0x22,0x41}, // 60 '<'
    {0x14,0x14,0x14,0x14,0x14}, // 61 '='
    {0x41,0x22,0x14,0x08,0x00}, // 62 '>'
    {0x02,0x01,0x51,0x09,0x06}, // 63 '?'
    {0x32,0x49,0x79,0x41,0x3E}, // 64 '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 66 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 67 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 69 'E'
    {0x7F,0x09,0x09,0x01,0x01}, // 70 'F'
    {0x3E,0x41,0x41,0x51,0x32}, // 71 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 73 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 74 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 75 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 76 'L'
    {0x7F,0x02,0x04,0x02,0x7F}, // 77 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 80 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 82 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 83 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 84 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 'V'
    {0x7F,0x20,0x18,0x20,0x7F}, // 87 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 88 'X'
    {0x03,0x04,0x78,0x04,0x03}, // 89 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 90 'Z'
    {0x00,0x00,0x7F,0x41,0x41}, // 91 '['
    {0x02,0x04,0x08,0x10,0x20}, // 92 '\'
    {0x41,0x41,0x7F,0x00,0x00}, // 93 ']'
    {0x04,0x02,0x01,0x02,0x04}, // 94 '^'
    {0x40,0x40,0x40,0x40,0x40}, // 95 '_'
    {0x00,0x01,0x02,0x04,0x00}, // 96 '`'
    {0x20,0x54,0x54,0x54,0x78}, // 97 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 98 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 99 'c'
    {0x38,0x44,0x44,0x48,0x7F}, //100 'd'
    {0x38,0x54,0x54,0x54,0x18}, //101 'e'
    {0x08,0x7E,0x09,0x01,0x02}, //102 'f'
    {0x08,0x14,0x54,0x54,0x3C}, //103 'g'
    {0x7F,0x08,0x04,0x04,0x78}, //104 'h'
    {0x00,0x44,0x7D,0x40,0x00}, //105 'i'
    {0x20,0x40,0x44,0x3D,0x00}, //106 'j'
    {0x00,0x7F,0x10,0x28,0x44}, //107 'k'
    {0x00,0x41,0x7F,0x40,0x00}, //108 'l'
    {0x7C,0x04,0x18,0x04,0x78}, //109 'm'
    {0x7C,0x08,0x04,0x04,0x78}, //110 'n'
    {0x38,0x44,0x44,0x44,0x38}, //111 'o'
    {0x7C,0x14,0x14,0x14,0x08}, //112 'p'
    {0x08,0x14,0x14,0x18,0x7C}, //113 'q'
    {0x7C,0x08,0x04,0x04,0x08}, //114 'r'
    {0x48,0x54,0x54,0x54,0x20}, //115 's'
    {0x04,0x3F,0x44,0x40,0x20}, //116 't'
    {0x3C,0x40,0x40,0x20,0x7C}, //117 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, //118 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, //119 'w'
    {0x44,0x28,0x10,0x28,0x44}, //120 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, //121 'y'
    {0x44,0x64,0x54,0x4C,0x44}, //122 'z'
    {0x00,0x08,0x36,0x41,0x00}, //123 '{'
    {0x00,0x00,0x7F,0x00,0x00}, //124 '|'
    {0x00,0x41,0x36,0x08,0x00}, //125 '}'
    {0x08,0x08,0x2A,0x1C,0x08}, //126 '~'
};

// ============================================================================
// Core Init & Flush
// ============================================================================

void st7567s_init(void) {
    i2c_config_t i2c_cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_LCD_SDA,
        .scl_io_num       = PIN_LCD_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,  // 400kHz fast I2C (ST7567S supports it)
    };
    i2c_param_config(LCD_I2C_NUM, &i2c_cfg);
    i2c_driver_install(LCD_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);

    ESP_LOGI(TAG, "I2C initialized (SDA=IO%d, SCL=IO%d, 400kHz)",
             PIN_LCD_SDA, PIN_LCD_SCL);

    // I2C bus scan
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
        ESP_LOGE(TAG, "No I2C devices found! Check wiring.");
        return;
    }
    // Use found address (auto-detect)
    if (found >= 1) {
        s_dev_addr = found_addr;
        ESP_LOGI(TAG, "Using I2C address 0x%02X", s_dev_addr);
    }

    // ST7567S initialization sequence (from datasheet p.42)
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_write_cmd(CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_write_cmd(CMD_BIAS_1_9);       // 1/9 bias
    i2c_write_cmd(CMD_SEG_NORMAL);     // SEG normal direction
    i2c_write_cmd(CMD_COM_REVERSE);    // COM reverse (top-to-bottom scan)
    i2c_write_cmd(CMD_REG_RATIO);      // Regulation ratio = 5.0
    i2c_write_cmd(CMD_SET_EV);         // Set electronic volume...
    i2c_write_cmd(0x28);               // ...value = 40/63 (mid contrast)
    i2c_write_cmd(CMD_BOOSTER_ON);     // Booster circuit ON
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_write_cmd(CMD_REG_ON);         // Voltage regulator ON
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_write_cmd(CMD_FOLLOWER_ON);    // Voltage follower ON
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_write_cmd(CMD_ALL_ON_RESUME);  // Display from RAM
    i2c_write_cmd(CMD_DISPLAY_NORMAL); // Normal (not inverted)
    i2c_write_cmd(CMD_SET_START_LINE); // Start line = 0
    i2c_write_cmd(CMD_DISPLAY_ON);     // Display ON

    // Clear framebuffer and flush
    st7567s_clear(0);
    s_page_dirty = 0xFF;  // All pages dirty
    st7567s_flush();

    ESP_LOGI(TAG, "ST7567S 128x64 initialized (addr=0x%02X)", s_dev_addr);
}

void st7567s_clear(uint8_t color) {
    memset(s_fb, color ? 0xFF : 0x00, COG_FB_SIZE);
    s_page_dirty = 0xFF;
}

void st7567s_set_contrast(uint8_t value) {
    if (value > 63) value = 63;
    i2c_write_cmd(CMD_SET_EV);
    i2c_write_cmd(value);
}

void st7567s_flush(void) {
    for (uint8_t page = 0; page < COG_PAGES; page++) {
        if (!(s_page_dirty & (1 << page))) continue;
        i2c_write_cmd(CMD_SET_PAGE | page);
        i2c_write_cmd(CMD_SET_COL_HI | 0);
        i2c_write_cmd(CMD_SET_COL_LO | 0);
        i2c_write_data(&s_fb[page * COG_WIDTH], COG_WIDTH);
    }
    s_page_dirty = 0;
}

// ============================================================================
// Drawing Primitives
// ============================================================================

void st7567s_pixel(int16_t x, int16_t y, uint8_t color) {
    if (x < 0 || x >= COG_WIDTH || y < 0 || y >= COG_HEIGHT) return;
    uint8_t page = y >> 3;
    uint8_t bit  = y & 7;
    uint16_t idx = page * COG_WIDTH + x;
    if (color)
        s_fb[idx] |= (1 << bit);
    else
        s_fb[idx] &= ~(1 << bit);
    s_page_dirty |= (1 << page);
}

void st7567s_hline(int16_t x, int16_t y, int16_t w, uint8_t color) {
    for (int16_t i = 0; i < w; i++) st7567s_pixel(x + i, y, color);
}

void st7567s_vline(int16_t x, int16_t y, int16_t h, uint8_t color) {
    for (int16_t i = 0; i < h; i++) st7567s_pixel(x, y + i, color);
}

void st7567s_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    st7567s_hline(x, y, w, color);
    st7567s_hline(x, y + h - 1, w, color);
    st7567s_vline(x, y, h, color);
    st7567s_vline(x + w - 1, y, h, color);
}

void st7567s_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    for (int16_t j = 0; j < h; j++)
        st7567s_hline(x, y + j, w, color);
}

// ============================================================================
// Text Rendering
// ============================================================================

void st7567s_text(int16_t x, int16_t y, const char *text, uint8_t color) {
    while (*text) {
        uint8_t ch = (uint8_t)*text;
        if (ch < 32 || ch > 126) ch = '?';
        const uint8_t *glyph = FONT_5X7[ch - 32];
        for (int col = 0; col < 5; col++) {
            uint8_t bits = glyph[col];
            for (int row = 0; row < 7; row++) {
                st7567s_pixel(x + col, y + row, (bits & (1 << row)) ? color : !color);
            }
        }
        // 1px spacing between characters
        for (int row = 0; row < 7; row++)
            st7567s_pixel(x + 5, y + row, !color);  // gap = background
        x += 6;
        text++;
    }
}

void st7567s_text_large(int16_t x, int16_t y, const char *text, uint8_t color) {
    while (*text) {
        uint8_t ch = (uint8_t)*text;
        if (ch < 32 || ch > 126) ch = '?';
        const uint8_t *glyph = FONT_5X7[ch - 32];
        for (int col = 0; col < 5; col++) {
            uint8_t bits = glyph[col];
            for (int row = 0; row < 7; row++) {
                uint8_t c = (bits & (1 << row)) ? color : !color;
                // 2× scale: each source pixel → 2×2 block
                st7567s_pixel(x + col*2,     y + row*2,     c);
                st7567s_pixel(x + col*2 + 1, y + row*2,     c);
                st7567s_pixel(x + col*2,     y + row*2 + 1, c);
                st7567s_pixel(x + col*2 + 1, y + row*2 + 1, c);
            }
        }
        // 2px spacing
        for (int row = 0; row < 14; row++) {
            st7567s_pixel(x + 10, y + row, !color);
            st7567s_pixel(x + 11, y + row, !color);
        }
        x += 12;
        text++;
    }
}

int16_t st7567s_text_width(const char *text) {
    int16_t len = (int16_t)strlen(text);
    return len > 0 ? len * 6 - 1 : 0;
}

// ============================================================================
// Encoder Navigation (mirrors lcd2004 pattern)
// ============================================================================

typedef enum { COG_ENC_NAV, COG_ENC_EDIT } cog_enc_mode_t;

#define COG_FOCUS_P1    0
#define COG_FOCUS_T     1
#define COG_FOCUS_P2    2
#define COG_FOCUS_P3    3
#define COG_FOCUS_P4    4
#define COG_FOCUS_S     5
#define COG_FOCUS_MODE  6
#define COG_FOCUS_COUNT 7

typedef struct {
    float min_val, max_val, step;
} cog_focus_meta_t;

static const cog_focus_meta_t s_focus_meta[COG_FOCUS_COUNT] = {
    {PULSE_MIN_MS, PULSE_MAX_MS, PULSE_STEP_MS},  // P1
    {PAUSE_MIN_MS, PAUSE_MAX_MS, PAUSE_STEP_MS},  // T
    {PULSE_MIN_MS, PULSE_MAX_MS, PULSE_STEP_MS},  // P2
    {PULSE_MIN_MS, PULSE_MAX_MS, PULSE_STEP_MS},  // P3
    {PULSE_MIN_MS, PULSE_MAX_MS, PULSE_STEP_MS},  // P4
    {S_VALUE_MIN,  S_VALUE_MAX,  S_VALUE_STEP},   // S
    {0, 0, 0},                                     // MODE
};

static int            s_enc_focus = -1;
static cog_enc_mode_t s_enc_mode  = COG_ENC_NAV;
static int            s_blink_tick = 0;
static bool           s_blink_visible = true;

// Settings screen state
typedef enum { COG_SCREEN_MAIN, COG_SCREEN_SETTINGS } cog_screen_t;
static cog_screen_t s_screen = COG_SCREEN_MAIN;
static int s_stng_focus = 0;
static uint8_t s_stng_preset_idx = 0;
#define STNG_TIMEOUT_US (30 * 1000000LL)
static int64_t s_stng_last_activity = 0;
static bool s_was_preset_mode = false;

static bool cog_is_preset_mode(void) {
    return g_settings.active_preset != PRESET_USER_DEFINED;
}

static int cog_focus_count(void) {
    return g_settings.auto_mode ? COG_FOCUS_COUNT : (COG_FOCUS_COUNT - 1);
}

static int cog_logical_to_real(int logical) {
    if (!g_settings.auto_mode && logical >= COG_FOCUS_S)
        return logical + 1;
    return logical;
}

static float *cog_get_value_ptr(int real) {
    switch (real) {
        case COG_FOCUS_P1: return &g_settings.p1;
        case COG_FOCUS_T:  return &g_settings.t;
        case COG_FOCUS_P2: return &g_settings.p2;
        case COG_FOCUS_P3: return &g_settings.p3;
        case COG_FOCUS_P4: return &g_settings.p4;
        case COG_FOCUS_S:  return &g_settings.s_value;
        default: return NULL;
    }
}

static void cog_handle_encoder(encoder_event_t evt) {
    int max_items = cog_focus_count();
    if (s_enc_focus < 0) {
        if (evt == ENC_EVENT_CW || evt == ENC_EVENT_CCW) {
            s_enc_mode = COG_ENC_NAV;
            s_enc_focus = 0;
            s_blink_tick = 0;
            s_blink_visible = true;
            audio_play_beep();
        }
        return;
    }
    if (s_enc_mode == COG_ENC_NAV) {
        if (evt == ENC_EVENT_CW) {
            s_enc_focus = (s_enc_focus + 1) % max_items;
            s_blink_tick = 0; s_blink_visible = true; audio_play_beep();
        } else if (evt == ENC_EVENT_CCW) {
            s_enc_focus = (s_enc_focus - 1 + max_items) % max_items;
            s_blink_tick = 0; s_blink_visible = true; audio_play_beep();
        } else if (evt == ENC_EVENT_PRESS) {
            int real = cog_logical_to_real(s_enc_focus);
            if (real == COG_FOCUS_MODE) {
                g_settings.auto_mode = !g_settings.auto_mode;
                if (s_enc_focus >= cog_focus_count())
                    s_enc_focus = cog_focus_count() - 1;
                settings_save();
            } else {
                s_enc_mode = COG_ENC_EDIT;
                s_blink_visible = true;
            }
            audio_play_beep();
        }
    } else {
        if (evt == ENC_EVENT_PRESS) {
            s_enc_mode = COG_ENC_NAV;
            settings_save();
            s_blink_tick = 0; audio_play_beep();
        } else {
            int dir = (evt == ENC_EVENT_CW) ? 1 : -1;
            int real = cog_logical_to_real(s_enc_focus);
            float *val = cog_get_value_ptr(real);
            if (val) {
                const cog_focus_meta_t *m = &s_focus_meta[real];
                *val += dir * m->step;
                if (*val > m->max_val) *val = m->max_val;
                if (*val < m->min_val) *val = m->min_val;
            }
            audio_play_beep();
        }
    }
}

static void cog_handle_encoder_settings(encoder_event_t evt) {
    s_stng_last_activity = esp_timer_get_time();
    if (evt == ENC_EVENT_LONG_PRESS) {
        s_screen = COG_SCREEN_MAIN;
        s_enc_mode = COG_ENC_NAV;
        s_enc_focus = -1;
        audio_play_beep();
        return;
    }
    // Simple settings: volume / mute / preset
    if (s_enc_mode == COG_ENC_NAV) {
        if (evt == ENC_EVENT_CW) {
            s_stng_focus = (s_stng_focus + 1) % 3;
            audio_play_beep();
        } else if (evt == ENC_EVENT_CCW) {
            s_stng_focus = (s_stng_focus + 2) % 3;
            audio_play_beep();
        } else if (evt == ENC_EVENT_PRESS) {
            s_enc_mode = COG_ENC_EDIT;
            if (s_stng_focus == 2) {
                s_stng_preset_idx = g_settings.active_preset;
                if (s_stng_preset_idx >= MAX_PRESETS) s_stng_preset_idx = 0;
            }
            audio_play_beep();
        }
    } else {
        if (evt == ENC_EVENT_PRESS) {
            if (s_stng_focus == 2 && s_stng_preset_idx < MAX_PRESETS) {
                settings_load_preset(s_stng_preset_idx);
                audio_play_ready();
                s_screen = COG_SCREEN_MAIN;
                s_enc_focus = -1;
            }
            s_enc_mode = COG_ENC_NAV;
            settings_save();
            audio_play_beep();
        } else {
            int dir = (evt == ENC_EVENT_CW) ? 1 : -1;
            if (s_stng_focus == 0) {
                int v = (int)g_settings.volume + dir * 10;
                if (v > 100) v = 100;
                if (v < 0) v = 0;
                g_settings.volume = (uint8_t)v;
                audio_set_volume(g_settings.volume);
            } else if (s_stng_focus == 1) {
                audio_play_beep();
                g_settings.sound_on = !g_settings.sound_on;
                audio_set_muted(!g_settings.sound_on);
                return;
            } else if (s_stng_focus == 2) {
                int p = (int)s_stng_preset_idx + dir;
                if (p >= MAX_PRESETS) p = 0;
                if (p < 0) p = MAX_PRESETS - 1;
                s_stng_preset_idx = (uint8_t)p;
            }
            audio_play_beep();
        }
    }
}

// ============================================================================
// Graphical Dashboard Rendering
// ============================================================================

static void cog_render_dashboard(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
) {
    char buf[24];
    bool is_pr = cog_is_preset_mode();

    // Determine blink state for focused item
    int blink_real = -1;
    bool blink_blank = false;
    if (s_enc_focus >= 0 && s_enc_mode == COG_ENC_NAV && !is_pr) {
        blink_real = cog_logical_to_real(s_enc_focus);
        blink_blank = !s_blink_visible;
    }

    // === Row 0 (y=0): Voltage + charge bar + percentage ===
    snprintf(buf, sizeof(buf), "%4.1fV", voltage_v);
    st7567s_text(0, 0, buf, 1);

    // Charge bar: x=32..95 (64px wide), y=1..7
    st7567s_rect(32, 0, 64, 8, 1);
    int bar_w = (charge_pct * 62) / 100;
    if (bar_w > 62) bar_w = 62;
    st7567s_fill_rect(33, 1, bar_w, 6, 1);
    // Clear remaining bar area
    if (bar_w < 62) st7567s_fill_rect(33 + bar_w, 1, 62 - bar_w, 6, 0);

    snprintf(buf, sizeof(buf), "%3d%%", charge_pct);
    st7567s_text(98, 0, buf, 1);

    // BLE indicator
    st7567s_text(122, 0, ble_connected ? "B" : " ", 1);

    // === Separator line ===
    st7567s_hline(0, 9, 128, 1);

    if (is_pr) {
        // Preset mode: show preset name large
        const char *pname = (g_settings.active_preset < MAX_PRESETS)
            ? g_settings.presets[g_settings.active_preset].name : "???";
        // Center the name
        int16_t tw = st7567s_text_width(pname);
        int16_t tx = (128 - tw) / 2;
        if (tx < 0) tx = 0;
        st7567s_fill_rect(0, 11, 128, 18, 0);  // Clear area
        st7567s_text(tx, 15, pname, 1);
    } else {
        // === Row 1 (y=11): P1 T P2 ===
        snprintf(buf, sizeof(buf), "P1:%02.0f", p1_ms);
        st7567s_text(0, 11, (blink_real == 0 && blink_blank) ? "     " : buf, 1);
        snprintf(buf, sizeof(buf), "T:%02.0f", t_ms);
        st7567s_text(38, 11, (blink_real == 1 && blink_blank) ? "    " : buf, 1);
        snprintf(buf, sizeof(buf), "P2:%02.0f", p2_ms);
        st7567s_text(68, 11, (blink_real == 2 && blink_blank) ? "     " : buf, 1);

        // === Row 2 (y=20): P3 P4 S ===
        snprintf(buf, sizeof(buf), "P3:%02.0f", p3_ms);
        st7567s_text(0, 20, (blink_real == 3 && blink_blank) ? "     " : buf, 1);
        snprintf(buf, sizeof(buf), "P4:%02.0f", p4_ms);
        st7567s_text(38, 20, (blink_real == 4 && blink_blank) ? "     " : buf, 1);
        if (auto_mode) {
            snprintf(buf, sizeof(buf), "S:%.1f", s_delay);
            st7567s_text(78, 20, (blink_real == 5 && blink_blank) ? "     " : buf, 1);
        } else {
            st7567s_text(78, 20, "     ", 1);  // Clear S area in MAN mode
        }
    }

    // === Separator ===
    st7567s_hline(0, 29, 128, 1);

    // === Status area (y=31): Large centered text ===
    st7567s_fill_rect(0, 31, 128, 14, 0);  // Clear status area
    int16_t stw = (int16_t)strlen(status_text) * 12;
    int16_t stx = (128 - stw) / 2;
    if (stx < 0) stx = 0;
    st7567s_text_large(stx, 31, status_text, 1);

    // === Separator ===
    st7567s_hline(0, 45, 128, 1);

    // === Bottom row (y=47): Mode + badge ===
    const char *mode_str = auto_mode ? "AUTO" : " MAN";
    if (blink_real == COG_FOCUS_MODE && blink_blank)
        mode_str = "    ";
    st7567s_text(0, 47, mode_str, 1);

    st7567s_text(30, 47, is_pr ? "PR" : "UD", 1);

    // Mini voltage graph bar (y=55..63, full width)
    st7567s_hline(0, 55, 128, 1);
    st7567s_fill_rect(0, 56, 128, 8, 0);
    int gw = (charge_pct * 128) / 100;
    if (gw > 128) gw = 128;
    st7567s_fill_rect(0, 56, gw, 8, 1);
}

static void cog_render_settings(void) {
    char buf[24];

    st7567s_fill_rect(0, 0, 128, 64, 0);  // Clear all

    st7567s_text(0, 0, "== SETTINGS ==", 1);
    st7567s_hline(0, 8, 128, 1);

    // Volume
    char c1 = (s_stng_focus == 0) ? '>' : ' ';
    snprintf(buf, sizeof(buf), "%cVol: %3d%%", c1, g_settings.volume);
    st7567s_text(0, 12, buf, 1);

    // Sound
    char c2 = (s_stng_focus == 1) ? '>' : ' ';
    snprintf(buf, sizeof(buf), "%cSnd: %3s", c2, g_settings.sound_on ? " ON" : "OFF");
    st7567s_text(0, 24, buf, 1);

    // Preset
    char c3 = (s_stng_focus == 2) ? '>' : ' ';
    const char *pn;
    if (s_enc_mode == COG_ENC_EDIT && s_stng_focus == 2) {
        pn = (s_stng_preset_idx < MAX_PRESETS)
            ? g_settings.presets[s_stng_preset_idx].name : "???";
    } else {
        pn = (g_settings.active_preset < MAX_PRESETS)
            ? g_settings.presets[g_settings.active_preset].name : "User";
    }
    snprintf(buf, sizeof(buf), "%c%.18s", c3, pn);
    st7567s_text(0, 36, buf, 1);

    st7567s_hline(0, 48, 128, 1);
    st7567s_text(0, 52, "Long=Exit  Press=OK", 1);
}

// ============================================================================
// Display HAL Implementation
// ============================================================================

void display_hal_init(void) {
    st7567s_init();
}

void display_hal_show_splash(void) {
    // Phase 1: Welcome (large centered text)
    st7567s_clear(0);
    int16_t tw1 = (int16_t)strlen(SPLASH_MSG_WELCOME) * 12;  // 2x font = 12px/char
    int16_t tx1 = (COG_WIDTH - tw1) / 2;
    if (tx1 < 0) tx1 = 0;
    st7567s_text_large(tx1, 22, SPLASH_MSG_WELCOME, 1);
    s_page_dirty = 0xFF;
    st7567s_flush();
    vTaskDelay(pdMS_TO_TICKS(SPLASH_WELCOME_MS));

    // Phase 2: App name (large) + version + credits
    st7567s_clear(0);
    char version_str[24];
    snprintf(version_str, sizeof(version_str), "v%s", FW_VERSION_STRING);

    int16_t tw2 = (int16_t)strlen(SPLASH_MSG_APP_NAME) * 12;
    int16_t tx2 = (COG_WIDTH - tw2) / 2;
    if (tx2 < 0) tx2 = 0;
    st7567s_text_large(tx2, 8, SPLASH_MSG_APP_NAME, 1);

    int16_t tw3 = st7567s_text_width(version_str);
    int16_t tx3 = (COG_WIDTH - tw3) / 2;
    if (tx3 < 0) tx3 = 0;
    st7567s_text(tx3, 28, version_str, 1);

    int16_t tw4 = st7567s_text_width(SPLASH_MSG_CREDITS);
    int16_t tx4 = (COG_WIDTH - tw4) / 2;
    if (tx4 < 0) tx4 = 0;
    st7567s_text(tx4, 42, SPLASH_MSG_CREDITS, 1);

    s_page_dirty = 0xFF;
    st7567s_flush();
    vTaskDelay(pdMS_TO_TICKS(SPLASH_VERSION_MS));

    // Clear for dashboard
    st7567s_clear(0);
    st7567s_flush();
}

void display_hal_update(
    float voltage_v, uint8_t charge_pct,
    float p1_ms, float t_ms, float p2_ms, float p3_ms, float p4_ms,
    bool auto_mode, float s_delay,
    const char *status_text, bool ble_connected
) {
    st7567s_clear(0);
    cog_render_dashboard(voltage_v, charge_pct, p1_ms, t_ms, p2_ms,
                         p3_ms, p4_ms, auto_mode, s_delay,
                         status_text, ble_connected);
    st7567s_flush();
}

void display_hal_show_status(const char *message) {
    // Quick status flash on bottom rows
    st7567s_fill_rect(0, 31, 128, 14, 0);
    int16_t tw = (int16_t)strlen(message) * 12;
    int16_t tx = (128 - tw) / 2;
    if (tx < 0) { tx = 0; }
    st7567s_text_large(tx, 31, message, 1);
    st7567s_flush();
}

void display_hal_weld_fired(void) {
    // Flash: invert display briefly
    i2c_write_cmd(0xA7);  // Inverse display
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_write_cmd(CMD_DISPLAY_NORMAL);
}

void display_hal_set_brightness(uint8_t percent) {
    // Map 0–100% to contrast 0–50 (ST7567S range)
    uint8_t ev = (percent * 50) / 100;
    st7567s_set_contrast(ev);
}

static void cog_update_task(void *pvParams) {
    (void)pvParams;
    ESP_LOGI(TAG, "cog_update_task running — 8Hz refresh");

    while (1) {
        // Poll encoder
        encoder_event_t enc_evt;
        while (encoder_poll(&enc_evt)) {
            if (s_screen == COG_SCREEN_MAIN) {
                if (enc_evt == ENC_EVENT_LONG_PRESS) {
                    s_screen = COG_SCREEN_SETTINGS;
                    s_stng_focus = 0;
                    s_enc_mode = COG_ENC_NAV;
                    s_enc_focus = -1;
                    s_stng_last_activity = esp_timer_get_time();
                    audio_play_beep();
                } else {
                    cog_handle_encoder(enc_evt);
                }
            } else {
                cog_handle_encoder_settings(enc_evt);
            }
        }

        // Settings timeout
        if (s_screen == COG_SCREEN_SETTINGS) {
            if ((esp_timer_get_time() - s_stng_last_activity) > STNG_TIMEOUT_US) {
                s_screen = COG_SCREEN_MAIN;
                s_enc_mode = COG_ENC_NAV;
                s_enc_focus = -1;
            }
        }

        // UD/PR transition
        bool is_pr_now = cog_is_preset_mode();
        if (is_pr_now != s_was_preset_mode) {
            s_enc_focus = -1;
            s_enc_mode = COG_ENC_NAV;
            s_was_preset_mode = is_pr_now;
        }

        // Blink
        if (s_enc_focus >= 0) {
            s_blink_tick++;
            s_blink_visible = (s_blink_tick % 2) == 0;
        }

        // Render
        if (s_screen == COG_SCREEN_SETTINGS) {
            st7567s_clear(0);
            cog_render_settings();
            st7567s_flush();
        } else if (ui_stub_is_dirty() || s_enc_focus >= 0) {
            ui_stub_refresh_display();
        }

        vTaskDelay(pdMS_TO_TICKS(125));  // 8Hz
    }
}

void display_hal_start_task(void) {
    xTaskCreatePinnedToCore(cog_update_task, "cog_task", 4096, NULL,
                            TASK_UI_PRIORITY, NULL, TASK_UI_CORE);
    ESP_LOGI(TAG, "COG display task started");
}

#endif // HAS_COG_LCD
