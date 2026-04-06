/**
 * NV3041A QSPI Panel Driver + GT911 I2C Touch Driver
 * Board: JC4827W543 (Guition ESP32-S3, 480×272 IPS)
 *
 * Panel driver modeled after esp_lcd_axs15231b.c — same QSPI bus interface,
 * same opcode format (0x02 for CMD, 0x32 for COLOR), different init commands.
 *
 * Init sequence sourced from Arduino_GFX Arduino_NV3041A.h (moononournation).
 *
 * GT911 touch driver implements the standard GT911 I2C register protocol:
 *   - Status register: 0x814E (bit7=ready, bits3:0=point count)
 *   - Point data: 0x8150 (8 bytes per point)
 *   - Clear status: write 0x00 to 0x814E after reading
 */

#include "board_config.h"

#if HAS_LVGL && (BOARD_VARIANT == BOARD_JC4827W543)

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>

#include "esp_lcd_nv3041a.h"

static const char *TAG = "lcd_panel.nv3041a";

/* QSPI opcodes — identical to AXS15231B */
#define LCD_OPCODE_WRITE_CMD   (0x02ULL)
#define LCD_OPCODE_READ_CMD    (0x0BULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

/* GT911 registers */
#define GT911_REG_STATUS      0x814E
#define GT911_REG_POINT_BASE  0x8150
#define GT911_POINT_SIZE      8     /* bytes per touch point */
#define GT911_MAX_TOUCH       1     /* we only need single-touch */

/* ========================================================================= */
/* NV3041A Panel Driver                                                       */
/* ========================================================================= */

static esp_err_t panel_nv3041a_del(esp_lcd_panel_t *panel);
static esp_err_t panel_nv3041a_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_nv3041a_init(esp_lcd_panel_t *panel);
static esp_err_t panel_nv3041a_draw_bitmap(esp_lcd_panel_t *panel,
                                            int x_start, int y_start,
                                            int x_end, int y_end,
                                            const void *color_data);
static esp_err_t panel_nv3041a_invert_color(esp_lcd_panel_t *panel, bool invert);
static esp_err_t panel_nv3041a_mirror(esp_lcd_panel_t *panel, bool mx, bool my);
static esp_err_t panel_nv3041a_swap_xy(esp_lcd_panel_t *panel, bool swap);
static esp_err_t panel_nv3041a_set_gap(esp_lcd_panel_t *panel, int x, int y);
static esp_err_t panel_nv3041a_disp_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val;
    uint8_t colmod_val;
    const nv3041a_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        unsigned int use_qspi_interface : 1;
        unsigned int reset_level : 1;
    } flags;
} nv3041a_panel_t;

/* QSPI command helpers — identical to AXS15231B */
static esp_err_t nv_tx_param(nv3041a_panel_t *nv, esp_lcd_panel_io_handle_t io,
                              int lcd_cmd, const void *param, size_t param_size) {
    if (nv->flags.use_qspi_interface) {
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
    }
    return esp_lcd_panel_io_tx_param(io, lcd_cmd, param, param_size);
}

static esp_err_t nv_tx_color(nv3041a_panel_t *nv, esp_lcd_panel_io_handle_t io,
                              int lcd_cmd, const void *param, size_t param_size) {
    if (nv->flags.use_qspi_interface) {
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_COLOR << 24;
    }
    return esp_lcd_panel_io_tx_color(io, lcd_cmd, param, param_size);
}

/* ---- NV3041A init command table (from Arduino_GFX Arduino_NV3041A.h) ---- */
static const nv3041a_lcd_init_cmd_t nv3041a_vendor_init_default[] = {
    /* Unlock NV3041A extended registers */
    {0xFF, (uint8_t[]){0xA5}, 1, 0},

    /* MADCTL: row/col exchange + mirror for landscape 480×272 */
    {0x36, (uint8_t[]){0xC0}, 1, 0},

    /* Color format: 0x01 = RGB565 (NV3041A-specific encoding) */
    {0x3A, (uint8_t[]){0x01}, 1, 0},

    /* Bus width: 0x03 = 16-bit */
    {0x41, (uint8_t[]){0x03}, 1, 0},

    /* VBP / VFP */
    {0x44, (uint8_t[]){0x15}, 1, 0},
    {0x45, (uint8_t[]){0x15}, 1, 0},

    /* vdds_trim */
    {0x7D, (uint8_t[]){0x03}, 1, 0},

    /* Power supply configuration */
    {0xC1, (uint8_t[]){0xBB}, 1, 0},
    {0xC2, (uint8_t[]){0x05}, 1, 0},
    {0xC3, (uint8_t[]){0x10}, 1, 0},
    {0xC6, (uint8_t[]){0x3E}, 1, 0},
    {0xC7, (uint8_t[]){0x25}, 1, 0},
    {0xC8, (uint8_t[]){0x11}, 1, 0},

    /* Voltage settings */
    {0x7A, (uint8_t[]){0x5F}, 1, 0},  /* user_vgsp   */
    {0x6F, (uint8_t[]){0x44}, 1, 0},  /* user_gvdd   */
    {0x78, (uint8_t[]){0x70}, 1, 0},  /* user_gvcl   */
    {0xC9, (uint8_t[]){0x00}, 1, 0},
    {0x67, (uint8_t[]){0x21}, 1, 0},

    /* Gate start/end (odd & even) */
    {0x51, (uint8_t[]){0x0A}, 1, 0},
    {0x52, (uint8_t[]){0x76}, 1, 0},
    {0x53, (uint8_t[]){0x0A}, 1, 0},
    {0x54, (uint8_t[]){0x76}, 1, 0},

    /* Source timing */
    {0x46, (uint8_t[]){0x0A}, 1, 0},
    {0x47, (uint8_t[]){0x2A}, 1, 0},
    {0x48, (uint8_t[]){0x0A}, 1, 0},
    {0x49, (uint8_t[]){0x1A}, 1, 0},
    {0x56, (uint8_t[]){0x43}, 1, 0},
    {0x57, (uint8_t[]){0x42}, 1, 0},
    {0x58, (uint8_t[]){0x3C}, 1, 0},
    {0x59, (uint8_t[]){0x64}, 1, 0},
    {0x5A, (uint8_t[]){0x41}, 1, 0},
    {0x5B, (uint8_t[]){0x3C}, 1, 0},
    {0x5C, (uint8_t[]){0x02}, 1, 0},
    {0x5D, (uint8_t[]){0x3C}, 1, 0},
    {0x5E, (uint8_t[]){0x1F}, 1, 0},
    {0x60, (uint8_t[]){0x80}, 1, 0},
    {0x61, (uint8_t[]){0x3F}, 1, 0},
    {0x62, (uint8_t[]){0x21}, 1, 0},
    {0x63, (uint8_t[]){0x07}, 1, 0},
    {0x64, (uint8_t[]){0xE0}, 1, 0},
    {0x65, (uint8_t[]){0x02}, 1, 0},

    /* AVDD/AVCL/VGH mux timing */
    {0xCA, (uint8_t[]){0x20}, 1, 0},
    {0xCB, (uint8_t[]){0x52}, 1, 0},
    {0xCC, (uint8_t[]){0x10}, 1, 0},
    {0xCD, (uint8_t[]){0x42}, 1, 0},
    {0xD0, (uint8_t[]){0x20}, 1, 0},
    {0xD1, (uint8_t[]){0x52}, 1, 0},
    {0xD2, (uint8_t[]){0x10}, 1, 0},
    {0xD3, (uint8_t[]){0x42}, 1, 0},
    {0xD4, (uint8_t[]){0x0A}, 1, 0},
    {0xD5, (uint8_t[]){0x32}, 1, 0},

    /* Gamma — positive (VRP/PRP/PKP) */
    {0x80, (uint8_t[]){0x00}, 1, 0},
    {0x81, (uint8_t[]){0x07}, 1, 0},
    {0x82, (uint8_t[]){0x02}, 1, 0},
    {0x83, (uint8_t[]){0x37}, 1, 0},
    {0x84, (uint8_t[]){0x35}, 1, 0},
    {0x85, (uint8_t[]){0x3F}, 1, 0},
    {0x86, (uint8_t[]){0x11}, 1, 0},
    {0x87, (uint8_t[]){0x27}, 1, 0},
    {0x88, (uint8_t[]){0x0B}, 1, 0},
    {0x89, (uint8_t[]){0x14}, 1, 0},
    {0x8A, (uint8_t[]){0x1A}, 1, 0},
    {0x8B, (uint8_t[]){0x0A}, 1, 0},
    {0x8C, (uint8_t[]){0x14}, 1, 0},
    {0x8D, (uint8_t[]){0x17}, 1, 0},
    {0x8E, (uint8_t[]){0x16}, 1, 0},
    {0x8F, (uint8_t[]){0x1B}, 1, 0},
    {0x90, (uint8_t[]){0x04}, 1, 0},
    {0x91, (uint8_t[]){0x0A}, 1, 0},
    {0x92, (uint8_t[]){0x16}, 1, 0},

    /* Gamma — negative (VRN/PRN/PKN) */
    {0xA0, (uint8_t[]){0x00}, 1, 0},
    {0xA1, (uint8_t[]){0x06}, 1, 0},
    {0xA2, (uint8_t[]){0x01}, 1, 0},
    {0xA3, (uint8_t[]){0x37}, 1, 0},
    {0xA4, (uint8_t[]){0x35}, 1, 0},
    {0xA5, (uint8_t[]){0x3F}, 1, 0},
    {0xA6, (uint8_t[]){0x10}, 1, 0},
    {0xA7, (uint8_t[]){0x27}, 1, 0},
    {0xA8, (uint8_t[]){0x0B}, 1, 0},
    {0xA9, (uint8_t[]){0x14}, 1, 0},
    {0xAA, (uint8_t[]){0x1A}, 1, 0},
    {0xAB, (uint8_t[]){0x0A}, 1, 0},
    {0xAC, (uint8_t[]){0x08}, 1, 0},
    {0xAD, (uint8_t[]){0x07}, 1, 0},
    {0xAE, (uint8_t[]){0x06}, 1, 0},
    {0xAF, (uint8_t[]){0x07}, 1, 0},
    {0xB0, (uint8_t[]){0x04}, 1, 0},
    {0xB1, (uint8_t[]){0x0A}, 1, 0},
    {0xB2, (uint8_t[]){0x15}, 1, 0},

    /* Lock extended registers */
    {0xFF, (uint8_t[]){0x00}, 1, 0},

    /* Sleep Out */
    {0x11, NULL, 0, 120},

    /* Display On */
    {0x29, NULL, 0, 100},
};

esp_err_t
esp_lcd_new_panel_nv3041a(const esp_lcd_panel_io_handle_t io,
                           const esp_lcd_panel_dev_config_t *panel_dev_config,
                           esp_lcd_panel_handle_t *ret_panel) {
    esp_err_t ret = ESP_OK;
    nv3041a_panel_t *nv = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG,
                      err, TAG, "invalid argument");
    nv = calloc(1, sizeof(nv3041a_panel_t));
    ESP_GOTO_ON_FALSE(nv, ESP_ERR_NO_MEM, err, TAG, "no mem for nv3041a panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG,
                          "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->color_space) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        nv->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        nv->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                          "unsupported RGB element order");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16:
        nv->colmod_val = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18:
        nv->colmod_val = 0x66;
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG,
                          "unsupported pixel width");
        break;
    }

    nv->io = io;
    nv->fb_bits_per_pixel = fb_bits_per_pixel;
    nv->reset_gpio_num = panel_dev_config->reset_gpio_num;
    nv->flags.reset_level = panel_dev_config->flags.reset_active_high;
    if (panel_dev_config->vendor_config) {
        nv->init_cmds = ((nv3041a_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds;
        nv->init_cmds_size = ((nv3041a_vendor_config_t *)panel_dev_config->vendor_config)->init_cmds_size;
        nv->flags.use_qspi_interface = ((nv3041a_vendor_config_t *)panel_dev_config->vendor_config)->flags.use_qspi_interface;
    }
    nv->base.del = panel_nv3041a_del;
    nv->base.reset = panel_nv3041a_reset;
    nv->base.init = panel_nv3041a_init;
    nv->base.draw_bitmap = panel_nv3041a_draw_bitmap;
    nv->base.invert_color = panel_nv3041a_invert_color;
    nv->base.set_gap = panel_nv3041a_set_gap;
    nv->base.mirror = panel_nv3041a_mirror;
    nv->base.swap_xy = panel_nv3041a_swap_xy;
    nv->base.disp_on_off = panel_nv3041a_disp_off;
    *ret_panel = &(nv->base);
    ESP_LOGI(TAG, "NV3041A panel created (480x272 QSPI)");

    return ESP_OK;

err:
    if (nv) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(nv);
    }
    return ret;
}

static esp_err_t panel_nv3041a_del(esp_lcd_panel_t *panel) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    if (nv->reset_gpio_num >= 0) {
        gpio_reset_pin(nv->reset_gpio_num);
    }
    free(nv);
    return ESP_OK;
}

static esp_err_t panel_nv3041a_reset(esp_lcd_panel_t *panel) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv->io;

    if (nv->reset_gpio_num >= 0) {
        gpio_set_level(nv->reset_gpio_num, !nv->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(nv->reset_gpio_num, nv->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(nv->reset_gpio_num, !nv->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else {
        nv_tx_param(nv, io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    return ESP_OK;
}

static esp_err_t panel_nv3041a_init(esp_lcd_panel_t *panel) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    esp_lcd_panel_io_handle_t io = nv->io;

    const nv3041a_lcd_init_cmd_t *init_cmds = nv->init_cmds;
    uint16_t init_cmds_size = nv->init_cmds_size;
    if (!init_cmds) {
        init_cmds = nv3041a_vendor_init_default;
        init_cmds_size = sizeof(nv3041a_vendor_init_default) / sizeof(nv3041a_vendor_init_default[0]);
    }

    for (int i = 0; i < init_cmds_size; i++) {
        ESP_RETURN_ON_ERROR(nv_tx_param(nv, io, init_cmds[i].cmd,
                                         init_cmds[i].data, init_cmds[i].data_bytes),
                            TAG, "send command 0x%02X failed", init_cmds[i].cmd);
        if (init_cmds[i].delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
        }
    }
    ESP_LOGI(TAG, "NV3041A init commands sent (%d cmds)", init_cmds_size);

    return ESP_OK;
}

static esp_err_t panel_nv3041a_draw_bitmap(esp_lcd_panel_t *panel,
                                            int x_start, int y_start,
                                            int x_end, int y_end,
                                            const void *color_data) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end));
    esp_lcd_panel_io_handle_t io = nv->io;

    x_start += nv->x_gap;
    x_end += nv->x_gap;
    y_start += nv->y_gap;
    y_end += nv->y_gap;

    /* CASET */
    nv_tx_param(nv, io, LCD_CMD_CASET,
                (uint8_t[]){
                    (x_start >> 8) & 0xFF, x_start & 0xFF,
                    ((x_end - 1) >> 8) & 0xFF, (x_end - 1) & 0xFF,
                }, 4);

    /* RASET */
    nv_tx_param(nv, io, LCD_CMD_RASET,
                (uint8_t[]){
                    (y_start >> 8) & 0xFF, y_start & 0xFF,
                    ((y_end - 1) >> 8) & 0xFF, (y_end - 1) & 0xFF,
                }, 4);

    /* RAMWR — pixel data */
    size_t len = (x_end - x_start) * (y_end - y_start) * nv->fb_bits_per_pixel / 8;
    nv_tx_color(nv, io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_nv3041a_invert_color(esp_lcd_panel_t *panel, bool invert) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    nv_tx_param(nv, nv->io, invert ? LCD_CMD_INVON : LCD_CMD_INVOFF, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_nv3041a_mirror(esp_lcd_panel_t *panel, bool mx, bool my) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    if (mx) nv->madctl_val |= LCD_CMD_MX_BIT; else nv->madctl_val &= ~LCD_CMD_MX_BIT;
    if (my) nv->madctl_val |= LCD_CMD_MY_BIT; else nv->madctl_val &= ~LCD_CMD_MY_BIT;
    nv_tx_param(nv, nv->io, LCD_CMD_MADCTL, (uint8_t[]){nv->madctl_val}, 1);
    return ESP_OK;
}

static esp_err_t panel_nv3041a_swap_xy(esp_lcd_panel_t *panel, bool swap) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    if (swap) nv->madctl_val |= LCD_CMD_MV_BIT; else nv->madctl_val &= ~LCD_CMD_MV_BIT;
    nv_tx_param(nv, nv->io, LCD_CMD_MADCTL, (uint8_t[]){nv->madctl_val}, 1);
    return ESP_OK;
}

static esp_err_t panel_nv3041a_set_gap(esp_lcd_panel_t *panel, int x, int y) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    nv->x_gap = x;
    nv->y_gap = y;
    return ESP_OK;
}

static esp_err_t panel_nv3041a_disp_off(esp_lcd_panel_t *panel, bool off) {
    nv3041a_panel_t *nv = __containerof(panel, nv3041a_panel_t, base);
    nv_tx_param(nv, nv->io, off ? LCD_CMD_DISPOFF : LCD_CMD_DISPON, NULL, 0);
    return ESP_OK;
}

/* ========================================================================= */
/* GT911 Touch Driver                                                         */
/* ========================================================================= */

static esp_err_t touch_gt911_read_data(esp_lcd_touch_handle_t tp);
static bool touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x,
                                uint16_t *y, uint16_t *strength,
                                uint8_t *point_num, uint8_t max_point_num);
static esp_err_t touch_gt911_del(esp_lcd_touch_handle_t tp);
static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_gt911(const esp_lcd_panel_io_handle_t io,
                                       const esp_lcd_touch_config_t *config,
                                       esp_lcd_touch_handle_t *tp) {
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "Invalid io");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Invalid config");
    ESP_RETURN_ON_FALSE(tp, ESP_ERR_INVALID_ARG, TAG, "Invalid touch handle");

    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t gt911 = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(gt911, ESP_ERR_NO_MEM, err, TAG, "Touch handle malloc failed");

    gt911->io = io;
    gt911->read_data = touch_gt911_read_data;
    gt911->get_xy = touch_gt911_get_xy;
    gt911->del = touch_gt911_del;
    gt911->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&gt911->config, config, sizeof(esp_lcd_touch_config_t));

    /* Configure interrupt pin */
    if (gt911->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_NEGEDGE,
            .pin_bit_mask = BIT64(gt911->config.int_gpio_num),
        };
        ESP_GOTO_ON_ERROR(gpio_config(&int_gpio_config), err, TAG,
                          "GPIO intr config failed");
    }

    /* Configure reset pin */
    if (gt911->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(gt911->config.rst_gpio_num),
        };
        ESP_GOTO_ON_ERROR(gpio_config(&rst_gpio_config), err, TAG,
                          "GPIO reset config failed");
    }

    /* Reset controller — also sets I2C address via INT pin state */
    ESP_GOTO_ON_ERROR(touch_gt911_reset(gt911), err, TAG, "GT911 reset failed");

    *tp = gt911;
    ESP_LOGI(TAG, "GT911 touch initialized (addr 0x%02X)",
             ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS);
    return ESP_OK;

err:
    if (gt911) {
        touch_gt911_del(gt911);
    }
    return ret;
}

/**
 * GT911 reset sequence that sets I2C address to 0x5D:
 *   1. INT pin LOW, RST pin LOW → 10ms
 *   2. INT pin LOW, RST pin HIGH → 50ms
 *   3. INT pin release (input) → 50ms
 *
 * The INT pin state during RST rising edge determines the address:
 *   INT=LOW  → address 0x5D
 *   INT=HIGH → address 0x14
 */
static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp) {
    if (tp->config.rst_gpio_num == GPIO_NUM_NC) {
        return ESP_OK;  /* No reset pin, skip */
    }

    gpio_num_t int_pin = tp->config.int_gpio_num;
    gpio_num_t rst_pin = tp->config.rst_gpio_num;

    /* Drive INT pin LOW to select address 0x5D */
    if (int_pin != GPIO_NUM_NC) {
        gpio_set_direction(int_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(int_pin, 0);
    }

    /* RST LOW */
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* RST HIGH (INT still LOW → addr 0x5D) */
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Release INT pin — switch back to input */
    if (int_pin != GPIO_NUM_NC) {
        gpio_set_direction(int_pin, GPIO_MODE_INPUT);
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "GT911 reset complete (RST=IO%d, INT=IO%d)", rst_pin, int_pin);
    return ESP_OK;
}

static esp_err_t touch_gt911_read_data(esp_lcd_touch_handle_t tp) {
    uint8_t status = 0;
    uint8_t point_data[GT911_POINT_SIZE];

    /* Read status register (0x814E) */
    esp_err_t ret = esp_lcd_panel_io_rx_param(tp->io, GT911_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Check buffer ready flag (bit 7) */
    if (!(status & 0x80)) {
        return ESP_OK;  /* No new data */
    }

    uint8_t num_points = status & 0x0F;

    if (num_points > 0 && num_points <= GT911_MAX_TOUCH) {
        /* Read point data (0x8150) */
        ret = esp_lcd_panel_io_rx_param(tp->io, GT911_REG_POINT_BASE,
                                         point_data, GT911_POINT_SIZE);
        if (ret != ESP_OK) {
            goto clear_status;
        }

        portENTER_CRITICAL(&tp->data.lock);
        tp->data.points = 1;
        /* GT911 point format: x_lo, x_hi, y_lo, y_hi, size_lo, size_hi, -, - */
        tp->data.coords[0].x = point_data[0] | (point_data[1] << 8);
        tp->data.coords[0].y = point_data[2] | (point_data[3] << 8);
        tp->data.coords[0].strength = point_data[4] | (point_data[5] << 8);
        portEXIT_CRITICAL(&tp->data.lock);
    }

clear_status:
    /* Clear status register — mandatory, GT911 won't update until cleared */
    {
        uint8_t clear = 0x00;
        esp_lcd_panel_io_tx_param(tp->io, GT911_REG_STATUS, &clear, 1);
    }

    return ESP_OK;
}

static bool touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x,
                                uint16_t *y, uint16_t *strength,
                                uint8_t *point_num, uint8_t max_point_num) {
    portENTER_CRITICAL(&tp->data.lock);
    *point_num = (tp->data.points > max_point_num) ? max_point_num : tp->data.points;
    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;
        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }
    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t touch_gt911_del(esp_lcd_touch_handle_t tp) {
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }
    free(tp);
    return ESP_OK;
}

#endif /* HAS_LVGL && BOARD_JC4827W543 */
