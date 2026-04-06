/**
 * @file esp_lcd_nv3041a.h
 * @brief ESP LCD & Touch: NV3041A (QSPI TFT) + GT911 (capacitive touch)
 *
 * Board: JC4827W543 (Guition ESP32-S3, 480×272 IPS)
 *
 * Panel driver follows the same esp_lcd interface as AXS15231B,
 * sharing the identical QSPI bus configuration (CS=45, SCK=47,
 * D0=21, D1=48, D2=40, D3=39, SPI mode 3).
 *
 * Touch driver implements GT911 I2C protocol for the capacitive
 * touch controller (I2C addr 0x5D, SCL=4, SDA=8, RST=38, INT=3).
 */

#pragma once

#include "esp_lcd_touch.h"
#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/* NV3041A Panel Driver                                                       */
/* ========================================================================= */

/**
 * @brief LCD panel initialization command entry (same format as AXS15231B).
 */
typedef struct {
    int cmd;
    const void *data;
    size_t data_bytes;
    unsigned int delay_ms;
} nv3041a_lcd_init_cmd_t;

/**
 * @brief LCD panel vendor configuration.
 */
typedef struct {
    const nv3041a_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        unsigned int use_qspi_interface: 1;
    } flags;
} nv3041a_vendor_config_t;

/**
 * @brief Create LCD panel for NV3041A.
 */
esp_err_t esp_lcd_new_panel_nv3041a(const esp_lcd_panel_io_handle_t io,
                                     const esp_lcd_panel_dev_config_t *panel_dev_config,
                                     esp_lcd_panel_handle_t *ret_panel);

/* ========================================================================= */
/* GT911 Touch Driver                                                         */
/* ========================================================================= */

/** GT911 I2C address (address depends on INT pin state during reset) */
#define ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS      (0x5D)
#define ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_ALT  (0x14)

/**
 * @brief Touch IO configuration for GT911.
 *
 * GT911 uses 16-bit register addresses. The esp_lcd_panel_io_i2c driver
 * sends the register address as the "command" (lcd_cmd_bits = 16).
 */
#define ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG()                \
    {                                                       \
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,     \
        .control_phase_bytes = 1,                           \
        .dc_bit_offset = 0,                                 \
        .lcd_cmd_bits = 16,                                 \
        .flags =                                            \
        {                                                   \
            .disable_control_phase = 1,                     \
        }                                                   \
    }

/**
 * @brief Create a new GT911 touch driver.
 *
 * @note I2C communication should be initialized before calling this.
 *
 * @param io   LCD panel IO handle (I2C)
 * @param config Touch configuration
 * @param tp   Output touch handle
 * @return ESP_OK on success
 */
esp_err_t esp_lcd_touch_new_i2c_gt911(const esp_lcd_panel_io_handle_t io,
                                       const esp_lcd_touch_config_t *config,
                                       esp_lcd_touch_handle_t *tp);

#ifdef __cplusplus
}
#endif
