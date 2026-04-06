#include "board_config.h"

#if HAS_LVGL

/**
 * Display Module — esp_lcd QSPI + LVGL 9.x
 *
 * JC3248W535: AXS15231B 320×480 portrait → 90° SW rotation → 480×320 landscape
 * JC4827W543: NV3041A   480×272 native landscape → direct flush (no rotation)
 *
 * Common: QSPI bus (CS=45, SCK=47, D0=21, D1=48, D2=40, D3=39), SPI mode 3,
 *         non-blocking flush via DMA, RGB565_SWAPPED color format.
 */

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "display.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#if (BOARD_VARIANT == BOARD_JC4827W543)
#include "esp_lcd_nv3041a.h"
#else
#include "esp_lcd_axs15231b.h"
#endif
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lvgl.h"


static const char *TAG = "Display";

// ============================================================================
// Hardware constants
// ============================================================================

#define QSPI_HOST   SPI2_HOST
#define LCD_BPP     16
#define LCD_LEDC_CH 1

#if (BOARD_VARIANT == BOARD_JC4827W543)
  // NV3041A: native 480×272 landscape — no rotation needed
  #define LCD_H_RES    480
  #define LCD_V_RES    272
  #define DISP_HOR_RES 480
  #define DISP_VER_RES 272
#else
  // AXS15231B: native 320×480 portrait, rotated to landscape by SW
  #define LCD_H_RES    320
  #define LCD_V_RES    480
  #define DISP_HOR_RES 480
  #define DISP_VER_RES 320
#endif

// ============================================================================
// Init Commands — board-specific
// ============================================================================
#if (BOARD_VARIANT != BOARD_JC4827W543)
// AXS15231B Initialization Commands (from official JC3248W535EN reference BSP)
static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xBB, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5}, 8, 0},
    {0xA0,
     (uint8_t[]){0xC0, 0x10, 0x00, 0x02, 0x00, 0x00, 0x04, 0x3F, 0x20, 0x05,
                 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00},
     17, 0},
    {0xA2, (uint8_t[]){0x30, 0x3C, 0x24, 0x14, 0xD0, 0x20, 0xFF, 0xE0,
                       0x40, 0x19, 0x80, 0x80, 0x80, 0x20, 0xf9, 0x10,
                       0x02, 0xff, 0xff, 0xF0, 0x90, 0x01, 0x32, 0xA0,
                       0x91, 0xE0, 0x20, 0x7F, 0xFF, 0x00, 0x5A},
     31, 0},
    {0xD0,
     (uint8_t[]){0xE0, 0x40, 0x51, 0x24, 0x08, 0x05, 0x10, 0x01, 0x20, 0x15,
                 0x42, 0xC2, 0x22, 0x22, 0xAA, 0x03, 0x10, 0x12, 0x60, 0x14,
                 0x1E, 0x51, 0x15, 0x00, 0x8A, 0x20, 0x00, 0x03, 0x3A, 0x12},
     30, 0},
    {0xA3, (uint8_t[]){0xA0, 0x06, 0xAa, 0x00, 0x08, 0x02, 0x0A, 0x04,
                       0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
                       0x04, 0x04, 0x04, 0x00, 0x55, 0x55},
     22, 0},
    {0xC1,
     (uint8_t[]){0x31, 0x04, 0x02, 0x02, 0x71, 0x05, 0x24, 0x55, 0x02, 0x00,
                 0x41, 0x00, 0x53, 0xFF, 0xFF, 0xFF, 0x4F, 0x52, 0x00, 0x4F,
                 0x52, 0x00, 0x45, 0x3B, 0x0B, 0x02, 0x0d, 0x00, 0xFF, 0x40},
     30, 0},
    {0xC3,
     (uint8_t[]){0x00, 0x00, 0x00, 0x50, 0x03, 0x00, 0x00, 0x00, 0x01, 0x80,
                 0x01},
     11, 0},
    {0xC4,
     (uint8_t[]){0x00, 0x24, 0x33, 0x80, 0x00, 0xea, 0x64, 0x32, 0xC8, 0x64,
                 0xC8, 0x32, 0x90, 0x90, 0x11, 0x06, 0xDC, 0xFA, 0x00, 0x00,
                 0x80, 0xFE, 0x10, 0x10, 0x00, 0x0A, 0x0A, 0x44, 0x50},
     29, 0},
    {0xC5, (uint8_t[]){0x18, 0x00, 0x00, 0x03, 0xFE, 0x3A, 0x4A, 0x20,
                       0x30, 0x10, 0x88, 0xDE, 0x0D, 0x08, 0x0F, 0x0F,
                       0x01, 0x3A, 0x4A, 0x20, 0x10, 0x10, 0x00},
     23, 0},
    {0xC6,
     (uint8_t[]){0x05, 0x0A, 0x05, 0x0A, 0x00, 0xE0, 0x2E, 0x0B, 0x12, 0x22,
                 0x12, 0x22, 0x01, 0x03, 0x00, 0x3F, 0x6A, 0x18, 0xC8, 0x22},
     20, 0},
    {0xC7,
     (uint8_t[]){0x50, 0x32, 0x28, 0x00, 0xa2, 0x80, 0x8f, 0x00, 0x80, 0xff,
                 0x07, 0x11, 0x9c, 0x67, 0xff, 0x24, 0x0c, 0x0d, 0x0e, 0x0f},
     20, 0},
    {0xC9, (uint8_t[]){0x33, 0x44, 0x44, 0x01}, 4, 0},
    {0xCF, (uint8_t[]){0x2C, 0x1E, 0x88, 0x58, 0x13, 0x18, 0x56, 0x18, 0x1E,
                       0x68, 0x88, 0x00, 0x65, 0x09, 0x22, 0xC4, 0x0C, 0x77,
                       0x22, 0x44, 0xAA, 0x55, 0x08, 0x08, 0x12, 0xA0, 0x08},
     27, 0},
    {0xD5,
     (uint8_t[]){0x40, 0x8E, 0x8D, 0x01, 0x35, 0x04, 0x92, 0x74, 0x04, 0x92,
                 0x74, 0x04, 0x08, 0x6A, 0x04, 0x46, 0x03, 0x03, 0x03, 0x03,
                 0x82, 0x01, 0x03, 0x00, 0xE0, 0x51, 0xA1, 0x00, 0x00, 0x00},
     30, 0},
    {0xD6,
     (uint8_t[]){0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x93, 0x00,
                 0x01, 0x83, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x03, 0x03,
                 0x03, 0x03, 0x03, 0x03, 0x00, 0x84, 0x00, 0x20, 0x01, 0x00},
     30, 0},
    {0xD7,
     (uint8_t[]){0x03, 0x01, 0x0b, 0x09, 0x0f, 0x0d, 0x1E, 0x1F, 0x18, 0x1d,
                 0x1f, 0x19, 0x40, 0x8E, 0x04, 0x00, 0x20, 0xA0, 0x1F},
     19, 0},
    {0xD8,
     (uint8_t[]){0x02, 0x00, 0x0a, 0x08, 0x0e, 0x0c, 0x1E, 0x1F, 0x18, 0x1d,
                 0x1f, 0x19},
     12, 0},
    {0xD9,
     (uint8_t[]){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
                 0x1F, 0x1F},
     12, 0},
    {0xDD,
     (uint8_t[]){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
                 0x1F, 0x1F},
     12, 0},
    {0xDF, (uint8_t[]){0x44, 0x73, 0x4B, 0x69, 0x00, 0x0A, 0x02, 0x90}, 8, 0},
    {0xE0,
     (uint8_t[]){0x3B, 0x28, 0x10, 0x16, 0x0c, 0x06, 0x11, 0x28, 0x5c, 0x21,
                 0x0D, 0x35, 0x13, 0x2C, 0x33, 0x28, 0x0D},
     17, 0},
    {0xE1,
     (uint8_t[]){0x37, 0x28, 0x10, 0x16, 0x0b, 0x06, 0x11, 0x28, 0x5C, 0x21,
                 0x0D, 0x35, 0x14, 0x2C, 0x33, 0x28, 0x0F},
     17, 0},
    {0xE2,
     (uint8_t[]){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32,
                 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D},
     17, 0},
    {0xE3,
     (uint8_t[]){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32,
                 0x0C, 0x14, 0x14, 0x36, 0x32, 0x2F, 0x0F},
     17, 0},
    {0xE4,
     (uint8_t[]){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E,
                 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D},
     17, 0},
    {0xE5,
     (uint8_t[]){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E,
                 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0F},
     17, 0},
    {0xA4, (uint8_t[]){0x85, 0x85, 0x95, 0x82, 0xAF, 0xAA, 0xAA, 0x80,
                        0x10, 0x30, 0x40, 0x40, 0x20, 0xFF, 0x60, 0x30}, 16, 0},
    {0xA4, (uint8_t[]){0x85, 0x85, 0x95, 0x85}, 4, 0},
    {0xBB, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 0},
    {0x13, (uint8_t[]){0x00}, 0, 0},
    {0x2C, (uint8_t[]){0x00, 0x00, 0x00, 0x00}, 4, 0},
};
#endif // !BOARD_JC4827W543

// ============================================================================
// Static state
// ============================================================================
static esp_lcd_panel_handle_t    s_panel_handle = NULL;
static esp_lcd_panel_io_handle_t s_io_handle    = NULL;
static esp_lcd_touch_handle_t    s_touch_handle = NULL;
static SemaphoreHandle_t         s_lvgl_mutex   = NULL;
static uint8_t                   s_brightness   = 80;

// LVGL display handle — needed by the ISR callback to call flush_ready.
static lv_display_t *s_disp = NULL;

// Draw buffer — full frame in SPIRAM.
// RGB565_SWAPPED = 2 bytes per pixel → 480×320×2 = 307 200 bytes.
// NOTE: lv_color_t in LVGL 9.x is a 3-byte RGB888 struct. The actual pixel
// format in the draw buffer (set by lv_display_set_color_format) is
// RGB565_SWAPPED = 16-bit/2-byte per pixel. We MUST use uint16_t as the
// element type — NEVER lv_color_t — to get correct sizeof and pointer math.
static uint16_t *buf1 = NULL;

#if (BOARD_VARIANT != BOARD_JC4827W543)
// Rotation DMA buffer — AXS15231B only (portrait→landscape SW rotation)
#define ROT_STRIP_W 48
static uint16_t *s_rot_buf = NULL;
#endif

// Semaphore to synchronise DMA completion.
static SemaphoreHandle_t s_trans_sem = NULL;

// ============================================================================
// LVGL Flush Callback
// ============================================================================

#if (BOARD_VARIANT == BOARD_JC4827W543)
// ---- JC4827W543: Direct flush (NV3041A native 480×272 landscape) ----
// No rotation needed — LVGL landscape matches panel native orientation.

static bool panel_trans_done_cb(esp_lcd_panel_io_handle_t io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_trans_sem, &woken);
  return (woken == pdTRUE);
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                          uint8_t *px_map) {
  esp_lcd_panel_handle_t panel = lv_display_get_user_data(disp);

  // NV3041A native landscape: send pixels directly, no rotation.
  int x1 = area->x1, y1 = area->y1;
  int x2 = area->x2 + 1, y2 = area->y2 + 1;

  esp_lcd_panel_draw_bitmap(panel, x1, y1, x2, y2, px_map);

  // Wait for DMA to complete before signaling LVGL.
  xSemaphoreTake(s_trans_sem, portMAX_DELAY);
  lv_display_flush_ready(disp);
}

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t touch_x[1], touch_y[1], touch_strength[1];
  uint8_t touch_cnt = 0;

  esp_lcd_touch_read_data(s_touch_handle);
  bool pressed = esp_lcd_touch_get_coordinates(
      s_touch_handle, touch_x, touch_y, touch_strength, &touch_cnt, 1);

  if (pressed && touch_cnt > 0) {
    data->point.x = touch_x[0];
    data->point.y = touch_y[0];
    data->state   = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

#else // AXS15231B (JC3248W535)
// ---- JC3248W535: 90° CW Software Rotation (portrait→landscape) ----

static bool panel_trans_done_cb(esp_lcd_panel_io_handle_t io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_trans_sem, &woken);
  return (woken == pdTRUE);
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                          uint8_t *px_map) {
  esp_lcd_panel_handle_t panel = lv_display_get_user_data(disp);

  const int ls_w = LCD_V_RES;  // landscape width  = 480
  const int ls_h = LCD_H_RES;  // landscape height = 320
  const int pt_w = LCD_H_RES;  // portrait width   = 320

  const int x1 = area->x1;
  const int x2 = area->x2;
  const uint16_t *src = (const uint16_t *)px_map;

  bool first_strip = true;

  for (int cx = x1; cx <= x2; cx += ROT_STRIP_W) {
    int strip_w = (cx + ROT_STRIP_W <= x2 + 1) ? ROT_STRIP_W : (x2 - cx + 1);

    if (!first_strip) {
      xSemaphoreTake(s_trans_sem, portMAX_DELAY);
    }
    first_strip = false;

    for (int y = 0; y < ls_h; y++) {
      for (int x = 0; x < strip_w; x++) {
        s_rot_buf[x * pt_w + (pt_w - 1 - y)] = src[y * ls_w + (cx + x)];
      }
    }

    esp_lcd_panel_draw_bitmap(panel,
                              0,    cx,
                              pt_w, cx + strip_w,
                              s_rot_buf);
  }

  xSemaphoreTake(s_trans_sem, portMAX_DELAY);
  lv_display_flush_ready(disp);
}

static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t touch_x[1], touch_y[1], touch_strength[1];
  uint8_t touch_cnt = 0;

  // AXS15231B returns I2C NACK when idle (no finger) — normal, suppress it.
  esp_err_t ret = esp_lcd_touch_read_data(s_touch_handle);
  if (ret != ESP_OK) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  bool pressed = esp_lcd_touch_get_coordinates(
      s_touch_handle, touch_x, touch_y, touch_strength, &touch_cnt, 1);

  if (pressed && touch_cnt > 0) {
    data->point.x = touch_x[0];
    data->point.y = touch_y[0];
    data->state   = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}
#endif // BOARD_VARIANT

// ============================================================================
// Display Driver Initialization
// ============================================================================

#if (BOARD_VARIANT != BOARD_JC4827W543)
// AXS15231B touch: portrait→landscape coordinate rotation
static void touch_process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x,
                                      uint16_t *y, uint16_t *strength,
                                      uint8_t *point_num,
                                      uint8_t max_point_num) {
  for (int i = 0; i < *point_num; i++) {
    uint16_t px = x[i];
    uint16_t py = y[i];
    x[i] = py;
    y[i] = (uint16_t)(LCD_H_RES - 1) - px;
  }
  (void)tp; (void)strength; (void)max_point_num;
}
#endif

static void start_backlight(void) {
  ledc_timer_config_t bl_timer = {
      .speed_mode      = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num       = 1,
      .freq_hz         = 5000,
      .clk_cfg         = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&bl_timer);

  ledc_channel_config_t bl_channel = {
      .gpio_num   = PIN_LCD_BL,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel    = LCD_LEDC_CH,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = 1,
      .duty       = 0,
      .hpoint     = 0,
  };
  ledc_channel_config(&bl_channel);
}

void display_init(void) {
  ESP_LOGI(TAG, "Initializing QSPI display (%s)...", BOARD_NAME_STRING);

  // ── 1. SPI/QSPI Bus ────────────────────────────────────────────────────────
  spi_bus_config_t buscfg = {
      .sclk_io_num     = PIN_LCD_CLK,
      .data0_io_num    = PIN_LCD_D0,
      .data1_io_num    = PIN_LCD_D1,
      .data2_io_num    = PIN_LCD_D2,
      .data3_io_num    = PIN_LCD_D3,
      .max_transfer_sz = DISP_HOR_RES * DISP_VER_RES * sizeof(uint16_t),
  };
  ESP_ERROR_CHECK(spi_bus_initialize(QSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // ── 2. Panel IO ────────────────────────────────────────────────────────────
  // SPI mode 3 (CPOL=1, CPHA=1): confirmed by the official reference macro
  // AXS15231B_PANEL_IO_QSPI_CONFIG in esp_lcd_axs15231b.h.
  // trans_queue_depth=10 matches the reference (not 1).
  esp_lcd_panel_io_spi_config_t io_config = {
      .cs_gpio_num        = PIN_LCD_CS,
      .dc_gpio_num        = -1,
      .spi_mode           = 3,
      .pclk_hz            = 40 * 1000 * 1000,
      .trans_queue_depth  = 10,
      .lcd_cmd_bits       = 32,
      .lcd_param_bits     = 8,
      .flags = { .quad_mode = true },
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
      (esp_lcd_spi_bus_handle_t)QSPI_HOST, &io_config, &s_io_handle));

  // ── 3. Panel device ────────────────────────────────────────────────────────
#if (BOARD_VARIANT == BOARD_JC4827W543)
  nv3041a_vendor_config_t vendor_config = {
      .init_cmds      = NULL,  // use built-in NV3041A default init
      .init_cmds_size = 0,
      .flags = { .use_qspi_interface = 1 },
  };
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num  = -1,
      .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel  = LCD_BPP,
      .vendor_config   = (void *)&vendor_config,
  };
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_nv3041a(s_io_handle, &panel_config, &s_panel_handle));
#else
  axs15231b_vendor_config_t vendor_config = {
      .init_cmds      = lcd_init_cmds,
      .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
      .flags = { .use_qspi_interface = 1 },
  };
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num  = -1,
      .rgb_ele_order   = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel  = LCD_BPP,
      .vendor_config   = (void *)&vendor_config,
  };
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_axs15231b(s_io_handle, &panel_config, &s_panel_handle));
#endif

  // ── 4. Reset & hardware init ───────────────────────────────────────────────
  esp_lcd_panel_reset(s_panel_handle);
  esp_lcd_panel_init(s_panel_handle);
  esp_lcd_panel_disp_on_off(s_panel_handle, false);
  vTaskDelay(pdMS_TO_TICKS(20));

  // ── 5. Backlight ───────────────────────────────────────────────────────────
  start_backlight();

  // ── 6. I2C & Touch ─────────────────────────────────────────────────────────
  i2c_config_t i2c_conf = {
      .mode             = I2C_MODE_MASTER,
      .sda_io_num       = PIN_TOUCH_SDA,
      .sda_pullup_en    = GPIO_PULLUP_DISABLE,
      .scl_io_num       = PIN_TOUCH_SCL,
      .scl_pullup_en    = GPIO_PULLUP_DISABLE,
      .master.clk_speed = 400000,
  };
  i2c_param_config(I2C_NUM_0, &i2c_conf);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

  esp_lcd_panel_io_handle_t tp_io_handle = NULL;
#if (BOARD_VARIANT == BOARD_JC4827W543)
  esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
  esp_lcd_new_panel_io_i2c(
      (esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle);

  esp_lcd_touch_config_t tp_cfg = {
      .x_max              = LCD_H_RES,
      .y_max              = LCD_V_RES,
      .rst_gpio_num       = PIN_TOUCH_RST,
      .int_gpio_num       = PIN_TOUCH_INT,
      .levels             = { .reset = 0, .interrupt = 0 },
      .flags              = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
  };
  esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &s_touch_handle);
#else
  esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_AXS15231B_CONFIG();
  esp_lcd_new_panel_io_i2c(
      (esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle);

  esp_lcd_touch_config_t tp_cfg = {
      .x_max              = LCD_H_RES,
      .y_max              = LCD_V_RES,
      .rst_gpio_num       = -1,
      .int_gpio_num       = -1,
      .process_coordinates = touch_process_coordinates,
      .levels             = { .reset = 0, .interrupt = 0 },
      .flags              = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
  };
  esp_lcd_touch_new_i2c_axs15231b(tp_io_handle, &tp_cfg, &s_touch_handle);
#endif

  // ── 7. LVGL init ───────────────────────────────────────────────────────────
  lv_init();
  s_lvgl_mutex = xSemaphoreCreateMutex();

  // DMA semaphore — starts at 0 (unclaimed). ISR gives it on each DMA done.
  // Max count = 1: we always wait for one completion at a time.
  s_trans_sem = xSemaphoreCreateCounting(1, 0);
  ESP_ERROR_CHECK(s_trans_sem == NULL ? ESP_ERR_NO_MEM : ESP_OK);

#if (BOARD_VARIANT != BOARD_JC4827W543)
  // AXS15231B rotation buffer: DMA-capable internal SRAM
  s_rot_buf = (uint16_t *)heap_caps_malloc(
      ROT_STRIP_W * LCD_H_RES * sizeof(uint16_t),
      MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  ESP_ERROR_CHECK(s_rot_buf == NULL ? ESP_ERR_NO_MEM : ESP_OK);
#endif
  // Use uint16_t sizing: lv_color_t in LVGL 9.x is 3 bytes (RGB888 struct);
  // using sizeof(lv_color_t) here would allocate 50% more memory than needed
  // AND lv_display_set_buffers would report the wrong pixel count to LVGL.
  size_t buf_size = DISP_HOR_RES * DISP_VER_RES * sizeof(uint16_t);
  buf1 = (uint16_t *)heap_caps_malloc(buf_size,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  ESP_ERROR_CHECK(buf1 == NULL ? ESP_ERR_NO_MEM : ESP_OK);

  // LVGL sees a 480×320 landscape display. No lv_display_set_rotation().
  // Rotation is handled entirely in lvgl_flush_cb (manual SW rotation).
  s_disp = lv_display_create(DISP_HOR_RES, DISP_VER_RES);
  lv_display_set_flush_cb(s_disp, lvgl_flush_cb);
  lv_display_set_buffers(s_disp, buf1, NULL, buf_size,
                         LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_user_data(s_disp, s_panel_handle);

  // RGB565 byte-swap so LVGL's little-endian buffer matches the panel.
  lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565_SWAPPED);

  // Register DMA-done callback — ISR gives s_trans_sem on each transfer.
  const esp_lcd_panel_io_callbacks_t cbs = {
      .on_color_trans_done = panel_trans_done_cb,
  };
  esp_lcd_panel_io_register_event_callbacks(s_io_handle, &cbs, NULL);

  // Touch input device
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, lvgl_touch_read_cb);

  // ── Indev tuning (LVGL 9.x runtime API) ────────────────────────────────────
  // LVGL 9.x hardcodes LV_INDEV_DEF_* inside lv_indev.c and ignores lv_conf.h
  // for these values. The only way to override is via the API after lv_indev_create().

  // Poll rate: 10 ms (100 Hz). Default = LV_DEF_REFR_PERIOD = 16 ms.
  lv_timer_set_period(lv_indev_get_read_timer(indev), 10);

  // Long-press: 300 ms instead of 400 ms — snappier button response.
  lv_indev_set_long_press_time(indev, 300);

  // Long-press repeat: 80 ms instead of 100 ms — faster auto-repeat on +/- buttons.
  lv_indev_set_long_press_repeat_time(indev, 80);

  // Scroll limit: 5 px instead of 10 px.
  // Lowers the distance a finger must travel before LVGL switches from
  // "tap" to "scroll" mode. Prevents short taps on small buttons from
  // being silently eaten by the scroll detector.
  lv_indev_set_scroll_limit(indev, 5);

#if (BOARD_VARIANT == BOARD_JC4827W543)
  ESP_LOGI(TAG, "Display initialized: %dx%d native landscape (NV3041A + GT911)", DISP_HOR_RES, DISP_VER_RES);
#else
  ESP_LOGI(TAG, "Display initialized: %dx%d landscape, 90 CW SW-rotation (AXS15231B)", DISP_HOR_RES, DISP_VER_RES);
#endif
}

// ============================================================================
// Brightness
// ============================================================================

void display_set_brightness(uint8_t percent) {
  if (percent > 100) percent = 100;
  s_brightness = percent;
  uint32_t duty = (1023 * percent) / 100;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH);
}

uint8_t display_get_brightness(void) { return s_brightness; }

void display_enable_backlight(void) {
  // Called once by ui_task() after the first LVGL frame is flushed.
  // Ramps backlight to the saved brightness so the panel is clean before
  // becoming visible — eliminates GRAM garbage visible at power-on.
  display_set_brightness(s_brightness);
  ESP_LOGI(TAG, "Backlight enabled at %d%%", s_brightness);
}

// ============================================================================
// LVGL lock / unlock (called from other tasks via ui_ API)
// ============================================================================

bool display_lock(uint32_t timeout_ms) {
  if (!s_lvgl_mutex) return false;
  return xSemaphoreTake(s_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void display_unlock(void) {
  xSemaphoreGive(s_lvgl_mutex);
}

// ============================================================================
// Legacy stubs — kept for API compatibility with ui.c / main.c
// ============================================================================

// display_set_flush_task() was used by the old blocking-flush pattern.
// It is no longer needed (ISR now calls lv_display_flush_ready directly)
// but is kept as a no-op so callers don't need updating.
void display_set_flush_task(void *task_handle) {
  (void)task_handle; // no-op: non-blocking flush does not need this
}

void *display_get_io_handle(void) { return (void *)s_io_handle; }

#endif // HAS_LVGL
