/**
 * LVGL UI Module — Full Dashboard & Settings
 *
 * Main Screen: Status bar, P1/T/P2 cards, voltage bar, graph, counter, status
 * Settings Screen: Brightness, sound, mode, presets, theme, calibration, about
 *
 * Uses LVGL 9.x API.
 *
 * Thread Safety:
 *   LVGL is NOT thread-safe. All lv_* calls MUST happen inside ui_task.
 *   External tasks (adc_task, welding_task) post ui_msg_t structs to
 *   s_ui_queue; ui_task drains the queue before calling lv_timer_handler.
 */

#include "ui.h"
#include "audio.h"
#include "ble_serial.h"
#include "display.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "settings.h"
#include "welding.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "UI";

// ============================================================================
// UI Message Queue (the ONLY safe way to update LVGL from other tasks)
// ============================================================================

typedef enum {
    UI_MSG_VOLTAGE,
    UI_MSG_WELD_STATE,
    UI_MSG_WELD_COUNT,
    UI_MSG_GRAPH_POINT,
    UI_MSG_REFRESH_PARAMS,          // Re-sync all param labels from g_settings (BLE write)
    UI_MSG_REBOOT_COUNTDOWN,        // Show factory-reset countdown then reboot
} ui_msg_type_t;

typedef struct {
    ui_msg_type_t type;
    union {
        float    voltage;           // UI_MSG_VOLTAGE, UI_MSG_GRAPH_POINT
        uint8_t  weld_state;        // UI_MSG_WELD_STATE
        struct {
            uint32_t session;
            uint32_t total;
        } weld_count;               // UI_MSG_WELD_COUNT
    };
} ui_msg_t;

// Queue depth: 16 slots is plenty for 100ms ADC + 10ms UI cadence.
#define UI_QUEUE_DEPTH 16
static QueueHandle_t s_ui_queue = NULL;

// ============================================================================
// Color Palette
// ============================================================================

// Dark theme colors
#define COLOR_BG_DARK lv_color_hex(0x1A1A2E)
#define COLOR_CARD_DARK lv_color_hex(0x16213E)
#define COLOR_ACCENT lv_color_hex(0x0F3460)
#define COLOR_PRIMARY lv_color_hex(0xE94560)
#define COLOR_GREEN lv_color_hex(0x00D68F)
#define COLOR_YELLOW lv_color_hex(0xFFBE0B)
#define COLOR_RED lv_color_hex(0xFF006E)
#define COLOR_BLUE lv_color_hex(0x3A86FF)
#define COLOR_TEXT_LIGHT lv_color_hex(0xEAEAEA)
#define COLOR_TEXT_DIM lv_color_hex(0x888888)

// Light theme colors
#define COLOR_BG_LIGHT lv_color_hex(0xF0F2F5)
#define COLOR_CARD_LIGHT lv_color_hex(0xFFFFFF)
#define COLOR_TEXT_DARK lv_color_hex(0x1A1A2E)

// ============================================================================
// Screen & Widget References
// ============================================================================

static lv_obj_t *scr_main = NULL;     // Main dashboard
static lv_obj_t *scr_settings = NULL; // Settings page

// Countdown overlay (factory reset reboot)
static lv_obj_t    *scr_countdown   = NULL;
static lv_obj_t    *lbl_countdown   = NULL;
static lv_timer_t  *tmr_countdown   = NULL;
static int          s_countdown_val = 3;

// Main screen widgets
static lv_obj_t *lbl_voltage = NULL;
static lv_obj_t *bar_voltage = NULL;
static lv_obj_t *lbl_voltage_pct = NULL;
static lv_obj_t *lbl_mode = NULL;
static lv_obj_t *lbl_status = NULL;
static lv_obj_t *lbl_weld_count = NULL;
static lv_obj_t *lbl_preset = NULL;
static lv_obj_t *chart_voltage = NULL;
static lv_chart_series_t *chart_series = NULL;
static lv_obj_t *volt_container = NULL;  // promoted to global for mode-toggle repositioning

// Parameter cards
typedef struct {
  lv_obj_t *card;
  lv_obj_t *label_name;
  lv_obj_t *label_value;
  lv_obj_t *btn_minus;
  lv_obj_t *btn_plus;
  float *value_ptr; // Points to g_settings.p1, .t, .p2, or .s_value
  float min_val;
  float max_val;
  float step;
  const char *unit;
} param_card_t;

static param_card_t card_p1, card_t, card_p2, card_s;

// ============================================================================
// Helpers
// ============================================================================

static lv_color_t get_voltage_color(float v) {
  if (v >= SUPERCAP_FULL_V)
    return COLOR_GREEN;
  if (v >= LOW_VOLTAGE_WARN)
    return COLOR_YELLOW;
  if (v >= LOW_VOLTAGE_BLOCK)
    return COLOR_RED;
  return lv_color_hex(0x660000);
}

static void format_ms_value(char *buf, size_t len, float val) {
  if (val == 0.0f) {
    snprintf(buf, len, "OFF");
  } else if (val == (int)val) {
    snprintf(buf, len, "%dms", (int)val);
  } else {
    snprintf(buf, len, "%.1fms", val);
  }
}

// ============================================================================
// Parameter Card Creation
// ============================================================================

static void param_btn_cb(lv_event_t *e) {
  param_card_t *card = (param_card_t *)lv_event_get_user_data(e);
  lv_obj_t *target = lv_event_get_target(e);

  if (target == card->btn_plus) {
    *card->value_ptr += card->step;
    if (*card->value_ptr > card->max_val)
      *card->value_ptr = card->max_val;
  } else if (target == card->btn_minus) {
    *card->value_ptr -= card->step;
    if (*card->value_ptr < card->min_val)
      *card->value_ptr = card->min_val;
  }

  // Update label
  char buf[16];
  if (card->unit[0] == 's') {
    snprintf(buf, sizeof(buf), "%.1f%s", *card->value_ptr, card->unit);
  } else {
    format_ms_value(buf, sizeof(buf), *card->value_ptr);
  }
  lv_label_set_text(card->label_value, buf);

  audio_play_beep();
  settings_save();
}

static void create_param_card(lv_obj_t *parent, param_card_t *card,
                              const char *name, float *val_ptr, float min_v,
                              float max_v, float step, const char *unit, int x,
                              int y, int w) {
  card->value_ptr = val_ptr;
  card->min_val = min_v;
  card->max_val = max_v;
  card->step = step;
  card->unit = unit;

  // Card container
  card->card = lv_obj_create(parent);
  lv_obj_set_size(card->card, w, 90);
  lv_obj_set_pos(card->card, x, y);
  lv_obj_set_style_bg_color(card->card, COLOR_CARD_DARK, 0);
  lv_obj_set_style_border_color(card->card, COLOR_ACCENT, 0);
  lv_obj_set_style_border_width(card->card, 1, 0);
  lv_obj_set_style_radius(card->card, 12, 0);
  lv_obj_set_style_pad_all(card->card, 6, 0);
  lv_obj_clear_flag(card->card, LV_OBJ_FLAG_SCROLLABLE);

  // Parameter name
  card->label_name = lv_label_create(card->card);
  lv_label_set_text(card->label_name, name);
  lv_obj_set_style_text_color(card->label_name, COLOR_TEXT_DIM, 0);
  lv_obj_set_style_text_font(card->label_name, &lv_font_montserrat_14, 0);
  lv_obj_align(card->label_name, LV_ALIGN_TOP_MID, 0, 0);

  // Value
  card->label_value = lv_label_create(card->card);
  char buf[16];
  if (unit[0] == 's') {
    snprintf(buf, sizeof(buf), "%.1f%s", *val_ptr, unit);
  } else {
    format_ms_value(buf, sizeof(buf), *val_ptr);
  }
  lv_label_set_text(card->label_value, buf);
  lv_obj_set_style_text_color(card->label_value, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_style_text_font(card->label_value, &lv_font_montserrat_24, 0);
  lv_obj_align(card->label_value, LV_ALIGN_CENTER, 0, 2);

  // Minus button
  card->btn_minus = lv_btn_create(card->card);
  lv_obj_set_size(card->btn_minus, 36, 30);
  lv_obj_align(card->btn_minus, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(card->btn_minus, COLOR_ACCENT, 0);
  lv_obj_set_style_radius(card->btn_minus, 8, 0);
  lv_obj_add_event_cb(card->btn_minus, param_btn_cb, LV_EVENT_CLICKED, card);
  lv_obj_t *lbl_m = lv_label_create(card->btn_minus);
  lv_label_set_text(lbl_m, LV_SYMBOL_MINUS);
  lv_obj_center(lbl_m);

  // Plus button
  card->btn_plus = lv_btn_create(card->card);
  lv_obj_set_size(card->btn_plus, 36, 30);
  lv_obj_align(card->btn_plus, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(card->btn_plus, COLOR_ACCENT, 0);
  lv_obj_set_style_radius(card->btn_plus, 8, 0);
  lv_obj_add_event_cb(card->btn_plus, param_btn_cb, LV_EVENT_CLICKED, card);
  lv_obj_t *lbl_p = lv_label_create(card->btn_plus);
  lv_label_set_text(lbl_p, LV_SYMBOL_PLUS);
  lv_obj_center(lbl_p);
}

// ============================================================================
// Main Dashboard Screen
// ============================================================================

static void btn_settings_cb(lv_event_t *e) {
  (void)e;
  audio_play_beep();
  lv_scr_load_anim(scr_settings, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
}

static void btn_mode_toggle_cb(lv_event_t *e) {
  (void)e;
  g_settings.auto_mode = !g_settings.auto_mode;
  lv_label_set_text(lbl_mode, g_settings.auto_mode ? "AUTO" : "MAN");
  lv_obj_set_style_text_color(
      lbl_mode, g_settings.auto_mode ? COLOR_BLUE : COLOR_GREEN, 0);

  // Show/hide S card, reposition voltage bar, and show/hide chart.
  // Layout (portrait 480×320, landscape rendered):
  //   MAN: P1/T/P2 (y=42–131) → vbar (y=142) → chart (y=188) → bottom (276)
  //   AUTO: P1/T/P2 (y=42–131) → S card (y=138–227) → vbar (y=232) → bottom (276)
  //   (chart hidden in AUTO — no vertical room remains)
  if (card_s.card) {
    if (g_settings.auto_mode) {
      lv_obj_clear_flag(card_s.card, LV_OBJ_FLAG_HIDDEN);
      if (volt_container) lv_obj_set_pos(volt_container, 10, 232);
      if (chart_voltage)  lv_obj_add_flag(chart_voltage, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(card_s.card, LV_OBJ_FLAG_HIDDEN);
      if (volt_container) lv_obj_set_pos(volt_container, 10, 142);
      if (chart_voltage)  lv_obj_clear_flag(chart_voltage, LV_OBJ_FLAG_HIDDEN);
    }
  }
  audio_play_beep();
  settings_save();
}

static void create_main_screen(void) {
  scr_main = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_main, COLOR_BG_DARK, 0);
  lv_obj_clear_flag(scr_main, LV_OBJ_FLAG_SCROLLABLE);

  // ── Status Bar (top) ─────────────────────────────
  lv_obj_t *status_bar = lv_obj_create(scr_main);
  lv_obj_set_size(status_bar, 480, 36);
  lv_obj_set_pos(status_bar, 0, 0);
  lv_obj_set_style_bg_color(status_bar, COLOR_CARD_DARK, 0);
  lv_obj_set_style_border_width(status_bar, 0, 0);
  lv_obj_set_style_radius(status_bar, 0, 0);
  lv_obj_set_style_pad_hor(status_bar, 10, 0);
  lv_obj_clear_flag(status_bar, LV_OBJ_FLAG_SCROLLABLE);

  // Title
  lv_obj_t *lbl_title = lv_label_create(status_bar);
  lv_label_set_text(lbl_title, LV_SYMBOL_CHARGE " MyWeld");
  lv_obj_set_style_text_color(lbl_title, COLOR_PRIMARY, 0);
  lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_16, 0);
  lv_obj_align(lbl_title, LV_ALIGN_LEFT_MID, 0, 0);

  // Preset name
  lbl_preset = lv_label_create(status_bar);
  lv_label_set_text(lbl_preset,
                    g_settings.presets[g_settings.active_preset].name);
  lv_obj_set_style_text_color(lbl_preset, COLOR_TEXT_DIM, 0);
  lv_obj_set_style_text_font(lbl_preset, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_preset, LV_ALIGN_CENTER, 0, 0);

  // Mode indicator (tappable)
  lbl_mode = lv_label_create(status_bar);
  lv_label_set_text(lbl_mode, g_settings.auto_mode ? "AUTO" : "MAN");
  lv_obj_set_style_text_color(
      lbl_mode, g_settings.auto_mode ? COLOR_BLUE : COLOR_GREEN, 0);
  lv_obj_set_style_text_font(lbl_mode, &lv_font_montserrat_16, 0);
  lv_obj_align(lbl_mode, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_add_flag(lbl_mode, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(lbl_mode, btn_mode_toggle_cb, LV_EVENT_CLICKED, NULL);

  // ── Parameter Cards ──────────────────────────────
  int card_y = 42;
  int card_w = 148;
  int gap = 6;

  create_param_card(scr_main, &card_p1, "PULSE 1", &g_settings.p1, PULSE_MIN_MS,
                    PULSE_MAX_MS, PULSE_STEP_MS, "ms", gap, card_y, card_w);

  create_param_card(scr_main, &card_t, "PAUSE", &g_settings.t, PULSE_MIN_MS,
                    PULSE_MAX_MS, PULSE_STEP_MS, "ms", gap + card_w + gap,
                    card_y, card_w);

  create_param_card(scr_main, &card_p2, "PULSE 2", &g_settings.p2, PULSE_MIN_MS,
                    PULSE_MAX_MS, PULSE_STEP_MS, "ms", gap + (card_w + gap) * 2,
                    card_y, card_w);

  // S value card (AUTO mode only)
  create_param_card(scr_main, &card_s, "DELAY (S)", &g_settings.s_value,
                    S_VALUE_MIN, S_VALUE_MAX, S_VALUE_STEP, "s", gap,
                    card_y + 96, 220);
  if (!g_settings.auto_mode) {
    lv_obj_add_flag(card_s.card, LV_OBJ_FLAG_HIDDEN);
  }

  // ── Voltage Bar ──────────────────────────────────
  // y=142 in MAN mode (directly below P1/T/P2 cards whose bottom = 132).
  // y=232 in AUTO mode (directly below DELAY card whose bottom = 228).
  int vbar_y = g_settings.auto_mode ? 232 : 142;

  volt_container = lv_obj_create(scr_main);
  lv_obj_set_size(volt_container, 460, 40);
  lv_obj_set_pos(volt_container, 10, vbar_y);
  lv_obj_set_style_bg_color(volt_container, COLOR_CARD_DARK, 0);
  lv_obj_set_style_border_width(volt_container, 0, 0);
  lv_obj_set_style_radius(volt_container, 10, 0);
  lv_obj_set_style_pad_all(volt_container, 4, 0);
  lv_obj_clear_flag(volt_container, LV_OBJ_FLAG_SCROLLABLE);

  bar_voltage = lv_bar_create(volt_container);
  lv_obj_set_size(bar_voltage, 320, 20);
  lv_obj_align(bar_voltage, LV_ALIGN_LEFT_MID, 4, 0);
  lv_bar_set_range(bar_voltage, 0, 100);
  lv_bar_set_value(bar_voltage, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_voltage, lv_color_hex(0x333333), LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_voltage, COLOR_GREEN, LV_PART_INDICATOR);
  lv_obj_set_style_radius(bar_voltage, 6, LV_PART_MAIN);
  lv_obj_set_style_radius(bar_voltage, 6, LV_PART_INDICATOR);

  lbl_voltage = lv_label_create(volt_container);
  lv_label_set_text(lbl_voltage, "0.0V");
  lv_obj_set_style_text_color(lbl_voltage, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_style_text_font(lbl_voltage, &lv_font_montserrat_20, 0);
  lv_obj_align(lbl_voltage, LV_ALIGN_RIGHT_MID, -4, -6);

  lbl_voltage_pct = lv_label_create(volt_container);
  lv_label_set_text(lbl_voltage_pct, "0%");
  lv_obj_set_style_text_color(lbl_voltage_pct, COLOR_TEXT_DIM, 0);
  lv_obj_set_style_text_font(lbl_voltage_pct, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_voltage_pct, LV_ALIGN_RIGHT_MID, -4, 10);

  // ── Voltage Chart ────────────────────────────────
  chart_voltage = lv_chart_create(scr_main);
  lv_obj_set_size(chart_voltage, 460, 80);
  lv_obj_set_pos(chart_voltage, 10, 185);
  lv_obj_set_style_bg_color(chart_voltage, COLOR_CARD_DARK, 0);
  lv_obj_set_style_border_color(chart_voltage, COLOR_ACCENT, 0);
  lv_obj_set_style_border_width(chart_voltage, 1, 0);
  lv_obj_set_style_radius(chart_voltage, 10, 0);
  lv_obj_set_style_line_color(chart_voltage, lv_color_hex(0x333333),
                              LV_PART_MAIN);
  lv_chart_set_type(chart_voltage, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(chart_voltage, 60); // 30 seconds at 0.5s interval
  lv_chart_set_range(chart_voltage, LV_CHART_AXIS_PRIMARY_Y, 0,
                     60); // 0–6.0V × 10
  lv_chart_set_div_line_count(chart_voltage, 3, 5);

  chart_series =
      lv_chart_add_series(chart_voltage, COLOR_GREEN, LV_CHART_AXIS_PRIMARY_Y);

  // Initialize with zeros
  for (int i = 0; i < 60; i++) {
    lv_chart_set_next_value(chart_voltage, chart_series, 0);
  }

  // In AUTO mode, chart hidden — voltage bar is pushed to y=232 leaving no
  // room for the chart before the bottom bar at y=276.
  if (g_settings.auto_mode) {
    lv_obj_add_flag(chart_voltage, LV_OBJ_FLAG_HIDDEN);
  }

  // ── Bottom Status Bar ────────────────────────────
  lv_obj_t *bottom_bar = lv_obj_create(scr_main);
  lv_obj_set_size(bottom_bar, 480, 44);
  lv_obj_set_pos(bottom_bar, 0, 276);
  lv_obj_set_style_bg_color(bottom_bar, COLOR_CARD_DARK, 0);
  lv_obj_set_style_border_width(bottom_bar, 0, 0);
  lv_obj_set_style_radius(bottom_bar, 0, 0);
  lv_obj_set_style_pad_hor(bottom_bar, 10, 0);
  lv_obj_clear_flag(bottom_bar, LV_OBJ_FLAG_SCROLLABLE);

  // Weld counter
  lbl_weld_count = lv_label_create(bottom_bar);
  lv_label_set_text_fmt(lbl_weld_count, LV_SYMBOL_CHARGE " %lu / %lu",
                        (unsigned long)g_settings.session_welds,
                        (unsigned long)g_settings.total_welds);
  lv_obj_set_style_text_color(lbl_weld_count, COLOR_TEXT_DIM, 0);
  lv_obj_set_style_text_font(lbl_weld_count, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_weld_count, LV_ALIGN_LEFT_MID, 0, 0);

  // Status indicator
  lbl_status = lv_label_create(bottom_bar);
  lv_label_set_text(lbl_status, LV_SYMBOL_OK " READY");
  lv_obj_set_style_text_color(lbl_status, COLOR_GREEN, 0);
  lv_obj_set_style_text_font(lbl_status, &lv_font_montserrat_16, 0);
  lv_obj_align(lbl_status, LV_ALIGN_CENTER, 0, 0);

  // Settings gear button
  lv_obj_t *btn_settings = lv_btn_create(bottom_bar);
  lv_obj_set_size(btn_settings, 40, 32);
  lv_obj_align(btn_settings, LV_ALIGN_RIGHT_MID, 0, 0);
  lv_obj_set_style_bg_color(btn_settings, COLOR_ACCENT, 0);
  lv_obj_set_style_radius(btn_settings, 8, 0);
  lv_obj_add_event_cb(btn_settings, btn_settings_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_gear = lv_label_create(btn_settings);
  lv_label_set_text(lbl_gear, LV_SYMBOL_SETTINGS);
  lv_obj_center(lbl_gear);
}

// ============================================================================
// Settings Screen
// ============================================================================

static void btn_back_cb(lv_event_t *e) {
  (void)e;
  audio_play_beep();
  lv_scr_load_anim(scr_main, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
}

static void slider_brightness_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  g_settings.brightness = (uint8_t)lv_slider_get_value(slider);
  display_set_brightness(g_settings.brightness);
  settings_save();
}

static void slider_volume_cb(lv_event_t *e) {
  lv_obj_t *slider = lv_event_get_target(e);
  g_settings.volume = (uint8_t)lv_slider_get_value(slider);
  audio_set_volume(g_settings.volume);
  audio_play_beep(); // Play beep at new volume so user hears the level
  settings_save();
}

static void sw_sound_cb(lv_event_t *e) {
  lv_obj_t *sw = lv_event_get_target(e);
  g_settings.sound_on = lv_obj_has_state(sw, LV_STATE_CHECKED);
  audio_set_muted(!g_settings.sound_on);
  if (g_settings.sound_on)
    audio_play_beep();
  settings_save();
}

static void sw_theme_cb(lv_event_t *e) {
  lv_obj_t *sw = lv_event_get_target(e);
  g_settings.theme = lv_obj_has_state(sw, LV_STATE_CHECKED) ? 1 : 0;
  ui_set_theme(g_settings.theme);
  settings_save();
}

static void preset_dropdown_cb(lv_event_t *e) {
  lv_obj_t *dd = lv_event_get_target(e);
  uint16_t sel = lv_dropdown_get_selected(dd);
  settings_load_preset(sel);

  // Update main screen cards
  char buf[16];
  format_ms_value(buf, sizeof(buf), g_settings.p1);
  lv_label_set_text(card_p1.label_value, buf);
  format_ms_value(buf, sizeof(buf), g_settings.t);
  lv_label_set_text(card_t.label_value, buf);
  format_ms_value(buf, sizeof(buf), g_settings.p2);
  lv_label_set_text(card_p2.label_value, buf);
  snprintf(buf, sizeof(buf), "%.1fs", g_settings.s_value);
  lv_label_set_text(card_s.label_value, buf);

  if (lbl_preset) {
    lv_label_set_text(lbl_preset, g_settings.presets[sel].name);
  }

  audio_play_beep();
}

static void create_settings_screen(void) {
  scr_settings = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_settings, COLOR_BG_DARK, 0);
  lv_obj_set_style_pad_all(scr_settings, 10, 0);

  // Back button
  lv_obj_t *btn_back = lv_btn_create(scr_settings);
  lv_obj_set_size(btn_back, 80, 32);
  lv_obj_set_pos(btn_back, 0, 0);
  lv_obj_set_style_bg_color(btn_back, COLOR_ACCENT, 0);
  lv_obj_set_style_radius(btn_back, 8, 0);
  lv_obj_add_event_cb(btn_back, btn_back_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_back = lv_label_create(btn_back);
  lv_label_set_text(lbl_back, LV_SYMBOL_LEFT " Back");
  lv_obj_center(lbl_back);

  // Title
  lv_obj_t *title = lv_label_create(scr_settings);
  lv_label_set_text(title, "Settings");
  lv_obj_set_style_text_color(title, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 2);

  int row_y = 45;
  int row_h = 40;

  // ── Preset Selector ──────────────────────────────
  lv_obj_t *lbl_preset_s = lv_label_create(scr_settings);
  lv_label_set_text(lbl_preset_s, "Preset:");
  lv_obj_set_style_text_color(lbl_preset_s, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_preset_s, 10, row_y + 6);

  // Build preset options string
  char preset_opts[MAX_PRESETS * (PRESET_NAME_LEN + 1)];
  preset_opts[0] = '\0';
  for (int i = 0; i < MAX_PRESETS; i++) {
    if (i > 0)
      strcat(preset_opts, "\n");
    strcat(preset_opts, g_settings.presets[i].name);
  }

  lv_obj_t *dd_preset = lv_dropdown_create(scr_settings);
  lv_dropdown_set_options(dd_preset, preset_opts);
  lv_dropdown_set_selected(dd_preset, g_settings.active_preset);
  lv_obj_set_size(dd_preset, 280, 36);
  lv_obj_set_pos(dd_preset, 180, row_y);
  lv_obj_add_event_cb(dd_preset, preset_dropdown_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  row_y += row_h + 5;

  // ── Brightness Slider ────────────────────────────
  lv_obj_t *lbl_br = lv_label_create(scr_settings);
  lv_label_set_text(lbl_br, LV_SYMBOL_IMAGE " Brightness:");
  lv_obj_set_style_text_color(lbl_br, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_br, 10, row_y + 6);

  lv_obj_t *slider_br = lv_slider_create(scr_settings);
  lv_obj_set_size(slider_br, 250, 16);
  lv_obj_set_pos(slider_br, 200, row_y + 8);
  lv_slider_set_range(slider_br, 10, 100);
  lv_slider_set_value(slider_br, g_settings.brightness, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(slider_br, COLOR_ACCENT, LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider_br, COLOR_PRIMARY, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider_br, COLOR_TEXT_LIGHT, LV_PART_KNOB);
  lv_obj_add_event_cb(slider_br, slider_brightness_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  row_y += row_h;

  // ── Volume Slider ───────────────────────────────
  lv_obj_t *lbl_vol = lv_label_create(scr_settings);
  lv_label_set_text(lbl_vol, LV_SYMBOL_VOLUME_MAX " Volume:");
  lv_obj_set_style_text_color(lbl_vol, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_vol, 10, row_y + 6);

  lv_obj_t *slider_vol = lv_slider_create(scr_settings);
  lv_obj_set_size(slider_vol, 250, 16);
  lv_obj_set_pos(slider_vol, 200, row_y + 8);
  lv_slider_set_range(slider_vol, 0, 100);
  lv_slider_set_value(slider_vol, g_settings.volume, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(slider_vol, COLOR_ACCENT, LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider_vol, COLOR_BLUE, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider_vol, COLOR_TEXT_LIGHT, LV_PART_KNOB);
  lv_obj_add_event_cb(slider_vol, slider_volume_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  row_y += row_h;

  // ── Sound Toggle ─────────────────────────────────
  lv_obj_t *lbl_snd = lv_label_create(scr_settings);
  lv_label_set_text(lbl_snd, LV_SYMBOL_AUDIO " Sound:");
  lv_obj_set_style_text_color(lbl_snd, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_snd, 10, row_y + 6);

  lv_obj_t *sw_sound = lv_switch_create(scr_settings);
  lv_obj_set_pos(sw_sound, 410, row_y + 2);
  if (g_settings.sound_on)
    lv_obj_add_state(sw_sound, LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(sw_sound, COLOR_GREEN,
                            LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(sw_sound, sw_sound_cb, LV_EVENT_VALUE_CHANGED, NULL);
  row_y += row_h;

  // ── Theme Toggle ─────────────────────────────────
  lv_obj_t *lbl_thm = lv_label_create(scr_settings);
  lv_label_set_text(lbl_thm, LV_SYMBOL_EYE_OPEN " Light Mode:");
  lv_obj_set_style_text_color(lbl_thm, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_thm, 10, row_y + 6);

  lv_obj_t *sw_theme = lv_switch_create(scr_settings);
  lv_obj_set_pos(sw_theme, 410, row_y + 2);
  if (g_settings.theme == 1)
    lv_obj_add_state(sw_theme, LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(sw_theme, COLOR_YELLOW,
                            LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(sw_theme, sw_theme_cb, LV_EVENT_VALUE_CHANGED, NULL);
  row_y += row_h;

  // ── Version Info ─────────────────────────────────
  row_y += 10;
  lv_obj_t *lbl_ver = lv_label_create(scr_settings);
  lv_label_set_text_fmt(lbl_ver, "MyWeld ESP32 v%s | %s", FW_VERSION_STRING,
                        FW_BUILD_DATE);
  lv_obj_set_style_text_color(lbl_ver, COLOR_TEXT_DIM, 0);
  lv_obj_set_style_text_font(lbl_ver, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_ver, LV_ALIGN_BOTTOM_MID, 0, -5);
}

// ============================================================================
// Public API
// ============================================================================

void ui_init(void) {
  create_main_screen();
  create_settings_screen();
  // NOTE: lv_scr_load is called from ui_task() after the flush task handle
  // and LVGL tick are properly initialized. Don't call it here.
  ESP_LOGI(TAG, "UI initialized: main dashboard + settings");
}

// ============================================================================
// Countdown Timer Callback (factory reset reboot sequence)
// ============================================================================

static void countdown_timer_cb(lv_timer_t *t)
{
    (void)t;
    s_countdown_val--;
    if (s_countdown_val > 0) {
        // Update digit and play beep each second
        if (lbl_countdown) {
            lv_label_set_text_fmt(lbl_countdown, "%d", s_countdown_val);
        }
        audio_play_beep();
    } else {
        // Countdown finished — clean up and reboot
        if (tmr_countdown) {
            lv_timer_delete(tmr_countdown);
            tmr_countdown = NULL;
        }
        ESP_LOGW(TAG, "Factory reset countdown complete — rebooting now");
        vTaskDelay(pdMS_TO_TICKS(200)); // Let final frame flush to display
        esp_restart();
    }
}

void ui_task(void *pvParameters) {
  ESP_LOGI(TAG, "UI task started on Core %d", xPortGetCoreID());

  // Create the inter-task message queue before the loop.
  s_ui_queue = xQueueCreate(UI_QUEUE_DEPTH, sizeof(ui_msg_t));
  if (!s_ui_queue) {
    ESP_LOGE(TAG, "Failed to create UI queue!");
    vTaskDelete(NULL);
    return;
  }

  // Register THIS task as the DMA flush notification receiver so that
  // lvgl_flush_cb's ulTaskNotifyTake gets woken by the DMA-done ISR.
  display_set_flush_task(xTaskGetCurrentTaskHandle());

  // Provide an initial tick so LVGL timers have a non-zero base, then load
  // the screen. Doing this here (not in app_main) ensures the flush task
  // handle is set and lv_tick is running before the first render attempt.
  lv_tick_inc(20);
  lv_scr_load(scr_main);
  lv_obj_invalidate(lv_screen_active());

  // Seed the tick counter so the first lv_tick_inc() delta is correct.
  int64_t last_tick_us = esp_timer_get_time();

  // Backlight is kept OFF during display_init() to hide GRAM garbage.
  // We enable it after the first frame is fully flushed to the panel.
  bool backlight_enabled = false;

  while (1) {
    // ── Advance LVGL tick ───────────────────────────────────────────────────
    // LV_TICK_CUSTOM=0 requires manual lv_tick_inc() calls.
    // We compute exact elapsed time so LVGL's animation & timer engine
    // stays accurate regardless of vTaskDelay jitter.
    int64_t now_us = esp_timer_get_time();
    uint32_t elapsed_ms = (uint32_t)((now_us - last_tick_us) / 1000);
    if (elapsed_ms > 0) {
      lv_tick_inc(elapsed_ms);
      last_tick_us = now_us;
    }

    // ── Drain the UI message queue ──────────────────────────────────────────
    ui_msg_t msg;
    while (xQueueReceive(s_ui_queue, &msg, 0) == pdTRUE) {
      switch (msg.type) {

        case UI_MSG_VOLTAGE: {
          if (!lbl_voltage || !bar_voltage) break;
          float voltage = msg.voltage;
          float pct = (voltage / SUPERCAP_MAX_V) * 100.0f;
          if (pct < 0)   pct = 0;
          if (pct > 100) pct = 100;
          lv_label_set_text_fmt(lbl_voltage, "%.1fV", voltage);
          lv_label_set_text_fmt(lbl_voltage_pct, "%d%%", (int)pct);
          lv_bar_set_value(bar_voltage, (int)pct, LV_ANIM_OFF);
          lv_color_t color = get_voltage_color(voltage);
          lv_obj_set_style_bg_color(bar_voltage, color, LV_PART_INDICATOR);
          break;
        }

        case UI_MSG_WELD_STATE: {
          if (!lbl_status) break;
          uint8_t state = msg.weld_state;
          const char *state_str = welding_state_str((weld_state_t)state);
          lv_color_t color;
          const char *icon;
          switch ((weld_state_t)state) {
            case WELD_STATE_IDLE:     color = COLOR_GREEN;    icon = LV_SYMBOL_OK;      break;
            case WELD_STATE_ARMED:    color = COLOR_YELLOW;   icon = LV_SYMBOL_WARNING; break;
            case WELD_STATE_FIRING_P1:
            case WELD_STATE_FIRING_P2: color = COLOR_PRIMARY; icon = LV_SYMBOL_CHARGE;  break;
            case WELD_STATE_BLOCKED:  color = COLOR_RED;      icon = LV_SYMBOL_CLOSE;   break;
            case WELD_STATE_ERROR:    color = COLOR_RED;      icon = LV_SYMBOL_WARNING; break;
            default:                  color = COLOR_TEXT_DIM; icon = LV_SYMBOL_REFRESH; break;
          }
          lv_label_set_text_fmt(lbl_status, "%s %s", icon, state_str);
          lv_obj_set_style_text_color(lbl_status, color, 0);
          break;
        }

        case UI_MSG_WELD_COUNT: {
          if (!lbl_weld_count) break;
          lv_label_set_text_fmt(lbl_weld_count, LV_SYMBOL_CHARGE " %lu / %lu",
                                (unsigned long)msg.weld_count.session,
                                (unsigned long)msg.weld_count.total);
          break;
        }

        case UI_MSG_GRAPH_POINT: {
          if (!chart_voltage || !chart_series) break;
          float v = msg.voltage;
          lv_chart_set_next_value(chart_voltage, chart_series, (int)(v * 10));
          lv_chart_refresh(chart_voltage);
          lv_chart_set_series_color(chart_voltage, chart_series, get_voltage_color(v));
          break;
        }

        case UI_MSG_REFRESH_PARAMS: {
          // Sync P1/T/P2/S labels from g_settings (triggered by BLE write)
          char buf[16];
          if (card_p1.label_value) {
            format_ms_value(buf, sizeof(buf), g_settings.p1);
            lv_label_set_text(card_p1.label_value, buf);
          }
          if (card_t.label_value) {
            format_ms_value(buf, sizeof(buf), g_settings.t);
            lv_label_set_text(card_t.label_value, buf);
          }
          if (card_p2.label_value) {
            format_ms_value(buf, sizeof(buf), g_settings.p2);
            lv_label_set_text(card_p2.label_value, buf);
          }
          if (card_s.label_value) {
            snprintf(buf, sizeof(buf), "%.1fs", g_settings.s_value);
            lv_label_set_text(card_s.label_value, buf);
          }
          // Update preset name in status bar
          if (lbl_preset) {
            lv_label_set_text(lbl_preset,
                              g_settings.presets[g_settings.active_preset].name);
          }
          // Sync mode label + S-card visibility
          if (lbl_mode) {
            lv_label_set_text(lbl_mode, g_settings.auto_mode ? "AUTO" : "MAN");
            lv_obj_set_style_text_color(
                lbl_mode, g_settings.auto_mode ? COLOR_BLUE : COLOR_GREEN, 0);
          }
          if (card_s.card) {
            if (g_settings.auto_mode) {
              lv_obj_clear_flag(card_s.card, LV_OBJ_FLAG_HIDDEN);
              if (volt_container) lv_obj_set_pos(volt_container, 10, 232);
              if (chart_voltage)  lv_obj_add_flag(chart_voltage, LV_OBJ_FLAG_HIDDEN);
            } else {
              lv_obj_add_flag(card_s.card, LV_OBJ_FLAG_HIDDEN);
              if (volt_container) lv_obj_set_pos(volt_container, 10, 142);
              if (chart_voltage)  lv_obj_clear_flag(chart_voltage, LV_OBJ_FLAG_HIDDEN);
            }
          }
          ESP_LOGI(TAG, "UI params refreshed from BLE (P1=%.1f T=%.1f P2=%.1f S=%.1f %s)",
                   g_settings.p1, g_settings.t, g_settings.p2, g_settings.s_value,
                   g_settings.auto_mode ? "AUTO" : "MAN");
          break;
        }

        case UI_MSG_REBOOT_COUNTDOWN: {
          // ── Build full-screen countdown overlay ────────────────────────────
          // Kill any previous countdown timer if re-triggered
          if (tmr_countdown) {
            lv_timer_delete(tmr_countdown);
            tmr_countdown = NULL;
          }
          if (scr_countdown) {
            lv_obj_del(scr_countdown);
            scr_countdown = NULL;
            lbl_countdown = NULL;
          }

          s_countdown_val = 3;

          // Full-screen dark overlay
          scr_countdown = lv_obj_create(NULL);
          lv_obj_set_style_bg_color(scr_countdown, lv_color_hex(0x0D0D1A), 0);
          lv_obj_clear_flag(scr_countdown, LV_OBJ_FLAG_SCROLLABLE);

          // "Factory Resetting" header label
          lv_obj_t *lbl_title = lv_label_create(scr_countdown);
          lv_label_set_text(lbl_title, LV_SYMBOL_WARNING " Factory Reset");
          lv_obj_set_style_text_color(lbl_title, lv_color_hex(0xFF4444), 0);
          lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_24, 0);
          lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 30);

          // Sub-label "Rebooting in..."
          lv_obj_t *lbl_sub = lv_label_create(scr_countdown);
          lv_label_set_text(lbl_sub, "Rebooting in...");
          lv_obj_set_style_text_color(lbl_sub, lv_color_hex(0xAAAAAA), 0);
          lv_obj_set_style_text_font(lbl_sub, &lv_font_montserrat_16, 0);
          lv_obj_align(lbl_sub, LV_ALIGN_CENTER, 0, -30);

          // Large countdown digit
          lbl_countdown = lv_label_create(scr_countdown);
          lv_label_set_text(lbl_countdown, "3");
          lv_obj_set_style_text_color(lbl_countdown, lv_color_hex(0xFF4444), 0);
          lv_obj_set_style_text_font(lbl_countdown, &lv_font_montserrat_48, 0);
          lv_obj_align(lbl_countdown, LV_ALIGN_CENTER, 0, 30);

          // Load the overlay immediately
          lv_scr_load(scr_countdown);

          // Beep to alert the user
          audio_play_beep();

          // LVGL timer fires every 1 second — callback decrements the digit
          tmr_countdown = lv_timer_create(countdown_timer_cb, 1000, NULL);

          ESP_LOGI(TAG, "Factory reset countdown started (3s)");
          break;
        }

        default: break;
      }
    }
    // ── End queue drain ─────────────────────────────────────────────────────

    // lv_timer_handler renders and flushes the display.
    // flush_ready is signalled by the DMA ISR (panel_trans_done_cb).
    uint32_t ms_to_next = lv_timer_handler();
    if (ms_to_next < 1)  ms_to_next = 1;
    if (ms_to_next > 10) ms_to_next = 10;

    // Enable backlight after the first complete frame is flushed.
    // The panel now shows clean UI pixels \u2014 no GRAM garbage visible.
    if (!backlight_enabled) {
      display_enable_backlight();
      backlight_enabled = true;
    }

    vTaskDelay(pdMS_TO_TICKS(ms_to_next));
  }
}


void ui_update_voltage(float voltage) {
  if (!s_ui_queue) return;
  ui_msg_t msg = { .type = UI_MSG_VOLTAGE, .voltage = voltage };
  // xQueueSend is ISR-safe under portMAX_DELAY=0 — non-blocking, drop if full.
  xQueueSend(s_ui_queue, &msg, 0);
}

void ui_update_protection(float voltage) {
  (void)voltage;
  // TODO: Update protection rail indicator if added
}

void ui_update_weld_state(uint8_t state) {
  if (!s_ui_queue) return;
  ui_msg_t msg = { .type = UI_MSG_WELD_STATE, .weld_state = state };
  xQueueSend(s_ui_queue, &msg, 0);
}

void ui_update_weld_count(uint32_t session, uint32_t total) {
  if (!s_ui_queue) return;
  ui_msg_t msg = {
    .type = UI_MSG_WELD_COUNT,
    .weld_count = { .session = session, .total = total }
  };
  xQueueSend(s_ui_queue, &msg, 0);
}

void ui_graph_add_point(float voltage) {
  if (!s_ui_queue) return;
  ui_msg_t msg = { .type = UI_MSG_GRAPH_POINT, .voltage = voltage };
  xQueueSend(s_ui_queue, &msg, 0);
}

void ui_refresh_params(void) {
  if (!s_ui_queue) return;
  ui_msg_t msg = { .type = UI_MSG_REFRESH_PARAMS };
  // Use a short timeout so the BLE callback (NimBLE task) doesn't block long.
  xQueueSend(s_ui_queue, &msg, pdMS_TO_TICKS(5));
}

void ui_trigger_reboot_countdown(void) {
  if (!s_ui_queue) {
    // Queue not up yet — reboot immediately as last resort
    esp_restart();
    return;
  }
  ui_msg_t msg = { .type = UI_MSG_REBOOT_COUNTDOWN };
  xQueueSend(s_ui_queue, &msg, pdMS_TO_TICKS(50));
}

void ui_set_theme(uint8_t theme) {
  // Called from sw_theme_cb which runs inside lv_timer_handler() in ui_task.
  // We are already in the LVGL context — no mutex needed (and taking it would
  // self-deadlock on a non-recursive FreeRTOS mutex).
  lv_color_t bg = (theme == 0) ? COLOR_BG_DARK : COLOR_BG_LIGHT;

  if (scr_main)
    lv_obj_set_style_bg_color(scr_main, bg, 0);
  if (scr_settings)
    lv_obj_set_style_bg_color(scr_settings, bg, 0);
  // Full theme switch would require updating all widget colors.
  // For now, just switch background.
  ESP_LOGI(TAG, "Theme set to %s", theme == 0 ? "dark" : "light");
}

// Wrapper for animation: sets object opacity (matches lv_anim_exec_xcb_t
// signature)
static void notify_opa_anim_cb(void *obj, int32_t opa) {
  lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)opa, 0);
}

void ui_show_notification(const char *message, uint16_t duration_ms) {
  if (display_lock(10)) {
    lv_obj_t *notify = lv_label_create(lv_scr_act());
    lv_label_set_text(notify, message);
    lv_obj_set_style_bg_color(notify, COLOR_ACCENT, 0);
    lv_obj_set_style_bg_opa(notify, LV_OPA_90, 0);
    lv_obj_set_style_text_color(notify, COLOR_TEXT_LIGHT, 0);
    lv_obj_set_style_pad_all(notify, 10, 0);
    lv_obj_set_style_radius(notify, 8, 0);
    lv_obj_align(notify, LV_ALIGN_TOP_MID, 0, 40);

    if (duration_ms > 0) {
      lv_anim_t a;
      lv_anim_init(&a);
      lv_anim_set_var(&a, notify);
      lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
      lv_anim_set_time(&a, duration_ms);
      lv_anim_set_exec_cb(&a, notify_opa_anim_cb);
      lv_anim_start(&a);
    }
    display_unlock();
  }
  ESP_LOGI(TAG, "Notification: %s", message);
}
