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
#include "encoder.h"
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
    UI_MSG_REBOOT_COUNTDOWN,        // Show reboot countdown (factory-reset or normal)
    UI_MSG_OTA_PROGRESS,            // OTA firmware update progress (0–100)
    UI_MSG_OTA_HIDE,                // Hide OTA overlay

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
        uint8_t  ota_progress;      // UI_MSG_OTA_PROGRESS (0–100)
        bool     is_factory_reset;  // UI_MSG_REBOOT_COUNTDOWN
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

// Countdown overlay (reboot / factory reset)
static lv_obj_t    *scr_countdown   = NULL;
static lv_obj_t    *lbl_countdown   = NULL;
static lv_timer_t  *tmr_countdown   = NULL;
static int          s_countdown_val = 3;

// OTA progress overlay
static lv_obj_t    *scr_ota         = NULL;
static lv_obj_t    *lbl_ota_pct     = NULL;
static lv_obj_t    *bar_ota         = NULL;

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
static lv_obj_t *lbl_ble_icon = NULL;    // BLE connected indicator (shown/hidden)
static lv_obj_t *lbl_charger_icon = NULL; // Charger status: ⚡ charging / ✕ not charging

// Settings screen widgets (stored globally so BLE refresh can update them)
static lv_obj_t *slider_brightness = NULL;
static lv_obj_t *slider_volume = NULL;
static lv_obj_t *sw_sound_global = NULL;
static lv_obj_t *sw_theme_global = NULL;

// Widgets promoted to global for encoder focus access
static lv_obj_t *s_btn_settings = NULL;  // Settings gear button (main screen)
static lv_obj_t *s_btn_back = NULL;      // Back button (settings screen)
static lv_obj_t *s_dd_preset = NULL;     // Preset dropdown (settings screen)

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

static param_card_t card_p1, card_t, card_p2, card_p3, card_p4, card_s;

// ============================================================================
// Encoder Navigation State
// ============================================================================

typedef enum {
    ENC_SCREEN_MAIN,
    ENC_SCREEN_SETTINGS
} enc_screen_t;

typedef enum {
    ENC_MODE_NAVIGATE,   // Rotate = move focus highlight
    ENC_MODE_EDIT        // Rotate = change value of focused item
} enc_mode_t;

// Focus item types (determines KEY press and rotate behavior)
typedef enum {
    FTYPE_PARAM,     // P1/T/P2/P3/P4/S — editable param_card_t
    FTYPE_TOGGLE,    // AUTO/MAN, Sound, Theme — toggle on KEY press
    FTYPE_ACTION,    // Settings, Back — execute on KEY press
    FTYPE_SLIDER,    // Brightness, Volume — edit mode adjusts slider
    FTYPE_DROPDOWN   // Preset — edit mode cycles options
} focus_type_t;

typedef struct {
    lv_obj_t     *widget;   // LVGL object to highlight
    focus_type_t  type;
    param_card_t *param;    // non-NULL only for FTYPE_PARAM
    int32_t slider_step;    // step for FTYPE_SLIDER
} focus_item_t;

// Main screen focusable items (max 8: P1, T, P2, P3, P4, S, MODE, SETTINGS)
#define MAIN_FOCUS_MAX 8
static focus_item_t s_main_items[MAIN_FOCUS_MAX];
static int s_main_count = 0;

// Settings screen focusable items (max 6: BACK, PRESET, BRIGHT, VOL, SOUND, THEME)
#define SETTINGS_FOCUS_MAX 6
static focus_item_t s_settings_items[SETTINGS_FOCUS_MAX];
static int s_settings_count = 0;

static enc_screen_t s_enc_screen = ENC_SCREEN_MAIN;
static enc_mode_t   s_enc_mode   = ENC_MODE_NAVIGATE;
static int          s_enc_focus  = -1;  // -1 = no encoder focus (touch only)

// Glow colors
#define COLOR_FOCUS_NAV    COLOR_BLUE      // Blue border glow (navigate)
#define COLOR_FOCUS_EDIT   COLOR_GREEN     // Green border glow (edit)

// ============================================================================
// Helpers
// ============================================================================

static inline bool is_user_defined(void) {
  return g_settings.active_preset == PRESET_USER_DEFINED;
}

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
// Encoder Focus Helpers
// ============================================================================

static focus_item_t* enc_current_items(void) {
    return (s_enc_screen == ENC_SCREEN_MAIN) ? s_main_items : s_settings_items;
}

static int enc_current_count(void) {
    if (s_enc_screen == ENC_SCREEN_MAIN) {
        // Full items: P1(0) T(1) P2(2) P3(3) P4(4) S(5) MODE(6) SETTINGS(7) = 8
        // Preset mode hides P1/T/P2/P3/P4 (−5), else P3/P4 hidden when value==0
        // MAN mode hides S (−1)
        int count = s_main_count;
        if (!is_user_defined()) {
            count -= 5; // P1/T/P2/P3/P4 hidden in preset mode
        } else {
            // P3 hidden when value is 0
            if (g_settings.p3 == 0.0f) count--;
            // P4 hidden when value is 0
            if (g_settings.p4 == 0.0f) count--;
        }
        if (!g_settings.auto_mode) count -= 1; // S hidden
        return count;
    }
    return s_settings_count;
}

// Map logical focus index to actual items[] index (skips hidden cards)
// Real items order: P1(0) T(1) P2(2) P3(3) P4(4) S(5) MODE(6) SETTINGS(7)
static int enc_logical_to_real(int logical) {
    if (s_enc_screen != ENC_SCREEN_MAIN) return logical;

    bool ud = is_user_defined();
    bool am = g_settings.auto_mode;

    // Build a skip table: iterate through all real items and map
    int visible = 0;
    for (int real = 0; real < s_main_count; real++) {
        bool hidden = false;
        // P1(0), T(1), P2(2) hidden in preset mode
        if (!ud && real <= 2) hidden = true;
        // P3(3) hidden in preset mode or when value == 0
        if (real == 3 && (!ud || g_settings.p3 == 0.0f)) hidden = true;
        // P4(4) hidden in preset mode or when value == 0
        if (real == 4 && (!ud || g_settings.p4 == 0.0f)) hidden = true;
        // S(5) hidden in MAN mode
        if (real == 5 && !am) hidden = true;

        if (!hidden) {
            if (visible == logical) return real;
            visible++;
        }
    }
    return s_main_count - 1; // fallback
}

static void enc_apply_glow(lv_obj_t *obj, bool focused, bool editing) {
    if (!obj) return;
    if (!focused) {
        // Restore default: thin accent border, no shadow
        lv_obj_set_style_border_color(obj, COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(obj, 1, 0);
        lv_obj_set_style_shadow_width(obj, 0, 0);
        lv_obj_set_style_shadow_opa(obj, LV_OPA_TRANSP, 0);
    } else if (editing) {
        // Green glow — edit mode
        lv_obj_set_style_border_color(obj, COLOR_FOCUS_EDIT, 0);
        lv_obj_set_style_border_width(obj, 3, 0);
        lv_obj_set_style_shadow_width(obj, 12, 0);
        lv_obj_set_style_shadow_color(obj, COLOR_FOCUS_EDIT, 0);
        lv_obj_set_style_shadow_opa(obj, LV_OPA_60, 0);
        lv_obj_set_style_shadow_spread(obj, 2, 0);
    } else {
        // Blue glow — navigate mode
        lv_obj_set_style_border_color(obj, COLOR_FOCUS_NAV, 0);
        lv_obj_set_style_border_width(obj, 2, 0);
        lv_obj_set_style_shadow_width(obj, 8, 0);
        lv_obj_set_style_shadow_color(obj, COLOR_FOCUS_NAV, 0);
        lv_obj_set_style_shadow_opa(obj, LV_OPA_40, 0);
        lv_obj_set_style_shadow_spread(obj, 1, 0);
    }
}

static void enc_clear_all_glow(void) {
    focus_item_t *items = enc_current_items();
    int count = (s_enc_screen == ENC_SCREEN_MAIN) ? s_main_count : s_settings_count;
    for (int i = 0; i < count; i++) {
        enc_apply_glow(items[i].widget, false, false);
    }
}

static void enc_update_focus_visual(void) {
    enc_clear_all_glow();
    if (s_enc_focus < 0) return;
    int real = enc_logical_to_real(s_enc_focus);
    focus_item_t *items = enc_current_items();
    bool editing = (s_enc_mode == ENC_MODE_EDIT);
    enc_apply_glow(items[real].widget, true, editing);
}

static void enc_set_focus(int index) {
    int max = enc_current_count();
    if (max <= 0) return;
    s_enc_focus = ((index % max) + max) % max;  // wrap around
    enc_update_focus_visual();
}

// Update a param card's displayed value from its value_ptr
static void enc_refresh_param_label(param_card_t *p) {
    if (!p || !p->label_value) return;
    char buf[16];
    if (p->unit[0] == 's') {
        snprintf(buf, sizeof(buf), "%.1f%s", *p->value_ptr, p->unit);
    } else {
        format_ms_value(buf, sizeof(buf), *p->value_ptr);
    }
    lv_label_set_text(p->label_value, buf);
}

// ============================================================================
// Encoder Event Handler
// ============================================================================

// Forward declaration — defined in Main Dashboard section below
static void btn_mode_toggle_cb(lv_event_t *e);

static void enc_handle_event(encoder_event_t evt) {
    // First rotation activates encoder focus if not yet active
    if (s_enc_focus < 0) {
        if (evt == ENC_EVENT_CW || evt == ENC_EVENT_CCW) {
            s_enc_mode = ENC_MODE_NAVIGATE;
            enc_set_focus(0);
            audio_play_beep();
            return;
        }
        return;  // Ignore KEY press when no focus
    }

    int real = enc_logical_to_real(s_enc_focus);
    focus_item_t *items = enc_current_items();
    focus_item_t *fi = &items[real];

    if (s_enc_mode == ENC_MODE_NAVIGATE) {
        // ── NAVIGATE mode ──
        if (evt == ENC_EVENT_CW) {
            enc_set_focus(s_enc_focus + 1);
            audio_play_beep();
        } else if (evt == ENC_EVENT_CCW) {
            enc_set_focus(s_enc_focus - 1);
            audio_play_beep();
        } else if (evt == ENC_EVENT_PRESS) {
            switch (fi->type) {
                case FTYPE_PARAM:
                case FTYPE_SLIDER:
                case FTYPE_DROPDOWN:
                    // Enter edit mode
                    s_enc_mode = ENC_MODE_EDIT;
                    enc_update_focus_visual();
                    break;

                case FTYPE_TOGGLE:
                    // Toggle immediately
                    if (fi->widget == lbl_mode) {
                        btn_mode_toggle_cb(NULL);
                        // Mode changed — recalculate MODE's logical index.
                        // MODE is always real index 6; find which logical maps to it.
                        int mode_logical = 0;
                        int max = enc_current_count();
                        for (int i = 0; i < max; i++) {
                            if (enc_logical_to_real(i) == 6) { mode_logical = i; break; }
                        }
                        enc_set_focus(mode_logical);
                    } else if (fi->widget == sw_sound_global) {
                        if (lv_obj_has_state(sw_sound_global, LV_STATE_CHECKED)) {
                            lv_obj_clear_state(sw_sound_global, LV_STATE_CHECKED);
                        } else {
                            lv_obj_add_state(sw_sound_global, LV_STATE_CHECKED);
                        }
                        // Trigger the callback
                        lv_obj_send_event(sw_sound_global, LV_EVENT_VALUE_CHANGED, NULL);
                    } else if (fi->widget == sw_theme_global) {
                        if (lv_obj_has_state(sw_theme_global, LV_STATE_CHECKED)) {
                            lv_obj_clear_state(sw_theme_global, LV_STATE_CHECKED);
                        } else {
                            lv_obj_add_state(sw_theme_global, LV_STATE_CHECKED);
                        }
                        lv_obj_send_event(sw_theme_global, LV_EVENT_VALUE_CHANGED, NULL);
                    }
                    audio_play_beep();
                    break;

                case FTYPE_ACTION:
                    // Execute action
                    if (fi->widget == s_btn_settings) {
                        audio_play_beep();
                        lv_scr_load_anim(scr_settings, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
                        s_enc_screen = ENC_SCREEN_SETTINGS;
                        s_enc_mode = ENC_MODE_NAVIGATE;
                        enc_clear_all_glow();
                        enc_set_focus(0);
                    } else if (fi->widget == s_btn_back) {
                        audio_play_beep();
                        lv_scr_load_anim(scr_main, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
                        s_enc_screen = ENC_SCREEN_MAIN;
                        s_enc_mode = ENC_MODE_NAVIGATE;
                        enc_clear_all_glow();
                        enc_set_focus(0);
                    }
                    break;
            }
        }
    } else {
        // ── EDIT mode ──
        if (evt == ENC_EVENT_PRESS) {
            // Confirm — save and return to navigate
            s_enc_mode = ENC_MODE_NAVIGATE;
            settings_save();
            enc_update_focus_visual();
            audio_play_beep();
        } else if (evt == ENC_EVENT_CW || evt == ENC_EVENT_CCW) {
            int dir = (evt == ENC_EVENT_CW) ? 1 : -1;

            switch (fi->type) {
                case FTYPE_PARAM: {
                    param_card_t *p = fi->param;
                    if (!p) break;
                    *p->value_ptr += dir * p->step;
                    if (*p->value_ptr > p->max_val) *p->value_ptr = p->max_val;
                    if (*p->value_ptr < p->min_val) *p->value_ptr = p->min_val;
                    enc_refresh_param_label(p);
                    audio_play_beep();
                    break;
                }
                case FTYPE_SLIDER: {
                    lv_obj_t *slider = fi->widget;
                    int32_t val = lv_slider_get_value(slider) + dir * fi->slider_step;
                    int32_t mn = lv_slider_get_min_value(slider);
                    int32_t mx = lv_slider_get_max_value(slider);
                    if (val < mn) val = mn;
                    if (val > mx) val = mx;
                    lv_slider_set_value(slider, val, LV_ANIM_ON);
                    lv_obj_send_event(slider, LV_EVENT_VALUE_CHANGED, NULL);
                    break;
                }
                case FTYPE_DROPDOWN: {
                    lv_obj_t *dd = fi->widget;
                    int32_t sel = (int32_t)lv_dropdown_get_selected(dd);
                    int32_t cnt = (int32_t)lv_dropdown_get_option_count(dd);
                    sel += dir;
                    if (sel < 0) sel = cnt - 1;
                    if (sel >= cnt) sel = 0;
                    lv_dropdown_set_selected(dd, (uint16_t)sel);
                    lv_obj_send_event(dd, LV_EVENT_VALUE_CHANGED, NULL);
                    break;
                }
                default:
                    break;
            }
        }
    }
}

// ============================================================================
// Parameter Card Creation
// ============================================================================


static void param_btn_cb(lv_event_t *e) {
  param_card_t *card = (param_card_t *)lv_event_get_user_data(e);
  lv_obj_t *target = lv_event_get_target(e);

  if (target == card->btn_plus || target == card->card) {
    // Tapping the card body acts like "+" for easy touch
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
  // Make the card body itself clickable for easy touch response
  lv_obj_add_flag(card->card, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(card->card, param_btn_cb, LV_EVENT_CLICKED, card);

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
  lv_obj_align(card->label_value, LV_ALIGN_CENTER, 0, -2);

  // Minus button – wide for touch, short to avoid covering value
  card->btn_minus = lv_btn_create(card->card);
  lv_obj_set_size(card->btn_minus, 48, 28);
  lv_obj_align(card->btn_minus, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_bg_color(card->btn_minus, COLOR_ACCENT, 0);
  lv_obj_set_style_radius(card->btn_minus, 8, 0);
  lv_obj_add_event_cb(card->btn_minus, param_btn_cb, LV_EVENT_CLICKED, card);
  lv_obj_t *lbl_m = lv_label_create(card->btn_minus);
  lv_label_set_text(lbl_m, LV_SYMBOL_MINUS);
  lv_obj_center(lbl_m);

  // Plus button – wide for touch, short to avoid covering value
  card->btn_plus = lv_btn_create(card->card);
  lv_obj_set_size(card->btn_plus, 48, 28);
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

// Forward declaration
static void update_dashboard_layout(void);

static void btn_mode_toggle_cb(lv_event_t *e) {
  (void)e;
  g_settings.auto_mode = !g_settings.auto_mode;
  lv_label_set_text(lbl_mode, g_settings.auto_mode ? "AUTO" : "MAN");
  lv_obj_set_style_text_color(
      lbl_mode, g_settings.auto_mode ? COLOR_BLUE : COLOR_GREEN, 0);

  update_dashboard_layout();
  audio_play_beep();
  settings_save();
}

/**
 * Recalculate dashboard card visibility and layout positions.
 *
 * Layout rules (480×320 display, status bar=36px at y=0, bottom bar=44px at y=276):
 *   Row 1 (y=42, h=90): P1, T, P2 — always visible in User Defined mode
 *   Row 2 (y=138, h=90): P3, P4   — visible only when value > 0 in UD mode
 *   S delay card: visible only in AUTO mode, positioned below the last param row
 *   Voltage bar and chart cascade below S (or below params if no S)
 *
 *   Preset mode: all pulse cards hidden, S/vbar/chart shift up
 */
static void update_dashboard_layout(void) {
  bool ud = is_user_defined();
  bool am = g_settings.auto_mode;

  // ── Row 1: P1 / T / P2 — visible only in User Defined mode ──
  if (card_p1.card) {
    if (ud) lv_obj_clear_flag(card_p1.card, LV_OBJ_FLAG_HIDDEN);
    else    lv_obj_add_flag(card_p1.card, LV_OBJ_FLAG_HIDDEN);
  }
  if (card_t.card) {
    if (ud) lv_obj_clear_flag(card_t.card, LV_OBJ_FLAG_HIDDEN);
    else    lv_obj_add_flag(card_t.card, LV_OBJ_FLAG_HIDDEN);
  }
  if (card_p2.card) {
    if (ud) lv_obj_clear_flag(card_p2.card, LV_OBJ_FLAG_HIDDEN);
    else    lv_obj_add_flag(card_p2.card, LV_OBJ_FLAG_HIDDEN);
  }

  // ── Row 2: P3 / P4 — visible only in UD mode when value > 0 ──
  bool p3_visible = ud && g_settings.p3 > 0.0f;
  bool p4_visible = ud && g_settings.p4 > 0.0f;

  if (card_p3.card) {
    if (p3_visible) lv_obj_clear_flag(card_p3.card, LV_OBJ_FLAG_HIDDEN);
    else            lv_obj_add_flag(card_p3.card, LV_OBJ_FLAG_HIDDEN);
  }
  if (card_p4.card) {
    if (p4_visible) lv_obj_clear_flag(card_p4.card, LV_OBJ_FLAG_HIDDEN);
    else            lv_obj_add_flag(card_p4.card, LV_OBJ_FLAG_HIDDEN);
  }

  // ── S delay card: visible only in AUTO mode ──
  if (card_s.card) {
    if (am) lv_obj_clear_flag(card_s.card, LV_OBJ_FLAG_HIDDEN);
    else    lv_obj_add_flag(card_s.card, LV_OBJ_FLAG_HIDDEN);
  }

  // ── Cascading Y positions ──
  // Compute the next available Y position after the visible parameter rows
  int next_y = 42;  // Start just below the status bar

  if (ud) {
    next_y = 138;  // After Row 1 (y=42, h=90, gap=6)
    if (p3_visible || p4_visible) {
      next_y = 234;  // After Row 2 (y=138, h=90, gap=6)
    }
  }

  // S card position
  if (am && card_s.card) {
    lv_obj_set_pos(card_s.card, 6, next_y);
    next_y += 96;  // S card h=90 + gap=6
  }

  // Voltage bar
  if (volt_container) {
    lv_obj_set_pos(volt_container, 10, next_y);
    next_y += 46;  // vbar h=40 + gap=6
  }

  // Chart — show if there's enough vertical space (need ~80px before bottom bar at y=276)
  if (chart_voltage) {
    if (next_y + 80 <= 276) {
      lv_obj_clear_flag(chart_voltage, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_pos(chart_voltage, 10, next_y);
      int chart_h = 276 - next_y - 4;  // Dynamic height to fill available space
      if (chart_h < 50) chart_h = 50;
      if (chart_h > 120) chart_h = 120;
      lv_obj_set_height(chart_voltage, chart_h);
    } else {
      lv_obj_add_flag(chart_voltage, LV_OBJ_FLAG_HIDDEN);
    }
  }

  // Update preset name in status bar
  if (lbl_preset) {
    if (ud) {
      lv_label_set_text(lbl_preset, "User Defined");
    } else {
      lv_label_set_text(lbl_preset, g_settings.presets[g_settings.active_preset].name);
    }
  }
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

  // Preset name (or "User Defined")
  lbl_preset = lv_label_create(status_bar);
  if (is_user_defined()) {
    lv_label_set_text(lbl_preset, "User Defined");
  } else {
    lv_label_set_text(lbl_preset,
                      g_settings.presets[g_settings.active_preset].name);
  }
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

  // BLE connected icon (hidden until a client connects)
  lbl_ble_icon = lv_label_create(status_bar);
  lv_label_set_text(lbl_ble_icon, LV_SYMBOL_BLUETOOTH);
  lv_obj_set_style_text_color(lbl_ble_icon, COLOR_BLUE, 0);
  lv_obj_set_style_text_font(lbl_ble_icon, &lv_font_montserrat_16, 0);
  lv_obj_align(lbl_ble_icon, LV_ALIGN_RIGHT_MID, -58, 0);
  lv_obj_add_flag(lbl_ble_icon, LV_OBJ_FLAG_HIDDEN);



  // ── Parameter Cards ──────────────────────────────
  int card_y = 42;
  int card_w = 148;
  int gap = 6;

  create_param_card(scr_main, &card_p1, "PULSE 1", &g_settings.p1, PULSE_MIN_MS,
                    PULSE_MAX_MS, PULSE_STEP_MS, "ms", gap, card_y, card_w);

  create_param_card(scr_main, &card_t, "PAUSE", &g_settings.t, PAUSE_MIN_MS,
                    PAUSE_MAX_MS, PAUSE_STEP_MS, "ms", gap + card_w + gap,
                    card_y, card_w);

  create_param_card(scr_main, &card_p2, "PULSE 2", &g_settings.p2, PULSE_MIN_MS,
                    PULSE_MAX_MS, PULSE_STEP_MS, "ms", gap + (card_w + gap) * 2,
                    card_y, card_w);

  // ── Row 2: P3 (Forge) and P4 (Temper) — only shown when value > 0 ──
  int row2_y = card_y + 96;  // 42 + 90 + 6 = 138

  create_param_card(scr_main, &card_p3, "FORGE", &g_settings.p3, PULSE_MIN_MS,
                    PULSE_MAX_MS, PULSE_STEP_MS, "ms", gap, row2_y, card_w);

  create_param_card(scr_main, &card_p4, "TEMPER", &g_settings.p4, PULSE_MIN_MS,
                    PULSE_MAX_MS, PULSE_STEP_MS, "ms", gap + card_w + gap,
                    row2_y, card_w);

  // S value card (AUTO mode only — position set by update_dashboard_layout)
  create_param_card(scr_main, &card_s, "DELAY (S)", &g_settings.s_value,
                    S_VALUE_MIN, S_VALUE_MAX, S_VALUE_STEP, "s", gap,
                    row2_y + 96, 220);

  // Apply initial visibility for all cards based on mode
  update_dashboard_layout();

  // ── Voltage Bar ──────────────────────────────────
  // Position is set by update_dashboard_layout(), use 142 as placeholder.
  int vbar_y = 142;

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

  // Charger status icon (thunder bolt = charging, X = not charging)
  lbl_charger_icon = lv_label_create(volt_container);
  lv_label_set_text(lbl_charger_icon, LV_SYMBOL_CHARGE);
  lv_obj_set_style_text_color(lbl_charger_icon, COLOR_YELLOW, 0);
  lv_obj_set_style_text_font(lbl_charger_icon, &lv_font_montserrat_20, 0);
  lv_obj_align(lbl_charger_icon, LV_ALIGN_LEFT_MID, 330, 0);

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

  // Chart visibility is managed by update_dashboard_layout() called above.
  // Re-apply now that chart_voltage exists.
  update_dashboard_layout();

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
  s_btn_settings = lv_btn_create(bottom_bar);
  lv_obj_set_size(s_btn_settings, 40, 32);
  lv_obj_align(s_btn_settings, LV_ALIGN_RIGHT_MID, 0, 0);
  lv_obj_set_style_bg_color(s_btn_settings, COLOR_ACCENT, 0);
  lv_obj_set_style_radius(s_btn_settings, 8, 0);
  lv_obj_add_event_cb(s_btn_settings, btn_settings_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_gear = lv_label_create(s_btn_settings);
  lv_label_set_text(lbl_gear, LV_SYMBOL_SETTINGS);
  lv_obj_center(lbl_gear);

  // ── Register encoder-focusable items (order = navigation order) ──
  // Real indices: P1(0) T(1) P2(2) P3(3) P4(4) S(5) MODE(6) SETTINGS(7)
  s_main_count = 0;
  s_main_items[s_main_count++] = (focus_item_t){ .widget = card_p1.card, .type = FTYPE_PARAM, .param = &card_p1 };
  s_main_items[s_main_count++] = (focus_item_t){ .widget = card_t.card,  .type = FTYPE_PARAM, .param = &card_t  };
  s_main_items[s_main_count++] = (focus_item_t){ .widget = card_p2.card, .type = FTYPE_PARAM, .param = &card_p2 };
  s_main_items[s_main_count++] = (focus_item_t){ .widget = card_p3.card, .type = FTYPE_PARAM, .param = &card_p3 };  // Hidden when p3==0
  s_main_items[s_main_count++] = (focus_item_t){ .widget = card_p4.card, .type = FTYPE_PARAM, .param = &card_p4 };  // Hidden when p4==0
  s_main_items[s_main_count++] = (focus_item_t){ .widget = card_s.card,  .type = FTYPE_PARAM, .param = &card_s  };  // Hidden in MAN mode
  s_main_items[s_main_count++] = (focus_item_t){ .widget = lbl_mode,     .type = FTYPE_TOGGLE };
  s_main_items[s_main_count++] = (focus_item_t){ .widget = s_btn_settings, .type = FTYPE_ACTION };
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

  if (sel == 0) {
    // "User Defined" — keep current pulse values, unlock editing
    settings_set_user_defined();
  } else {
    // Load preset (dropdown index 1 → preset[0], etc.)
    settings_load_preset(sel - 1);
  }

  // Sync card labels from g_settings (may have changed from preset load)
  char buf[16];
  format_ms_value(buf, sizeof(buf), g_settings.p1);
  lv_label_set_text(card_p1.label_value, buf);
  format_ms_value(buf, sizeof(buf), g_settings.t);
  lv_label_set_text(card_t.label_value, buf);
  format_ms_value(buf, sizeof(buf), g_settings.p2);
  lv_label_set_text(card_p2.label_value, buf);
  format_ms_value(buf, sizeof(buf), g_settings.p3);
  lv_label_set_text(card_p3.label_value, buf);
  format_ms_value(buf, sizeof(buf), g_settings.p4);
  lv_label_set_text(card_p4.label_value, buf);
  snprintf(buf, sizeof(buf), "%.1fs", g_settings.s_value);
  lv_label_set_text(card_s.label_value, buf);

  // Refresh dashboard layout (show/hide cards based on mode and values)
  update_dashboard_layout();

  audio_play_beep();
}

static void create_settings_screen(void) {
  scr_settings = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_settings, COLOR_BG_DARK, 0);
  lv_obj_set_style_pad_all(scr_settings, 10, 0);

  // Back button
  s_btn_back = lv_btn_create(scr_settings);
  lv_obj_set_size(s_btn_back, 80, 32);
  lv_obj_set_pos(s_btn_back, 0, 0);
  lv_obj_set_style_bg_color(s_btn_back, COLOR_ACCENT, 0);
  lv_obj_set_style_radius(s_btn_back, 8, 0);
  lv_obj_add_event_cb(s_btn_back, btn_back_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_back = lv_label_create(s_btn_back);
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

  // Build preset options string: "User Defined" at index 0, then presets
  char preset_opts[(MAX_PRESETS + 1) * (PRESET_NAME_LEN + 1)];
  strcpy(preset_opts, "User Defined");
  for (int i = 0; i < MAX_PRESETS; i++) {
    strcat(preset_opts, "\n");
    strcat(preset_opts, g_settings.presets[i].name);
  }

  s_dd_preset = lv_dropdown_create(scr_settings);
  lv_dropdown_set_options(s_dd_preset, preset_opts);
  // Map active_preset to dropdown index: PRESET_USER_DEFINED → 0, else preset+1
  uint16_t dd_sel = is_user_defined() ? 0 : (g_settings.active_preset + 1);
  lv_dropdown_set_selected(s_dd_preset, dd_sel);
  lv_obj_set_size(s_dd_preset, 280, 36);
  lv_obj_set_pos(s_dd_preset, 180, row_y);
  lv_obj_add_event_cb(s_dd_preset, preset_dropdown_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  row_y += row_h + 5;

  // ── Brightness Slider ────────────────────────────
  lv_obj_t *lbl_br = lv_label_create(scr_settings);
  lv_label_set_text(lbl_br, LV_SYMBOL_IMAGE " Brightness:");
  lv_obj_set_style_text_color(lbl_br, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_br, 10, row_y + 6);

  slider_brightness = lv_slider_create(scr_settings);
  lv_obj_set_size(slider_brightness, 250, 16);
  lv_obj_set_pos(slider_brightness, 200, row_y + 8);
  lv_slider_set_range(slider_brightness, 10, 100);
  lv_slider_set_value(slider_brightness, g_settings.brightness, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(slider_brightness, COLOR_ACCENT, LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider_brightness, COLOR_PRIMARY, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider_brightness, COLOR_TEXT_LIGHT, LV_PART_KNOB);
  lv_obj_add_event_cb(slider_brightness, slider_brightness_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  row_y += row_h;

  // ── Volume Slider ───────────────────────────────
  lv_obj_t *lbl_vol = lv_label_create(scr_settings);
  lv_label_set_text(lbl_vol, LV_SYMBOL_VOLUME_MAX " Volume:");
  lv_obj_set_style_text_color(lbl_vol, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_vol, 10, row_y + 6);

  slider_volume = lv_slider_create(scr_settings);
  lv_obj_set_size(slider_volume, 250, 16);
  lv_obj_set_pos(slider_volume, 200, row_y + 8);
  lv_slider_set_range(slider_volume, 0, 100);
  lv_slider_set_value(slider_volume, g_settings.volume, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(slider_volume, COLOR_ACCENT, LV_PART_MAIN);
  lv_obj_set_style_bg_color(slider_volume, COLOR_BLUE, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(slider_volume, COLOR_TEXT_LIGHT, LV_PART_KNOB);
  lv_obj_add_event_cb(slider_volume, slider_volume_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);
  row_y += row_h;

  // ── Sound Toggle ─────────────────────────────────
  lv_obj_t *lbl_snd = lv_label_create(scr_settings);
  lv_label_set_text(lbl_snd, LV_SYMBOL_AUDIO " Sound:");
  lv_obj_set_style_text_color(lbl_snd, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_snd, 10, row_y + 6);

  sw_sound_global = lv_switch_create(scr_settings);
  lv_obj_set_pos(sw_sound_global, 410, row_y + 2);
  if (g_settings.sound_on)
    lv_obj_add_state(sw_sound_global, LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(sw_sound_global, COLOR_GREEN,
                            LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(sw_sound_global, sw_sound_cb, LV_EVENT_VALUE_CHANGED, NULL);
  row_y += row_h;

  // ── Theme Toggle ─────────────────────────────────
  lv_obj_t *lbl_thm = lv_label_create(scr_settings);
  lv_label_set_text(lbl_thm, LV_SYMBOL_EYE_OPEN " Light Mode:");
  lv_obj_set_style_text_color(lbl_thm, COLOR_TEXT_LIGHT, 0);
  lv_obj_set_pos(lbl_thm, 10, row_y + 6);

  sw_theme_global = lv_switch_create(scr_settings);
  lv_obj_set_pos(sw_theme_global, 410, row_y + 2);
  if (g_settings.theme == 1)
    lv_obj_add_state(sw_theme_global, LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(sw_theme_global, COLOR_YELLOW,
                            LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_event_cb(sw_theme_global, sw_theme_cb, LV_EVENT_VALUE_CHANGED, NULL);
  row_y += row_h;

  // ── Version Info ─────────────────────────────────
  row_y += 10;
  lv_obj_t *lbl_ver = lv_label_create(scr_settings);
  lv_label_set_text_fmt(lbl_ver, "MyWeld ESP32 v%s | %s", FW_VERSION_STRING,
                        FW_BUILD_DATE);
  lv_obj_set_style_text_color(lbl_ver, COLOR_TEXT_DIM, 0);
  lv_obj_set_style_text_font(lbl_ver, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_ver, LV_ALIGN_BOTTOM_MID, 0, -5);

  // ── Register encoder-focusable items (order = navigation order) ──
  s_settings_count = 0;
  s_settings_items[s_settings_count++] = (focus_item_t){ .widget = s_btn_back,       .type = FTYPE_ACTION };
  s_settings_items[s_settings_count++] = (focus_item_t){ .widget = s_dd_preset,      .type = FTYPE_DROPDOWN };
  s_settings_items[s_settings_count++] = (focus_item_t){ .widget = slider_brightness, .type = FTYPE_SLIDER, .slider_step = 5 };
  s_settings_items[s_settings_count++] = (focus_item_t){ .widget = slider_volume,     .type = FTYPE_SLIDER, .slider_step = 5 };
  s_settings_items[s_settings_count++] = (focus_item_t){ .widget = sw_sound_global,   .type = FTYPE_TOGGLE };
  s_settings_items[s_settings_count++] = (focus_item_t){ .widget = sw_theme_global,   .type = FTYPE_TOGGLE };
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
// Countdown Timer Callback (reboot / factory-reset sequence)
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
        ESP_LOGW(TAG, "Reboot countdown complete — rebooting now");
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

          // Update charger icon: ⚡ when charging, ✕ when not
          if (lbl_charger_icon) {
            bool charging = welding_is_charging();
            lv_label_set_text(lbl_charger_icon,
                              charging ? LV_SYMBOL_CHARGE : LV_SYMBOL_CLOSE);
            lv_obj_set_style_text_color(lbl_charger_icon,
                              charging ? COLOR_YELLOW : COLOR_TEXT_DIM, 0);
          }
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
          if (card_p3.label_value) {
            format_ms_value(buf, sizeof(buf), g_settings.p3);
            lv_label_set_text(card_p3.label_value, buf);
          }
          if (card_p4.label_value) {
            format_ms_value(buf, sizeof(buf), g_settings.p4);
            lv_label_set_text(card_p4.label_value, buf);
          }
          if (card_s.label_value) {
            snprintf(buf, sizeof(buf), "%.1fs", g_settings.s_value);
            lv_label_set_text(card_s.label_value, buf);
          }
          // Sync mode label
          if (lbl_mode) {
            lv_label_set_text(lbl_mode, g_settings.auto_mode ? "AUTO" : "MAN");
            lv_obj_set_style_text_color(
                lbl_mode, g_settings.auto_mode ? COLOR_BLUE : COLOR_GREEN, 0);
          }
          // Refresh dashboard: show/hide cards, reposition, update preset name
          update_dashboard_layout();
          // Sync settings screen widgets (brightness, volume, sound, theme)
          if (slider_brightness) {
            lv_slider_set_value(slider_brightness, g_settings.brightness, LV_ANIM_OFF);
          }
          display_set_brightness(g_settings.brightness);
          if (slider_volume) {
            lv_slider_set_value(slider_volume, g_settings.volume, LV_ANIM_OFF);
          }
          if (sw_sound_global) {
            if (g_settings.sound_on) {
              lv_obj_add_state(sw_sound_global, LV_STATE_CHECKED);
            } else {
              lv_obj_clear_state(sw_sound_global, LV_STATE_CHECKED);
            }
          }
          if (sw_theme_global) {
            if (g_settings.theme == 1) {
              lv_obj_add_state(sw_theme_global, LV_STATE_CHECKED);
            } else {
              lv_obj_clear_state(sw_theme_global, LV_STATE_CHECKED);
            }
          }
          // Sync preset dropdown to current active_preset
          if (s_dd_preset) {
            uint16_t dd_sel = is_user_defined() ? 0 : (g_settings.active_preset + 1);
            lv_dropdown_set_selected(s_dd_preset, dd_sel);
          }
          ESP_LOGI(TAG, "UI params refreshed from BLE (P1=%.1f T=%.1f P2=%.1f S=%.1f %s br=%d vol=%d)",
                   g_settings.p1, g_settings.t, g_settings.p2, g_settings.s_value,
                   g_settings.auto_mode ? "AUTO" : "MAN",
                   g_settings.brightness, g_settings.volume);
          break;
        }

        case UI_MSG_REBOOT_COUNTDOWN: {
          // ── Build full-screen countdown overlay ────────────────────────────────────────
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
          bool factory = msg.is_factory_reset;

          // Full-screen dark overlay
          scr_countdown = lv_obj_create(NULL);
          lv_obj_set_style_bg_color(scr_countdown, lv_color_hex(0x0D0D1A), 0);
          lv_obj_clear_flag(scr_countdown, LV_OBJ_FLAG_SCROLLABLE);

          // Header label — contextual title based on reset type
          lv_obj_t *lbl_title = lv_label_create(scr_countdown);
          if (factory) {
            lv_label_set_text(lbl_title, LV_SYMBOL_WARNING " Factory Reset");
            lv_obj_set_style_text_color(lbl_title, lv_color_hex(0xFF4444), 0);
          } else {
            lv_label_set_text(lbl_title, LV_SYMBOL_REFRESH " Rebooting");
            lv_obj_set_style_text_color(lbl_title, COLOR_BLUE, 0);
          }
          lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_24, 0);
          lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 30);

          // Sub-label "Restarting in..."
          lv_obj_t *lbl_sub = lv_label_create(scr_countdown);
          lv_label_set_text(lbl_sub, "Restarting in...");
          lv_obj_set_style_text_color(lbl_sub, lv_color_hex(0xAAAAAA), 0);
          lv_obj_set_style_text_font(lbl_sub, &lv_font_montserrat_16, 0);
          lv_obj_align(lbl_sub, LV_ALIGN_CENTER, 0, -30);

          // Large countdown digit
          lbl_countdown = lv_label_create(scr_countdown);
          lv_label_set_text(lbl_countdown, "3");
          lv_obj_set_style_text_color(lbl_countdown,
              factory ? lv_color_hex(0xFF4444) : COLOR_BLUE, 0);
          lv_obj_set_style_text_font(lbl_countdown, &lv_font_montserrat_48, 0);
          lv_obj_align(lbl_countdown, LV_ALIGN_CENTER, 0, 30);

          // Load the overlay immediately
          lv_scr_load(scr_countdown);

          // Beep to alert the user
          audio_play_beep();

          // LVGL timer fires every 1 second — callback decrements the digit
          tmr_countdown = lv_timer_create(countdown_timer_cb, 1000, NULL);

          ESP_LOGI(TAG, "%s countdown started (3s)",
                   factory ? "Factory reset" : "Reboot");
          break;
        }

        default: break;

        case UI_MSG_OTA_PROGRESS: {
          uint8_t pct = msg.ota_progress;

          // Create OTA overlay screen on first call
          if (!scr_ota) {
            scr_ota = lv_obj_create(NULL);
            lv_obj_set_style_bg_color(scr_ota, lv_color_hex(0x0D0D1A), 0);
            lv_obj_clear_flag(scr_ota, LV_OBJ_FLAG_SCROLLABLE);

            // Title
            lv_obj_t *lbl_title = lv_label_create(scr_ota);
            lv_label_set_text(lbl_title, LV_SYMBOL_DOWNLOAD " Firmware Update");
            lv_obj_set_style_text_color(lbl_title, COLOR_BLUE, 0);
            lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_24, 0);
            lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 40);

            // Progress bar
            bar_ota = lv_bar_create(scr_ota);
            lv_obj_set_size(bar_ota, 360, 30);
            lv_obj_align(bar_ota, LV_ALIGN_CENTER, 0, -10);
            lv_bar_set_range(bar_ota, 0, 100);
            lv_bar_set_value(bar_ota, 0, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(bar_ota, lv_color_hex(0x333333), LV_PART_MAIN);
            lv_obj_set_style_bg_color(bar_ota, COLOR_BLUE, LV_PART_INDICATOR);
            lv_obj_set_style_radius(bar_ota, 8, LV_PART_MAIN);
            lv_obj_set_style_radius(bar_ota, 8, LV_PART_INDICATOR);

            // Percentage label
            lbl_ota_pct = lv_label_create(scr_ota);
            lv_label_set_text(lbl_ota_pct, "0%");
            lv_obj_set_style_text_color(lbl_ota_pct, COLOR_TEXT_LIGHT, 0);
            lv_obj_set_style_text_font(lbl_ota_pct, &lv_font_montserrat_48, 0);
            lv_obj_align(lbl_ota_pct, LV_ALIGN_CENTER, 0, 50);

            // Warning
            lv_obj_t *lbl_warn = lv_label_create(scr_ota);
            lv_label_set_text(lbl_warn, LV_SYMBOL_WARNING " DO NOT POWER OFF");
            lv_obj_set_style_text_color(lbl_warn, COLOR_RED, 0);
            lv_obj_set_style_text_font(lbl_warn, &lv_font_montserrat_16, 0);
            lv_obj_align(lbl_warn, LV_ALIGN_BOTTOM_MID, 0, -30);

            lv_scr_load(scr_ota);
          }

          // Update progress
          if (bar_ota) lv_bar_set_value(bar_ota, pct, LV_ANIM_ON);
          if (lbl_ota_pct) lv_label_set_text_fmt(lbl_ota_pct, "%d%%", pct);

          // Change bar color to green when complete
          if (pct >= 100 && bar_ota) {
            lv_obj_set_style_bg_color(bar_ota, COLOR_GREEN, LV_PART_INDICATOR);
          }
          break;
        }



        case UI_MSG_OTA_HIDE: {
          if (scr_ota) {
            lv_scr_load(scr_main);
            lv_obj_del(scr_ota);
            scr_ota = NULL;
            lbl_ota_pct = NULL;
            bar_ota = NULL;
          }
          break;
        }
      }
    }
    // ── End queue drain ─────────────────────────────────────────────────────

    // ── Poll rotary encoder ────────────────────────────────────────────────
    encoder_event_t enc_evt;
    while (encoder_poll(&enc_evt)) {
        enc_handle_event(enc_evt);
    }

    // ── BLE connected icon: show/hide based on live connection state ───────
    if (lbl_ble_icon) {
      if (ble_serial_is_connected()) {
        lv_obj_clear_flag(lbl_ble_icon, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(lbl_ble_icon, LV_OBJ_FLAG_HIDDEN);
      }
    }

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

void ui_trigger_reboot_countdown(bool is_factory_reset) {
  if (!s_ui_queue) {
    // Queue not up yet — reboot immediately as last resort
    esp_restart();
    return;
  }
  ui_msg_t msg = { .type = UI_MSG_REBOOT_COUNTDOWN, .is_factory_reset = is_factory_reset };
  xQueueSend(s_ui_queue, &msg, pdMS_TO_TICKS(50));
}

void ui_show_ota_progress(uint8_t percent) {
  if (!s_ui_queue) return;
  ui_msg_t msg = { .type = UI_MSG_OTA_PROGRESS, .ota_progress = percent };
  xQueueSend(s_ui_queue, &msg, pdMS_TO_TICKS(5));
}

void ui_hide_ota_progress(void) {
  if (!s_ui_queue) return;
  ui_msg_t msg = { .type = UI_MSG_OTA_HIDE };
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
