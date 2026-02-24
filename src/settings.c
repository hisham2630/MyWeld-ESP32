/**
 * Settings Module — Full NVS Persistence
 * 
 * Uses ESP-IDF nvs_flash API to store/load all parameters.
 * Debounced auto-save prevents flash wear.
 */

#include "settings.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "Settings";
static const char *NVS_NAMESPACE = "myweld";

app_settings_t g_settings;

static nvs_handle_t s_nvs_handle = 0;
static esp_timer_handle_t s_save_timer = NULL;
static bool s_dirty = false;

// Factory default presets
static const weld_preset_t factory_presets[MAX_PRESETS] = {
    { "0.1mm Nickel",     3.0f,  5.0f,  5.0f,  0.5f, false },
    { "0.15mm Nickel",    5.0f,  8.0f,  8.0f,  0.5f, false },
    { "0.2mm Nickel",     8.0f, 10.0f, 12.0f,  0.5f, false },
    { "0.3mm Nickel",    12.0f, 12.0f, 18.0f,  0.6f, false },
    { "0.5mm Nickel",    18.0f, 15.0f, 25.0f,  0.8f, false },
    { "Stainless Steel", 20.0f, 15.0f, 30.0f,  0.8f, false },
    { "Aluminum",        25.0f, 12.0f, 35.0f,  0.8f, false },
    { "Custom 1",        10.0f, 10.0f, 10.0f,  0.5f, false },
    { "Custom 2",        10.0f, 10.0f, 10.0f,  0.5f, false },
    { "Custom 3",        10.0f, 10.0f, 10.0f,  0.5f, false },
};

static void settings_load_defaults(void)
{
    g_settings.p1 = PULSE_DEFAULT_P1;
    g_settings.t  = PULSE_DEFAULT_T;
    g_settings.p2 = PULSE_DEFAULT_P2;
    g_settings.s_value = S_VALUE_DEFAULT;
    g_settings.auto_mode = false;
    g_settings.brightness = 80;
    g_settings.sound_on = true;
    g_settings.theme = 0;
    g_settings.session_welds = 0;
    g_settings.total_welds = 0;
    g_settings.active_preset = 1;
    memcpy(g_settings.presets, factory_presets, sizeof(factory_presets));
    g_settings.adc_cal_voltage = 1.0f;
    g_settings.adc_cal_protection = 1.0f;
    strncpy(g_settings.ble_name, BLE_DEVICE_NAME, sizeof(g_settings.ble_name) - 1);
    g_settings.ble_name[sizeof(g_settings.ble_name) - 1] = '\0';
    strncpy(g_settings.pin, PIN_DEFAULT, PIN_MAX_LEN - 1);
    g_settings.pin[PIN_MAX_LEN - 1] = '\0';
}

// ============================================================================
// NVS Read/Write Helpers
// ============================================================================

static float nvs_get_float(const char *key, float def)
{
    int32_t raw = 0;
    if (nvs_get_i32(s_nvs_handle, key, &raw) == ESP_OK) {
        // Store floats as int32 × 1000 for precision
        return (float)raw / 1000.0f;
    }
    return def;
}

static void nvs_set_float(const char *key, float val)
{
    int32_t raw = (int32_t)(val * 1000.0f);
    nvs_set_i32(s_nvs_handle, key, raw);
}

static void settings_load_from_nvs(void)
{
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &s_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS namespace not found, using defaults");
        settings_load_defaults();
        return;
    }

    g_settings.p1 = nvs_get_float("p1", PULSE_DEFAULT_P1);
    g_settings.t  = nvs_get_float("t", PULSE_DEFAULT_T);
    g_settings.p2 = nvs_get_float("p2", PULSE_DEFAULT_P2);
    g_settings.s_value = nvs_get_float("sValue", S_VALUE_DEFAULT);

    uint8_t u8val;
    if (nvs_get_u8(s_nvs_handle, "autoMode", &u8val) == ESP_OK) g_settings.auto_mode = u8val;
    if (nvs_get_u8(s_nvs_handle, "bright", &u8val) == ESP_OK) g_settings.brightness = u8val;
    if (nvs_get_u8(s_nvs_handle, "soundOn", &u8val) == ESP_OK) g_settings.sound_on = u8val;
    if (nvs_get_u8(s_nvs_handle, "theme", &u8val) == ESP_OK) g_settings.theme = u8val;
    if (nvs_get_u8(s_nvs_handle, "actPreset", &u8val) == ESP_OK) g_settings.active_preset = u8val;

    uint32_t u32val;
    if (nvs_get_u32(s_nvs_handle, "totalWeld", &u32val) == ESP_OK) g_settings.total_welds = u32val;

    g_settings.adc_cal_voltage = nvs_get_float("adcCalV", 1.0f);
    g_settings.adc_cal_protection = nvs_get_float("adcCalP", 1.0f);

    size_t name_len = sizeof(g_settings.ble_name);
    if (nvs_get_str(s_nvs_handle, "bleName", g_settings.ble_name, &name_len) != ESP_OK) {
        strncpy(g_settings.ble_name, BLE_DEVICE_NAME, sizeof(g_settings.ble_name) - 1);
    }

    size_t pin_len = sizeof(g_settings.pin);
    if (nvs_get_str(s_nvs_handle, "pin", g_settings.pin, &pin_len) != ESP_OK) {
        strncpy(g_settings.pin, PIN_DEFAULT, PIN_MAX_LEN - 1);
        g_settings.pin[PIN_MAX_LEN - 1] = '\0';
    }

    // Load presets
    for (int i = 0; i < MAX_PRESETS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "preset%d", i);
        size_t blob_size = sizeof(weld_preset_t);
        if (nvs_get_blob(s_nvs_handle, key, &g_settings.presets[i], &blob_size) != ESP_OK) {
            memcpy(&g_settings.presets[i], &factory_presets[i], sizeof(weld_preset_t));
        }
    }

    g_settings.session_welds = 0; // Always reset session counter on boot

    nvs_close(s_nvs_handle);
    s_nvs_handle = 0;

    ESP_LOGI(TAG, "Settings loaded from NVS: P1=%.1f T=%.1f P2=%.1f mode=%s preset=%d total_welds=%lu",
             g_settings.p1, g_settings.t, g_settings.p2,
             g_settings.auto_mode ? "AUTO" : "MAN",
             g_settings.active_preset,
             (unsigned long)g_settings.total_welds);
}

static void settings_write_to_nvs(void)
{
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return;
    }

    nvs_set_float("p1", g_settings.p1);
    nvs_set_float("t", g_settings.t);
    nvs_set_float("p2", g_settings.p2);
    nvs_set_float("sValue", g_settings.s_value);
    nvs_set_u8(s_nvs_handle, "autoMode", g_settings.auto_mode);
    nvs_set_u8(s_nvs_handle, "bright", g_settings.brightness);
    nvs_set_u8(s_nvs_handle, "soundOn", g_settings.sound_on);
    nvs_set_u8(s_nvs_handle, "theme", g_settings.theme);
    nvs_set_u8(s_nvs_handle, "actPreset", g_settings.active_preset);
    nvs_set_u32(s_nvs_handle, "totalWeld", g_settings.total_welds);
    nvs_set_float("adcCalV", g_settings.adc_cal_voltage);
    nvs_set_float("adcCalP", g_settings.adc_cal_protection);
    nvs_set_str(s_nvs_handle, "bleName", g_settings.ble_name);
    nvs_set_str(s_nvs_handle, "pin", g_settings.pin);

    for (int i = 0; i < MAX_PRESETS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "preset%d", i);
        nvs_set_blob(s_nvs_handle, key, &g_settings.presets[i], sizeof(weld_preset_t));
    }

    nvs_commit(s_nvs_handle);
    nvs_close(s_nvs_handle);
    s_nvs_handle = 0;
    s_dirty = false;

    ESP_LOGI(TAG, "Settings saved to NVS");
}

// ============================================================================
// Debounced Save Timer Callback
// ============================================================================

static void save_timer_callback(void *arg)
{
    (void)arg;
    if (s_dirty) {
        settings_write_to_nvs();
    }
}

// ============================================================================
// Public API
// ============================================================================

void settings_init(void)
{
    settings_load_defaults();
    settings_load_from_nvs();

    // Create debounce timer for auto-save
    esp_timer_create_args_t timer_args = {
        .callback = save_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "nvs_save",
    };
    esp_timer_create(&timer_args, &s_save_timer);

    ESP_LOGI(TAG, "Settings initialized");
}

void settings_save(void)
{
    s_dirty = true;
    // Restart debounce timer (2 seconds)
    esp_timer_stop(s_save_timer);
    esp_timer_start_once(s_save_timer, NVS_SAVE_DEBOUNCE_MS * 1000);
}

void settings_save_now(void)
{
    esp_timer_stop(s_save_timer);
    settings_write_to_nvs();
}

void settings_factory_reset(void)
{
    // Erase NVS namespace
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    settings_load_defaults();
    settings_write_to_nvs();
    ESP_LOGW(TAG, "Factory reset complete — all settings restored to defaults");
}

void settings_load_preset(uint8_t index)
{
    if (index >= MAX_PRESETS) return;
    const weld_preset_t *p = &g_settings.presets[index];
    g_settings.p1 = p->p1;
    g_settings.t  = p->t;
    g_settings.p2 = p->p2;
    g_settings.s_value = p->s_value;
    g_settings.auto_mode = p->auto_mode;
    g_settings.active_preset = index;
    settings_save();
    ESP_LOGI(TAG, "Loaded preset %d: %s (P1=%.1f T=%.1f P2=%.1f S=%.1f %s)",
             index, p->name, p->p1, p->t, p->p2, p->s_value,
             p->auto_mode ? "AUTO" : "MAN");
}

void settings_save_preset(uint8_t index, const char *name,
                          float p1, float t, float p2,
                          float s_value, bool auto_mode)
{
    if (index >= MAX_PRESETS) return;
    weld_preset_t *p = &g_settings.presets[index];
    strncpy(p->name, name, PRESET_NAME_LEN - 1);
    p->name[PRESET_NAME_LEN - 1] = '\0';
    p->p1        = p1;
    p->t         = t;
    p->p2        = p2;
    p->s_value   = s_value;
    p->auto_mode = auto_mode;
    settings_save_now();
    ESP_LOGI(TAG, "Saved preset %d: '%s' (P1=%.1f T=%.1f P2=%.1f S=%.1f %s)",
             index, name, p1, t, p2, s_value, auto_mode ? "AUTO" : "MAN");
}

void settings_increment_weld_count(void)
{
    g_settings.session_welds++;
    g_settings.total_welds++;
    // Persist total counter immediately (important data)
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, "totalWeld", g_settings.total_welds);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool settings_verify_pin(const char *pin)
{
    if (pin == NULL) return false;
    return strncmp(g_settings.pin, pin, PIN_MAX_LEN) == 0;
}

void settings_change_pin(const char *new_pin)
{
    if (new_pin == NULL || strlen(new_pin) == 0) return;
    strncpy(g_settings.pin, new_pin, PIN_MAX_LEN - 1);
    g_settings.pin[PIN_MAX_LEN - 1] = '\0';
    settings_save_now();
    ESP_LOGI(TAG, "PIN changed");
}
