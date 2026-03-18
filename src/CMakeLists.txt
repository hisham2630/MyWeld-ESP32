# MyWeld ESP32-S3 — Component source files
#
# Conditional compilation based on DISPLAY_TYPE and AUDIO_TYPE
# defines set in platformio.ini build_flags.
#
# DISPLAY_TYPE: 1=QSPI_TFT, 2=NEXTION, 3=LCD_2004
# AUDIO_TYPE:   1=I2S, 2=BUZZER

# --- Core sources (always compiled) ---
set(CORE_SRCS
    "main.c"
    "settings.c"
    "welding.c"
    "ble_serial.c"
    "ota.c"
    "encoder.c"
)

# --- Display sources (variant-specific) ---
set(DISPLAY_SRCS "")

if(DEFINED ENV{DISPLAY_TYPE})
    set(DISPLAY_TYPE_VAL $ENV{DISPLAY_TYPE})
else()
    # Default: check compile definitions from PlatformIO
    # PlatformIO passes -DDISPLAY_TYPE=N which CMake sees in compile flags
    # We use a fallback approach: include all and let the preprocessor guard
    set(DISPLAY_TYPE_VAL 0)
endif()

# For PlatformIO + ESP-IDF, preprocessor guards (#if HAS_LVGL / HAS_NEXTION / HAS_CHAR_LCD)
# handle the actual conditional compilation. We include all source files but
# the preprocessor will compile only the active variant's code.
# This avoids complex CMake logic for extracting -D flags from build_flags.
set(DISPLAY_SRCS
    "display.c"
    "ui.c"
    "ui_stubs.c"
    "esp_lcd_axs15231b.c"
    "esp_lcd_touch.c"
    "nextion.c"
    "lcd2004.c"
)

# --- Audio sources (both included, preprocessor guards select) ---
set(AUDIO_SRCS
    "audio.c"
    "buzzer.c"
)

# --- Component dependencies ---
# Base dependencies (always needed)
set(COMP_REQUIRES
    driver
    esp_adc
    nvs_flash
    esp_timer
    freertos
    esp_event
    bt
    app_update
    mbedtls
    esp_partition
)

# esp_lcd is needed for the QSPI TFT variant (display.c uses it)
# For other variants it won't fail — the preprocessor guards prevent
# any esp_lcd API calls from being compiled.
list(APPEND COMP_REQUIRES esp_lcd)

# --- Register component ---
idf_component_register(
    SRCS
        ${CORE_SRCS}
        ${DISPLAY_SRCS}
        ${AUDIO_SRCS}
    INCLUDE_DIRS
        "."
    REQUIRES
        ${COMP_REQUIRES}
)

