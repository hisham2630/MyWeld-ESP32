# MyWeld ESP32-S3 — Component source files
#
# Conditional compilation based on DISPLAY_TYPE and AUDIO_TYPE
# defines set in platformio.ini build_flags.
#
# DISPLAY_TYPE: 1=QSPI_TFT, 2=NEXTION, 3=LCD_2004, 4=COG_12864
# AUDIO_TYPE:   1=I2S, 2=BUZZER

# --- Core sources (always compiled) ---
set(CORE_SRCS
    "main.c"
    "settings.c"
    "welding.c"
    "ble_serial.c"
    "ota.c"
    "encoder.c"
    "hw_descriptor.c"
    "status_led.c"
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
    "esp_lcd_nv3041a.c"
    "esp_lcd_touch.c"
    "nextion.c"
    "lcd2004.c"
    "st7567s.c"
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

# esp_lcd is needed ONLY for the QSPI TFT variant (DISPLAY_TYPE=1).
# For other variants, including it causes an ICE (internal compiler error)
# in the GCC toolchain when compiling esp_lcd_panel_rgb.c.
# We check the compile definitions passed via PlatformIO's build_flags.
set(_display_type_val 0)
get_directory_property(_compile_defs COMPILE_DEFINITIONS)
foreach(_def ${_compile_defs})
    if(_def MATCHES "^DISPLAY_TYPE=(.+)$")
        set(_display_type_val ${CMAKE_MATCH_1})
    endif()
endforeach()

if(_display_type_val STREQUAL "1")
    list(APPEND COMP_REQUIRES esp_lcd)
endif()

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

