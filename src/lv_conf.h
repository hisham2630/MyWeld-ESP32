/**
 * LVGL 9.x Configuration for MyWeld ESP32-S3
 * Board: JC3248W535 (480x320, AXS15231B QSPI)
 * LVGL version: 9.x (9.2+ confirmed via platformio.ini)
 *
 * How lv_conf_internal.h reads this file:
 *   It uses #ifndef guards around every default value. Any macro defined here
 *   takes priority over the defaults in lv_conf_internal.h.
 *
 * Verified against: .pio/libdeps/jc3248w535/lvgl/src/lv_conf_internal.h
 */
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* =========================================================================
 * PLATFORM
 * Disable ARM Helium SIMD — this is an ARM Cortex-M55 acceleration layer.
 * LVGL 9.x ships lv_blend_helium.S which the Xtensa ESP32 assembler cannot
 * parse (it emits "unknown opcode typedef" errors). Explicitly disabling it
 * forces LVGL to use its standard C software renderer instead.
 * ========================================================================= */
#define LV_USE_NATIVE_HELIUM_ASM 0

/* =========================================================================
 * DRAW / ASM OPTIMIZATIONS
 * =========================================================================
 * CRITICAL for ESP32-S3 (Xtensa arch): Explicitly disable ARM Helium (MVE)
 * assembly optimizations. LVGL 9.5 ships lv_blend_helium.S which contains
 * ARM Thumb-2 instructions. When the Xtensa assembler processes this file
 * it hits C-style 'typedef' in included headers and aborts with:
 *   "Error: unknown opcode or format name 'typedef'"
 * Setting LV_DRAW_SW_ASM to LV_DRAW_SW_ASM_NONE disables that code path.
 * ========================================================================= */
#define LV_DRAW_SW_ASM LV_DRAW_SW_ASM_NONE

/* =========================================================================
 * COLOR
 * ========================================================================= */

/** 16-bit RGB565 */
#define LV_COLOR_DEPTH 16

/* =========================================================================
 * STDLIB WRAPPER  (LVGL 9.x uses LV_USE_STDLIB_MALLOC, not LV_MEM_CUSTOM)
 *
 * LV_STDLIB_BUILTIN = 0  → LVGL internal pool allocator (needs LV_MEM_SIZE)
 * LV_STDLIB_CLIB    = 1  → Use OS malloc/free/realloc  ← we want this
 *
 * With SPIRAM_USE_MALLOC enabled in sdkconfig, the standard malloc() will
 * draw from both internal and PSRAM heaps, so LVGL internal buffers can
 * overflow into PSRAM automatically.
 * ========================================================================= */
#define LV_USE_STDLIB_MALLOC    LV_STDLIB_CLIB   /* = 1 */
#define LV_USE_STDLIB_STRING    LV_STDLIB_CLIB   /* = 1 */
#define LV_USE_STDLIB_SPRINTF   LV_STDLIB_CLIB   /* = 1 */

/* =========================================================================
 * HAL / TICK
 * ========================================================================= */

/** Display refresh period [ms] — ~60 FPS target */
#define LV_DEF_REFR_PERIOD 16

/**
 * LV_TICK_CUSTOM = 0: we call lv_tick_inc() manually each ui_task loop.
 * This is the correct approach for bare ESP-IDF without os-level tick hook.
 * NOTE: LV_TICK_CUSTOM does not exist in LVGL 9.x (removed). Keep this
 * comment for clarity but don't define it — it's harmless either way.
 */

/**
 * LV_USE_OS = LV_OS_NONE: we manage concurrency with our own mutex in
 * display.c (display_lock / display_unlock). This avoids LVGL trying to
 * create its own FreeRTOS mutex on top of ours, which caused deadlocks.
 */
#define LV_USE_OS   LV_OS_NONE   /* = 0 */

/* =========================================================================
 * LOGGING
 * ========================================================================= */

#define LV_USE_LOG 1
#if LV_USE_LOG
    /** Log level: TRACE=0, INFO=1, WARN=2, ERROR=3, USER=4, NONE=5 */
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
    /** 1: Print via printf */
    #define LV_LOG_PRINTF 1
#endif

/* =========================================================================
 * FONTS  (must be defined here AND in platformio.ini build_flags)
 * ========================================================================= */

#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_36 1
#define LV_FONT_MONTSERRAT_48 1

#define LV_FONT_DEFAULT &lv_font_montserrat_16

/* =========================================================================
 * WIDGETS
 * ========================================================================= */

#define LV_USE_LABEL     1
#define LV_USE_BTN       1
#define LV_USE_BTNMATRIX 1
#define LV_USE_BAR       1
#define LV_USE_SLIDER    1
#define LV_USE_SWITCH    1
#define LV_USE_CHART     1
#define LV_USE_ARC       1
#define LV_USE_DROPDOWN  1
#define LV_USE_ROLLER    1
#define LV_USE_TEXTAREA  1
#define LV_USE_TABLE     1
#define LV_USE_TABVIEW   1
#define LV_USE_MSGBOX    1
#define LV_USE_KEYBOARD  1
#define LV_USE_SPAN      1
#define LV_USE_IMG       1
#define LV_USE_LINE      1
#define LV_USE_ANIMIMG   1

/* =========================================================================
 * THEMES / LAYOUTS
 * ========================================================================= */

#define LV_USE_THEME_DEFAULT 1
#define LV_USE_FLEX          1
#define LV_USE_GRID          1

/* =========================================================================
 * ASSERTS / DEBUG
 * ========================================================================= */

#define LV_USE_ASSERT_NULL   1
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_OBJ    0

#define LV_USE_REFR_DEBUG    0
#define LV_USE_LAYER_DEBUG   0

#endif /* LV_CONF_H */
