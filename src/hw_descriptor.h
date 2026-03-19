/**
 * Hardware Compatibility Descriptor
 *
 * Embedded at compile time in every firmware binary. Used by:
 *   - OTA module: reject incoming firmware if hw_compat_id doesn't match
 *   - BLE VERSION_RESPONSE: expose to the Android app for pre-flight checks
 *   - GitHub releases: map variant slug → hw_compat_id in manifest
 *
 * The hw_compat_id is a simple 32-bit value computed from the board variant,
 * display type, and audio type. It's NOT cryptographic — it's a fast mismatch
 * check to prevent cross-flashing between hardware variants.
 */

#ifndef HW_DESCRIPTOR_H
#define HW_DESCRIPTOR_H

#include <stdint.h>
#include "board_config.h"

/**
 * Magic number "MWVD" (MyWeld Variant Descriptor) — used to identify
 * the descriptor in a raw firmware binary for offline validation tools.
 */
#define HW_DESC_MAGIC  0x4D575644  /* ASCII "MWVD" in little-endian */

/**
 * Hardware Compatibility Descriptor — packed struct embedded in .rodata.
 * Total: 20 bytes.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;            /* HW_DESC_MAGIC — identifies this struct */
    uint8_t  board_variant;    /* BOARD_VARIANT (1=JC3248W535, 2=DevKit, 3=GOOUUU CAM) */
    uint8_t  display_type;     /* DISPLAY_TYPE  (1=QSPI, 2=Nextion, 3=LCD2004) */
    uint8_t  audio_type;       /* AUDIO_TYPE    (1=I2S, 2=Buzzer) */
    uint8_t  reserved_0;       /* Padding / future use */
    uint32_t hw_compat_id;     /* Quick comparison hash of variant+display+audio */
    uint8_t  reserved[8];      /* Future expansion (total struct = 20 bytes) */
} hw_descriptor_t;

_Static_assert(sizeof(hw_descriptor_t) == 20, "hw_descriptor_t must be 20 bytes");

/**
 * Global const instance — linker embeds this in .rodata.
 * Defined in hw_descriptor.c.
 */
extern const hw_descriptor_t HW_DESCRIPTOR;

/**
 * Compute a hw_compat_id from variant parameters.
 *
 * This is NOT a hash — it's a simple unique mapping for our small domain
 * (3 boards × 3 displays × 2 audio = 18 possible combinations).
 * The encoding is: [variant:16][display:8][audio:8].
 *
 * Implemented as a macro (not inline function) so the result is a
 * compile-time constant usable in static initializers.
 */
#define HW_COMPUTE_COMPAT_ID(variant, disp, audio) \
    (((uint32_t)(variant) << 16) | ((uint32_t)(disp) << 8) | (uint32_t)(audio))

/**
 * Convenience macro for the current build's compat ID.
 */
#define HW_COMPAT_ID_CURRENT \
    HW_COMPUTE_COMPAT_ID(BOARD_VARIANT, DISPLAY_TYPE, AUDIO_TYPE)

/**
 * Human-readable variant slug for this build.
 * Used in log messages and can be matched to GitHub release asset names.
 */
#if (BOARD_VARIANT == BOARD_JC3248W535)
  #define HW_VARIANT_SLUG  "jc3248w535"
#elif (BOARD_VARIANT == BOARD_GOOUUU_CAM)
  #if (DISPLAY_TYPE == DISPLAY_NEXTION)
    #define HW_VARIANT_SLUG  "goouuu-nextion"
  #elif (DISPLAY_TYPE == DISPLAY_LCD_2004)
    #define HW_VARIANT_SLUG  "goouuu-lcd2004"
  #else
    #define HW_VARIANT_SLUG  "goouuu"
  #endif
#elif (DISPLAY_TYPE == DISPLAY_NEXTION)
  #define HW_VARIANT_SLUG  "nextion"
#elif (DISPLAY_TYPE == DISPLAY_LCD_2004)
  #define HW_VARIANT_SLUG  "lcd2004"
#else
  #define HW_VARIANT_SLUG  "unknown"
#endif

#endif /* HW_DESCRIPTOR_H */
