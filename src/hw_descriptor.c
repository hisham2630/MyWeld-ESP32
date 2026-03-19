/**
 * Hardware Compatibility Descriptor — Compile-Time Instance
 *
 * This const struct is placed in .rodata by the linker. It survives
 * in the firmware binary image and can be read by:
 *   - The OTA module (to compare against incoming firmware)
 *   - The BLE VERSION_RESPONSE handler (to expose to the Android app)
 *   - Offline tools (by scanning for the HW_DESC_MAGIC pattern)
 */

#include "hw_descriptor.h"

const hw_descriptor_t HW_DESCRIPTOR = {
    .magic          = HW_DESC_MAGIC,
    .board_variant  = BOARD_VARIANT,
    .display_type   = DISPLAY_TYPE,
    .audio_type     = AUDIO_TYPE,
    .reserved_0     = 0,
    .hw_compat_id   = HW_COMPAT_ID_CURRENT,
    .reserved       = {0},
};
