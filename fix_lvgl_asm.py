"""
PlatformIO pre-build script: disable ARM Helium/NEON assembly in LVGL.

LVGL 9.5+ ships lv_blend_helium.S (ARM Cortex-M55 Helium assembly).
PlatformIO's library builder compiles ALL .S files regardless of C
preprocessor guards — the Xtensa assembler chokes on ARM instructions.

The AddBuildMiddleware approach does NOT work for ESP-IDF library sources
because they are compiled by ESP-IDF's own build system, not PlatformIO's.

Fix: rename .S files to .S.bak so they are invisible to the build system.
This runs as a pre-build step and is idempotent (safe to run repeatedly).
"""

import os
import glob

Import("env")

# Resolve the LVGL library path for the active environment
lvgl_blend_dir = os.path.join(
    env.subst("$PROJECT_LIBDEPS_DIR"),
    env.subst("$PIOENV"),
    "lvgl", "src", "draw", "sw", "blend"
)

# Patterns to match ARM-specific assembly in LVGL
arm_asm_globs = [
    os.path.join(lvgl_blend_dir, "helium", "*.S"),
    os.path.join(lvgl_blend_dir, "neon", "*.S"),
]

for pattern in arm_asm_globs:
    for asm_file in glob.glob(pattern):
        disabled = asm_file + ".bak"
        if os.path.exists(asm_file):
            os.rename(asm_file, disabled)
            print(f"  [fix_lvgl_asm] Disabled ARM assembly: {os.path.basename(asm_file)}")
