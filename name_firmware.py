"""
PlatformIO pre-build script:
  1. Auto-increments FW_VERSION_PATCH in src/config.h on every build
  2. Updates FW_VERSION_STRING to match
  3. Sets firmware output filename to MyWeld_<env>_v<MAJOR>.<MINOR>.<PATCH>
"""

import re
Import("env")

CONFIG_PATH = "src/config.h"

def read_config():
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return f.read()

def write_config(content):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        f.write(content)

def get_version(content):
    major = re.search(r"#define\s+FW_VERSION_MAJOR\s+(\d+)", content)
    minor = re.search(r"#define\s+FW_VERSION_MINOR\s+(\d+)", content)
    patch = re.search(r"#define\s+FW_VERSION_PATCH\s+(\d+)", content)
    return (
        int(major.group(1)) if major else 0,
        int(minor.group(1)) if minor else 0,
        int(patch.group(1)) if patch else 0,
    )

def bump_patch(content, major, minor, patch):
    new_patch = patch + 1
    ver_str = f"{major}.{minor}.{new_patch}"

    content = re.sub(
        r"(#define\s+FW_VERSION_PATCH\s+)\d+",
        rf"\g<1>{new_patch}",
        content,
    )
    content = re.sub(
        r'(#define\s+FW_VERSION_STRING\s+)"[^"]*"',
        rf'\g<1>"{ver_str}"',
        content,
    )
    return content, major, minor, new_patch

try:
    content = read_config()
    major, minor, patch = get_version(content)

    # Bump patch version
    content, major, minor, patch = bump_patch(content, major, minor, patch)
    write_config(content)

    ver_str = f"{major}.{minor}.{patch}"
    env_name = env["PIOENV"]
    prog_name = f"MyWeld_{env_name}_v{ver_str}"

    env.Replace(PROGNAME=prog_name)
    print(f"  Version bumped → v{ver_str}")
    print(f"  Firmware output: {prog_name}.bin")

except Exception as e:
    print(f"WARNING: Version bump failed ({e}), using default name")
