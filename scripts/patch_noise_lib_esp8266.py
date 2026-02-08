"""
Patch roberbike/noise-monitor-i2c-slave for platforms that don't support
Wire.begin(addr, sda, scl, freq):
- ESP8266: Wire.begin(sda, scl, address)
- SAMD (atmelsam): Wire.begin(address)
- ESP32: Wire.begin(addr, sda, scl, freq)
"""
import os
import re

Import("env")

# Replacement: single line -> multi-platform #if
OLD_LINE = re.compile(
    r"(\s*)Wire\.begin\s*\(\s*I2C_SLAVE_ADDR\s*,\s*PIN_SDA\s*,\s*PIN_SCL\s*,\s*100000\s*\)\s*;",
    re.MULTILINE,
)
REPLACEMENT = r"""\1#if defined(ARDUINO_ARCH_ESP8266)
\1    Wire.begin(PIN_SDA, PIN_SCL, I2C_SLAVE_ADDR);
\1#elif defined(ARDUINO_ARCH_SAMD)
\1    Wire.begin(I2C_SLAVE_ADDR);
\1#else
\1    Wire.begin(I2C_SLAVE_ADDR, PIN_SDA, PIN_SCL, 100000);
\1#endif"""


def _apply_patch(lib_main):
    if not os.path.isfile(lib_main):
        return False
    with open(lib_main, "r", encoding="utf-8", errors="replace") as f:
        content = f.read()
    if "#if defined(ARDUINO_ARCH_ESP8266)" in content and "Wire.begin(PIN_SDA, PIN_SCL, I2C_SLAVE_ADDR)" in content:
        return False  # already patched
    new_content, count = OLD_LINE.subn(REPLACEMENT, content, count=1)
    if not count:
        return False
    with open(lib_main, "w", encoding="utf-8", newline="\n") as f:
        f.write(new_content)
    return True


def _run_patch(source, target, env):
    pioenv = env.get("PIOENV", "")
    if pioenv not in ("esp8266", "atmelsam"):
        return
    project_dir = env["PROJECT_DIR"]
    lib_main = os.path.join(
        project_dir, ".pio", "libdeps", pioenv,
        "noise-monitor-i2c-slave", "src", "main.cpp",
    )
    if _apply_patch(lib_main):
        print(" [patch] noise-monitor-i2c-slave: Wire.begin() patched for %s" % pioenv)


# Run at load for current env (esp8266/atmelsam)
pioenv = env.get("PIOENV", "")
if pioenv in ("esp8266", "atmelsam"):
    project_dir = env["PROJECT_DIR"]
    lib_main = os.path.join(
        project_dir, ".pio", "libdeps", pioenv,
        "noise-monitor-i2c-slave", "src", "main.cpp",
    )
    if _apply_patch(lib_main):
        print(" [patch] noise-monitor-i2c-slave: Wire.begin() patched for %s" % pioenv)
    elif not os.path.isfile(lib_main):
        print(" [patch] noise-monitor-i2c-slave: main.cpp not found (lib not yet installed?)")

env.AddPreAction("buildlib", _run_patch)
