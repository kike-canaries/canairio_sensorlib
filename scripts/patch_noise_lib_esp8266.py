"""
Patch 'Noise Monitor UNE-EN ISO 1996-2' for non-ESP32 platforms (esp8266, atmelsam).

The library main.cpp includes ESP-IDF headers (driver/adc.h, esp_adc_cal.h, sys/time.h)
that do not exist on ESP8266 or Atmel SAM, causing a fatal compile error.

For those platforms the noise DSP task is never started (no hardware); the I2C slave
stubs are irrelevant for the sensor library consumer. We wrap the whole file in an
#if defined(ARDUINO_ARCH_ESP32) guard so the compiler sees an empty translation unit.
"""
import os

Import("env")  # noqa: F821 - SCons injects this

GUARD_OPEN = "#if defined(ARDUINO_ARCH_ESP32)\n"
GUARD_CLOSE = "\n#endif  // ARDUINO_ARCH_ESP32\n"
SENTINEL = "ARDUINO_ARCH_ESP32"  # present when already patched


def _lib_main_path(project_dir, pioenv):
    """Return the expected path to Noise Monitor main.cpp for the given env."""
    # PlatformIO may store the lib under the exact name or a sanitised variant
    for lib_name in (
        "Noise Monitor UNE-EN ISO 1996-2",
        "Noise_Monitor_UNE-EN_ISO_1996-2",
    ):
        path = os.path.join(
            project_dir, ".pio", "libdeps", pioenv, lib_name, "src", "main.cpp"
        )
        if os.path.isfile(path):
            return path
    return None


def _apply_patch(lib_main):
    if not os.path.isfile(lib_main):
        return False
    with open(lib_main, "r", encoding="utf-8", errors="replace") as f:
        content = f.read()
    if SENTINEL in content:
        return False  # already patched or already guarded
    new_content = GUARD_OPEN + content + GUARD_CLOSE
    with open(lib_main, "w", encoding="utf-8", newline="\n") as f:
        f.write(new_content)
    return True


def _run_patch(source, target, env):
    pioenv = env.get("PIOENV", "")
    if pioenv not in ("esp8266", "atmelsam"):
        return
    project_dir = env["PROJECT_DIR"]
    lib_main = _lib_main_path(project_dir, pioenv)
    if lib_main and _apply_patch(lib_main):
        print(" [patch] Noise Monitor: ESP32-only code guarded for %s" % pioenv)
    elif lib_main is None:
        print(" [patch] Noise Monitor: main.cpp not found (lib not yet installed?)")


# Run immediately at script load (covers the first build pass)
pioenv = env.get("PIOENV", "")  # noqa: F821
if pioenv in ("esp8266", "atmelsam"):
    project_dir = env["PROJECT_DIR"]
    lib_main = _lib_main_path(project_dir, pioenv)
    if lib_main and _apply_patch(lib_main):
        print(" [patch] Noise Monitor: ESP32-only code guarded for %s" % pioenv)
    elif lib_main is None:
        print(" [patch] Noise Monitor: main.cpp not found (lib not yet installed?)")

env.AddPreAction("buildlib", _run_patch)
