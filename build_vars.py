Import("env")

# Suppress stupid warning for ESP32
env["CFLAGS"] += " -Wno-incompatible-pointer-types"

