TARGET=ESP32
PORT=/dev/ttyUSB0
ESP32_PSRAM=disabled
ESP32_DEBUGLEVEL=verbose
GITHUB_REPOS=reeltwo/Reeltwo \
adafruit/Adafruit_NeoPixel \
reeltwo/PCF8574

ifeq ("$(USE_SDCARD)", "1")
ARDUINO_OPTS+='-prefs="compiler.cpp.extra_flags=-DUSE_SDCARD=1"'
endif

include ../Arduino.mk
