#!/bin/bash
# arduino-cli installation on ~/bin:
# curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
# test repo:
# https://github.com/kike-canaries/canairio_sensorlib/

arduino-cli config init

# github libraries support
arduino-cli config set library.enable_unsafe_install true
# dependencies
arduino-cli lib install "Adafruit Unified Sensor@1.1.7"
arduino-cli lib install "Adafruit BME280 Library@2.2.2"
arduino-cli lib install "Adafruit BMP280 Library@2.6.6"
arduino-cli lib install "Adafruit BME680 Library@2.0.2"
arduino-cli lib install "Adafruit SHT31 Library@2.2.0"
arduino-cli lib install "Adafruit SCD30@1.0.9"
arduino-cli lib install "Adafruit BusIO@1.14.1"
arduino-cli lib install "Sensirion Core@0.6.0"
arduino-cli lib install "AM232X@0.4.5"
arduino-cli lib install "MH-Z19@1.5.4"
arduino-cli lib install "S8_UART@1.0.1"
arduino-cli lib install "Sensirion I2C SCD4x@0.3.1"
arduino-cli lib install --git-url https://github.com/paulvha/sps30.git
arduino-cli lib install --git-url https://github.com/enjoyneering/AHTxx.git
arduino-cli lib install --git-url https://github.com/hpsaturn/DHT_nonblocking.git
arduino-cli lib install --git-url https://github.com/jcomas/CM1106_UART.git
arduino-cli lib install --git-url https://github.com/paulvha/SN-GCJA5.git
# target tag:
arduino-cli lib install --git-url https://github.com/kike-canaries/canairio_sensorlib.git#v0.6.4

arduino-cli config set board_manager.additional_urls\
  https://arduino.esp8266.com/stable/package_esp8266com_index.json\
  https://dl.espressif.com/dl/package_esp32_index.json

arduino-cli core update-index

# esp8266 test
arduino-cli core install esp8266:esp8266
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 --build-property "build.extra_flags=-DCORE_DEBUG_LEVEL=0" examples/basic

# esp32 test
#arduino-cli core install esp32:esp32
#arduino-cli compile --fqbn esp32:esp32:ttgo-t7-v14-mini32 --build-property "build.extra_flags=-DCORE_DEBUG_LEVEL=0" examples/basic

