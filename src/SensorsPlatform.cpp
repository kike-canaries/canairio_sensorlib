// CanAirIO sensorlib — SensorsPlatform.cpp
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include "Sensors.hpp"

#include <math.h>
#include <string.h>

#ifndef SLIB_I2C_CLOCK_HZ
#define SLIB_I2C_CLOCK_HZ 100000
#endif

void Sensors::onSensorError(const char *msg) {
  DEBUG(msg);
  if (_onErrorCb != nullptr) _onErrorCb(msg);
}

void Sensors::DEBUG(const char *text, const char *textb) const {
  if (devmode) {
    _debugPort.print(text);
    if (textb) {
      _debugPort.print(" ");
      _debugPort.print(textb);
    }
    _debugPort.println();
  }
}

void Sensors::printI2C(const char *name, const char *i2cname, int pin_sda, int pin_scl) {
  if (devmode) {
    Serial.printf("-->[SLIB] (%s) %s pins \t: SDA:%i SCL:%i\r\n", name, i2cname, pin_sda, pin_scl);
  }
}

void Sensors::startI2C() {
#if defined(SLIB_I2C_SDA) && defined(SLIB_I2C_SCL)
  Wire.begin(SLIB_I2C_SDA, SLIB_I2C_SCL);
  printI2C("CUSTOM", "I2C", SLIB_I2C_SDA, SLIB_I2C_SCL);
#elif defined(ESP32S3)
  Wire.begin();
  printI2C("ESP32S3", "I2C", SDA, SCL);
#elif defined(ARDUINO_ARCH_ESP32)
  Wire.begin();
  printI2C("ESP32", "I2C", SDA, SCL);
#elif defined(ARDUINO_ARCH_ESP8266)
  Wire.begin(SDA, SCL);
  printI2C("ESP8266", "I2C", SDA, SCL);
#else
  Wire.begin(SDA, SCL);
  printI2C("UNDEFINED", "I2C", SDA, SCL);
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Wire.setClock(SLIB_I2C_CLOCK_HZ);
  if (devmode) Serial.printf("-->[SLIB] I2C clock set to \t: %iHz\r\n", SLIB_I2C_CLOCK_HZ);
#endif

  enableWire1();
}

void Sensors::enableWire1() {
#if defined(SLIB_I2C2_SDA) && defined(SLIB_I2C2_SCL)
  Wire1.flush();
  Wire1.begin(SLIB_I2C2_SDA, SLIB_I2C2_SCL);
  printI2C("CUSTOM", "I2C2", SLIB_I2C2_SDA, SLIB_I2C2_SCL);
#endif
#ifdef M5COREINK
  Wire1.flush();
  Wire1.begin(25, 26);  // M5CoreInk hat pines (header on top)
#endif
#ifdef M5AIRQ
  Wire1.flush();
  Wire1.begin(GROVE_SDA, GROVE_SCL);
#endif
}

void Sensors::disableWire1() {
#ifdef M5STICKCPLUS
  Wire1.flush();
  Wire1.begin(21, 22);  // Restore AXP192 I2C pins (failed after some time)
#endif
#ifdef M5COREINK
  Wire1.flush();
  Wire1.begin(21, 22);  // M5CoreInk hat pines (header on top)
#endif
}
