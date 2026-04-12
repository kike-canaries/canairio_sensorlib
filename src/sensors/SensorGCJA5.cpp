#include "SensorGCJA5.hpp"

bool SensorGCJA5::init() {
  _available = _sensor.begin();
  return _available;
}

bool SensorGCJA5::read() {
  if (!_sensor.isConnected()) return false;
  uint16_t pm1 = _sensor.getPM1_0();
  uint16_t pm25 = _sensor.getPM2_5();
  uint16_t pm10 = _sensor.getPM10();
  if (pm1 > 1000 || pm25 > 1000 || pm10 > 1000) return false;
  _pm1 = pm1;
  _pm25 = pm25;
  _pm10 = pm10;
  return true;
}
