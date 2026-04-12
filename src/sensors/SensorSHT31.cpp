#include "SensorSHT31.hpp"

bool SensorSHT31::init() {
  _sht = Adafruit_SHT31();
#ifndef Wire1
  _available = _sht.begin();
#else
  if (!_sht.begin()) {
    _sht = Adafruit_SHT31(&Wire1);
    _available = _sht.begin();
  } else {
    _available = true;
  }
#endif
  return _available;
}

bool SensorSHT31::read() {
  float humidity = _sht.readHumidity();
  float temperature = _sht.readTemperature();
  if (!isnan(humidity)) _humi = humidity;
  if (isnan(temperature)) return false;
  _temp = temperature;
  return true;
}
