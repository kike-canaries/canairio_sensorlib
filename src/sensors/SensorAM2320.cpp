#include "SensorAM2320.hpp"

bool SensorAM2320::init() {
#ifndef Wire1
  _available = _am.begin();
#else
  _am = AM232X(&Wire);
  if (!_am.begin()) {
    _am = AM232X(&Wire1);
    _available = _am.begin();
  } else {
    _available = true;
  }
#endif
  if (_available) _am.wakeUp();
  return _available;
}

bool SensorAM2320::read() {
  if (!_am.isConnected()) return false;
  int status = _am.read();
  if (status != AM232X_OK) return false;
  float humidity = _am.getHumidity();
  float temperature = _am.getTemperature();
  if (!isnan(humidity)) _humi = humidity;
  if (isnan(temperature)) return false;
  _temp = temperature;
  return true;
}
