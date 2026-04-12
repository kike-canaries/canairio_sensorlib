#include "SensorAHT10.hpp"

bool SensorAHT10::init() {
#ifdef M5STICKCPLUS  // issue: https://github.com/enjoyneering/AHTxx/issues/11
  _available = _aht.begin(EXT_I2C_SDA, EXT_I2C_SCL, 100000, 50000);
#else
  _available = _aht.begin();
#endif
  return _available;
}

bool SensorAHT10::read() {
  float temperature = _aht.readTemperature();
  if (temperature == AHTXX_ERROR) return false;
  _temp = temperature;
  float humidity = _aht.readHumidity();
  if (humidity != AHTXX_ERROR) _humi = humidity;
  return true;
}
