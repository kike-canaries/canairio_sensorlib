#include "SensorBME280.hpp"

bool SensorBME280::init() {
#ifndef Wire1
  _available = _bme.begin() || _bme.begin(BME280_ADDRESS_ALTERNATE);
#else
  _available = _bme.begin() || _bme.begin(BME280_ADDRESS_ALTERNATE) ||
               _bme.begin(BME280_ADDRESS, &Wire1) || _bme.begin(BME280_ADDRESS_ALTERNATE, &Wire1);
#endif
  return _available;
}

bool SensorBME280::read() {
  float humidity = _bme.readHumidity();
  float temperature = _bme.readTemperature();
  if (isnan(humidity) || humidity == 0 || isnan(temperature)) return false;
  _humi = humidity;
  _temp = temperature;
  _pres = _bme.readPressure();
  _alt = _bme.readAltitude(_sealevel);
  return true;
}
