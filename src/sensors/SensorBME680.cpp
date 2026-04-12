#include "SensorBME680.hpp"

bool SensorBME680::init() {
#ifndef Wire1
  _available = _bme.begin();
#else
  if (!_bme.begin()) {
    _bme = Adafruit_BME680(&Wire1);
    _available = _bme.begin();
  } else {
    _available = true;
  }
#endif
  if (_available) {
    _bme.setTemperatureOversampling(BME680_OS_8X);
    _bme.setHumidityOversampling(BME680_OS_2X);
    _bme.setPressureOversampling(BME680_OS_4X);
    _bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    _bme.setGasHeater(320, 150);
  }
  return _available;
}

bool SensorBME680::read() {
  if (!_bme.performReading()) return false;
  _temp = _bme.temperature;
  _humi = _bme.humidity;
  _pres = _bme.pressure / 100.0f;
  _gas = _bme.gas_resistance / 1000.0f;
  _alt = _bme.readAltitude(_sealevel);
  return true;
}
