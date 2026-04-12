#include "SensorBMP280.hpp"

bool SensorBMP280::init() {
#ifndef Wire1
  _available = _bmp.begin() || _bmp.begin(BMP280_ADDRESS_ALT);
#else
  if (!_bmp.begin() && !_bmp.begin(BMP280_ADDRESS_ALT)) {
    _bmp = Adafruit_BMP280(&Wire1);
    _available = _bmp.begin() || _bmp.begin(BMP280_ADDRESS_ALT);
  } else {
    _available = true;
  }
#endif
  if (_available) {
    _bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);
  }
  return _available;
}

bool SensorBMP280::read() {
  float temperature = _bmp.readTemperature();
  float pressure = _bmp.readPressure();
  float altitude = _bmp.readAltitude(_sealevel);
  if (pressure == 0 || isnan(temperature) || isnan(altitude)) return false;
  _temp = temperature;
  _pres = pressure / 100.0f;
  _alt = altitude;
  return true;
}
