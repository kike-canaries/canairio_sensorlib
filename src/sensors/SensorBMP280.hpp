#ifndef SENSOR_BMP280_HPP
#define SENSOR_BMP280_HPP

#include <Adafruit_BMP280.h>

#include "ISensor.hpp"

class SensorBMP280 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SBMP280; }
  const char* getName() const override { return "BMP280"; }
  void setSeaLevelPressure(float hpa) override { _sealevel = hpa; }

  float getTemperature() const { return _temp; }
  float getPressure() const { return _pres; }
  float getAltitude() const { return _alt; }

 private:
  Adafruit_BMP280 _bmp;
  float _temp = 0.0f;
  float _pres = 0.0f;
  float _alt = 0.0f;
  float _sealevel = 1013.25f;
  bool _available = false;
};

#endif  // SENSOR_BMP280_HPP
