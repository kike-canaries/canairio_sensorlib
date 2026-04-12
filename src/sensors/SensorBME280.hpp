#ifndef SENSOR_BME280_HPP
#define SENSOR_BME280_HPP

#include <Adafruit_BME280.h>

#include "ISensor.hpp"

class SensorBME280 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SBME280; }
  const char* getName() const override { return "BME280"; }
  void setSeaLevelPressure(float hpa) override { _sealevel = hpa; }

  float getTemperature() const { return _temp; }
  float getHumidity() const { return _humi; }
  float getPressure() const { return _pres; }
  float getAltitude() const { return _alt; }

 private:
  Adafruit_BME280 _bme;
  float _temp = 0.0f;
  float _humi = 0.0f;
  float _pres = 0.0f;
  float _alt = 0.0f;
  float _sealevel = 1013.25f;
  bool _available = false;
};

#endif  // SENSOR_BME280_HPP
