#ifndef SENSOR_BME680_HPP
#define SENSOR_BME680_HPP

#include <Adafruit_BME680.h>

#include "ISensor.hpp"

class SensorBME680 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SBME680; }
  const char* getName() const override { return "BME680"; }
  void setSeaLevelPressure(float hpa) override { _sealevel = hpa; }

  float getTemperature() const { return _temp; }
  float getHumidity() const { return _humi; }
  float getPressure() const { return _pres; }
  float getGasResistance() const { return _gas; }
  float getAltitude() const { return _alt; }

 private:
  Adafruit_BME680 _bme;
  float _temp = 0.0f;
  float _humi = 0.0f;
  float _pres = 0.0f;
  float _gas = 0.0f;
  float _alt = 0.0f;
  float _sealevel = 1013.25f;
  bool _available = false;
};

#endif  // SENSOR_BME680_HPP
