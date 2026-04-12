#ifndef SENSOR_AM2320_HPP
#define SENSOR_AM2320_HPP

#include <AM232X.h>

#include "ISensor.hpp"

class SensorAM2320 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SAM232X; }
  const char* getName() const override { return "AM2320"; }

  float getTemperature() const { return _temp; }
  float getHumidity() const { return _humi; }

 private:
  AM232X _am;
  bool _available = false;
  float _temp = 0.0f;
  float _humi = 0.0f;
};

#endif  // SENSOR_AM2320_HPP
