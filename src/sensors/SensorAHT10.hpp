#ifndef SENSOR_AHT10_HPP
#define SENSOR_AHT10_HPP

#include <AHTxx.h>

#include "ISensor.hpp"

class SensorAHT10 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SAHTXX; }
  const char* getName() const override { return "AHTXX"; }

  float getTemperature() const { return _temp; }
  float getHumidity() const { return _humi; }

 private:
  AHTxx _aht{AHTXX_ADDRESS_X38, AHT1x_SENSOR};
  float _temp = 0.0f;
  float _humi = 0.0f;
  bool _available = false;
};

#endif  // SENSOR_AHT10_HPP
