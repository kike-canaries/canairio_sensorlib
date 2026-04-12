#ifndef SENSOR_SHT31_HPP
#define SENSOR_SHT31_HPP

#include <Adafruit_SHT31.h>

#include "ISensor.hpp"

class SensorSHT31 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SSHT31; }
  const char* getName() const override { return "SHT31"; }

  float getTemperature() const { return _temp; }
  float getHumidity() const { return _humi; }

 private:
  Adafruit_SHT31 _sht;
  float _temp = 0.0f;
  float _humi = 0.0f;
  bool _available = false;
};

#endif  // SENSOR_SHT31_HPP
