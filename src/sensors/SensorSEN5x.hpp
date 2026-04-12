#ifndef SENSOR_SEN5X_HPP
#define SENSOR_SEN5X_HPP

#include <SensirionI2CSen5x.h>

#include "ISensor.hpp"

class SensorSEN5x : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SSEN5X; }
  const char* getName() const override { return "SEN5x"; }
  void setTemperatureOffset(float offset) override;

  uint16_t getPM1() const { return _pm1; }
  uint16_t getPM25() const { return _pm25; }
  uint16_t getPM4() const { return _pm4; }
  uint16_t getPM10() const { return _pm10; }
  float getTemperature() const { return _temp; }
  float getHumidity() const { return _humi; }
  float getVocIndex() const { return _vocIndex; }
  float getNoxIndex() const { return _noxIndex; }

 private:
  SensirionI2CSen5x _sen5x;
  uint16_t _pm1 = 0;
  uint16_t _pm25 = 0;
  uint16_t _pm4 = 0;
  uint16_t _pm10 = 0;
  float _temp = 0.0f;
  float _humi = 0.0f;
  float _vocIndex = 0.0f;
  float _noxIndex = 0.0f;
  bool _available = false;
};

#endif  // SENSOR_SEN5X_HPP
