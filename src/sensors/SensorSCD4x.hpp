#ifndef SENSOR_SCD4X_HPP
#define SENSOR_SCD4X_HPP

#include <SensirionI2CScd4x.h>

#include "ISensor.hpp"

class SensorSCD4x : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SSCD4X; }
  const char* getName() const override { return "SCD4x"; }
  void setTemperatureOffset(float offset) override;
  void setAltitude(float altitude) override;

  float getCO2() const { return _co2; }
  float getTemperature() const { return _temperature; }
  float getHumidity() const { return _humidity; }
  float getTemperatureOffset();
  uint16_t getSensorAltitude();
  bool calibrate(int ppmValue = 0) override;
  bool startPeriodicMeasurement();

 private:
  SensirionI2CScd4x _scd4x;
  float _co2 = 0.0f;
  float _temperature = 0.0f;
  float _humidity = 0.0f;
  bool _available = false;
};

#endif  // SENSOR_SCD4X_HPP
