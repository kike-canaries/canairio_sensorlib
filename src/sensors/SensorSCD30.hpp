#ifndef SENSOR_SCD30_HPP
#define SENSOR_SCD30_HPP

#include <Adafruit_SCD30.h>

#include "ISensor.hpp"

class SensorSCD30 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SSCD30; }
  const char* getName() const override { return "SCD30"; }
  void setTemperatureOffset(float offset) override;
  void setAltitude(float altitude) override;

  float getCO2() const { return _co2; }
  float getTemperature() const { return _temperature; }
  float getHumidity() const { return _humidity; }
  float getTemperatureOffset();
  uint16_t getAltitudeOffset();
  void setMeasurementInterval(uint16_t seconds);
  void forceRecalibration(int ppmValue);

 private:
  Adafruit_SCD30 _scd30;
  float _co2 = 0.0f;
  float _temperature = 0.0f;
  float _humidity = 0.0f;
  bool _available = false;
};

#endif  // SENSOR_SCD30_HPP
