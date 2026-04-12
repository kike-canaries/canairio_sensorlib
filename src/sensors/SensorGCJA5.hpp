#ifndef SENSOR_GCJA5_HPP
#define SENSOR_GCJA5_HPP

#include <SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h>

#include "ISensor.hpp"

class SensorGCJA5 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SGCJA5; }
  const char* getName() const override { return "GCJA5"; }

  uint16_t getPM1() const { return _pm1; }
  uint16_t getPM25() const { return _pm25; }
  uint16_t getPM10() const { return _pm10; }

 private:
  SFE_PARTICLE_SENSOR _sensor;
  uint16_t _pm1 = 0;
  uint16_t _pm25 = 0;
  uint16_t _pm10 = 0;
  bool _available = false;
};

#endif  // SENSOR_GCJA5_HPP
