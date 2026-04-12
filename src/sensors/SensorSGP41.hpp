#ifndef SENSOR_SGP41_HPP
#define SENSOR_SGP41_HPP

#include <SensirionI2CSgp41.h>

#include "ISensor.hpp"

class SensorSGP41 : public ISensor {
 public:
  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return SENSORS::SSGP41; }
  const char* getName() const override { return "SGP41"; }

  uint16_t getVocRaw() const { return _vocRaw; }
  uint16_t getNoxRaw() const { return _noxRaw; }

 private:
  SensirionI2CSgp41 _sgp41;
  uint16_t _vocRaw = 0;
  uint16_t _noxRaw = 0;
  uint8_t _conditioningSeconds = 10;
  bool _available = false;
};

#endif  // SENSOR_SGP41_HPP
