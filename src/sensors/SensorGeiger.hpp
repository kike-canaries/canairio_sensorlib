#ifndef SENSOR_GEIGER_HPP
#define SENSOR_GEIGER_HPP

#include <drivers/geiger.h>

#include "ISensor.hpp"

/**
 * @brief ISensor wrapper for the CAJOE Geiger counter driver.
 *
 * Thin adapter: the real work lives in GEIGER (drivers/geiger.h).
 * The sensor requires an explicit GPIO pin, so init() just validates
 * that the driver was already created (by enableGeigerSensor).
 */
class SensorGeiger : public ISensor {
 public:
  explicit SensorGeiger(int gpio, bool debug = false);
  ~SensorGeiger();

  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _driver != nullptr; }
  SENSORS getSensorType() const override { return SENSORS::SCAJOE; }
  const char* getName() const override { return "Geiger"; }

  uint32_t getCPM() const;
  float getMicroSievertHour() const;
  void clear();

 private:
  GEIGER* _driver = nullptr;
};

#endif  // SENSOR_GEIGER_HPP
