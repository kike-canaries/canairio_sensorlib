#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include <Arduino.h>

#include "SensorTypes.hpp"

/**
 * @brief Abstract interface for all modular sensor drivers.
 *
 * Each concrete sensor implements init/read/isAvailable plus type metadata.
 * Optional hooks (setAltitude, calibrate, setAmbientData) have default no-op
 * implementations so that only sensors that need them override.
 */
class ISensor {
 public:
  virtual ~ISensor() {}

  virtual bool init() = 0;

  virtual void loop() {}

  virtual bool read() = 0;

  virtual bool isAvailable() const = 0;

  virtual SENSORS getSensorType() const = 0;

  virtual const char* getName() const = 0;

  virtual void setSampleTime(int seconds) { (void)seconds; }
  virtual void setAltitude(float altitude) { (void)altitude; }
  virtual void setTemperatureOffset(float offset) { (void)offset; }
  virtual void setSeaLevelPressure(float hpa) { (void)hpa; }
  virtual bool calibrate(int value = 0) {
    (void)value;
    return false;
  }
  virtual void setAmbientData(float temp, float hum, float pres) {
    (void)temp;
    (void)hum;
    (void)pres;
  }
};

#endif  // ISENSOR_HPP
