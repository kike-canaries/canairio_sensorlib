#ifndef SensorDriver_h
#define SensorDriver_h

#include <Arduino.h>

#include "../SensorsTypes.hpp"

class Sensors;

/**
 * @brief Common lifecycle contract for every sensor driver.
 *
 * A driver is a stateless (or sensor-local state) object bound to the Sensors
 * facade. The facade owns the drivers and only talks to them through begin()
 * and read(), so a sensor can be added, disabled or replaced without touching
 * the main read loop.
 *
 * begin() must be idempotent: it probes/initialises the hardware, registers
 * the sensor on success and returns whether the sensor is available.
 * read() must be cheap and non-blocking whenever the underlying protocol
 * allows it, and must leave the last valid values untouched on failure.
 */
class SensorDriver {
 public:
  explicit SensorDriver(Sensors &sensors) : _sensors(sensors) {}
  virtual ~SensorDriver() = default;

  /// Human readable name, matching the SENSORS registry string.
  virtual const char *name() const = 0;

  /// Sensor type used for registration and unit ownership.
  virtual SENSORS type() const = 0;

  /// Probe and initialise the hardware. Safe to call more than once.
  virtual bool begin() = 0;

  /// Read one sample and publish it through the facade.
  virtual bool read() = 0;

  /// True when the sensor was detected and registered.
  virtual bool enabled() const;

 protected:
  Sensors &_sensors;
};

#endif  // SensorDriver_h
