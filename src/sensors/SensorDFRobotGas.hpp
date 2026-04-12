#ifndef SENSOR_DFROBOT_GAS_HPP
#define SENSOR_DFROBOT_GAS_HPP

#include <DFRobot_MultiGasSensor.h>

#include "ISensor.hpp"

/**
 * @brief Modular DFRobot Gravity gas sensor driver.
 *
 * Wraps a single DFRobot_GAS_I2C instance for one gas type (CO, NH3, NO2, O3).
 * Reimplements DFRobot's temperature compensation formulas locally so we can
 * feed an external ambient temperature instead of the stale onboard thermistor.
 * Pressure compensation normalises to 1013.25 hPa.
 */
class SensorDFRobotGas : public ISensor {
 public:
  SensorDFRobotGas(uint8_t address, SENSORS type, const char* name, uint8_t gasType,
                   TwoWire* wire = &Wire);

  bool init() override;
  bool read() override;
  bool isAvailable() const override { return _available; }
  SENSORS getSensorType() const override { return _type; }
  const char* getName() const override { return _name; }

  void setAmbientData(float temp, float hum, float pres) override;

  float getConcentration() const { return _concentration; }
  float getInternalTemperature() const { return _internalTemp; }
  bool hasExternalAmbient() const { return _hasAmbient; }

 private:
  DFRobot_GAS_I2C _driver;
  SENSORS _type;
  const char* _name;
  uint8_t _gasType;
  float _concentration = 0.0f;
  float _internalTemp = 0.0f;
  bool _available = false;

  float _ambientTemp = 0.0f;
  float _ambientPres = 0.0f;
  bool _hasAmbient = false;

  float tempCompensation(float rawPpm, float temperature);
  float pressCompensation(float ppm, float pressure);
};

#endif  // SENSOR_DFROBOT_GAS_HPP
