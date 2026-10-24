#ifndef GasDrivers_h
#define GasDrivers_h

#include <DFRobot_MultiGasSensor.h>

#include "../SensorDriver.h"

/**
 * @brief Common implementation for the DFRobot Gravity electrochemical family.
 *
 * The four gases share the I2C protocol, the passive acquisition mode and the
 * temperature/pressure/humidity compensation, so only the address, the target
 * value and the gas type differ between instances.
 */
class DfrobotGasDriver : public SensorDriver {
 public:
  explicit DfrobotGasDriver(Sensors &s) : SensorDriver(s) {}

 protected:
  bool beginGas(DFRobot_GAS_I2C &gas, uint8_t address, SENSORS sensor);
  bool readGas(DFRobot_GAS_I2C &gas, float &value, uint8_t gasType, UNIT unit);
  bool hasExternalTempSensor() const;

 private:
  static float tempCompensation(float rawPpm, float temperature, uint8_t gasType);
  static float pressCompensation(float ppm, float pressure);
  static float humidityCompensation(float ppm, float humidity, uint8_t gasType);
};

class DfrobotCoDriver : public DfrobotGasDriver {
 public:
  explicit DfrobotCoDriver(Sensors &s) : DfrobotGasDriver(s) {}
  const char *name() const override { return "DFRCO"; }
  SENSORS type() const override { return SENSORS::SDFRCO; }
  bool begin() override;
  bool read() override;
};

class DfrobotNh3Driver : public DfrobotGasDriver {
 public:
  explicit DfrobotNh3Driver(Sensors &s) : DfrobotGasDriver(s) {}
  const char *name() const override { return "DFRNH3"; }
  SENSORS type() const override { return SENSORS::SDFRNH3; }
  bool begin() override;
  bool read() override;
};

class DfrobotNo2Driver : public DfrobotGasDriver {
 public:
  explicit DfrobotNo2Driver(Sensors &s) : DfrobotGasDriver(s) {}
  const char *name() const override { return "DFRNO2"; }
  SENSORS type() const override { return SENSORS::SDFRNO2; }
  bool begin() override;
  bool read() override;
};

class DfrobotO3Driver : public DfrobotGasDriver {
 public:
  explicit DfrobotO3Driver(Sensors &s) : DfrobotGasDriver(s) {}
  const char *name() const override { return "DFRO3"; }
  SENSORS type() const override { return SENSORS::SDFRO3; }
  bool begin() override;
  bool read() override;
};

#endif  // GasDrivers_h
