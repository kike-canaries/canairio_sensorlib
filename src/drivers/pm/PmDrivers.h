#ifndef PmDrivers_h
#define PmDrivers_h

#include "../SensorDriver.h"

/**
 * @brief Sensirion SPS30 particulate matter sensor.
 *
 * Supports both I2C and SHDLC/UART transports. The transport is chosen by the
 * UART autodetect path (beginUart) or by Sensors::init (begin -> I2C).
 */
class Sps30Driver : public SensorDriver {
 public:
  explicit Sps30Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "SPS30"; }
  SENSORS type() const override { return SENSORS::SSPS30; }
  bool begin() override;  // I2C transport
  bool beginUart();       // used by the UART autodetect path
  bool read() override;   // returns false when there is no valid frame

 private:
  bool tests();
  void errToMess(char *mess, uint8_t r);
  void errorloop(char *mess, uint8_t r);
  void deviceInfo();
};

/**
 * @brief Panasonic SN-GCJA5 (I2C transport).
 *
 * The UART variant is handled by UartDriver; read() is a no-op when the UART
 * autodetect selected this sensor.
 */
class Gcja5Driver : public SensorDriver {
 public:
  explicit Gcja5Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "GCJA5"; }
  SENSORS type() const override { return SENSORS::SGCJA5; }
  bool begin() override;
  bool read() override;
};

#endif  // PmDrivers_h
