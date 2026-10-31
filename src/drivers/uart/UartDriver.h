#ifndef UartDriver_h
#define UartDriver_h

#include "../SensorDriver.h"

/**
 * @brief Transport driver for the autodetected UART sensor.
 *
 * CanAirIO allows only one UART air sensor at a time (Plantower/Honeywell PM,
 * Panasonic GCJA5, Nova SDS011, IKEA Vindriktning, PMS5003T, MH-Z19, CM1106 or
 * SenseAir S8). The concrete protocol is selected during begin() and kept in
 * the Sensors facade as the current UART device type.
 *
 * The protocol helpers are private: the rest of the library only needs
 * begin(), read() and the AG_OPENAIR second-UART hooks.
 */
class UartDriver : public SensorDriver {
 public:
  explicit UartDriver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "UART"; }
  SENSORS type() const override;

  /// Configure the serial port and autodetect the attached device.
  bool begin(u_int pms_type, int pms_rx, int pms_tx);

  bool begin() override;
  bool read() override;
  bool enabled() const override;

  /// AG_OPENAIR: SenseAir S8 on a dedicated second UART.
  bool beginS8();
  bool readS8();

  /// Re-init support: drop the serial port state and owned sensor objects.
  void flush();
  void reset();

 private:
  bool sensorSerialInit(u_int pms_type, int pms_rx, int pms_tx);
  bool pmSensorAutoDetect(u_int pms_type);
  bool pmSensorRead();
  bool pmGenericRead();
  bool pmGCJA5Read();
  bool pmSDS011Read();
  bool pm1006Read();
  bool pm5003TRead();
  bool CO2Mhz19Read();
  bool CO2CM1106Read();
  bool senseAirS8Read();
  bool CO2Mhz19Init();
  bool CO2CM1106Init();
  bool senseAirS8Init();
  bool sensorSerial2Init(int rx, int tx);
  bool PM1006Init();
  bool PM5003TInit();
  bool serialInit(u_int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx);
  String hwSerialRead(unsigned int length_buffer);
  void printUART(const char *name, unsigned long speed, int pin_rx, int pin_tx);
};

#endif  // UartDriver_h
