#ifndef NoiseSensorDriver_h
#define NoiseSensorDriver_h

#include <Wire.h>
#include <time.h>

#include "NoiseSlave.h"
#include "SensorDriver.h"

/**
 * @brief I2C noise node (LAeq, L10, L90, Ld/Le/Ln/Lden).
 *
 * Detection is cheap and retried from loop() until the node answers, so the
 * driver exposes service() in addition to the regular begin()/read() pair.
 * The node has no RTC: the driver also owns the clock synchronization policy.
 */
class NoiseSensorDriver : public SensorDriver {
 public:
  explicit NoiseSensorDriver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "NOISE"; }
  SENSORS type() const override { return SENSORS::SNOISE; }
  bool begin() override;
  bool read() override;
  bool enabled() const override;

  /// Background detection/clock sync, called from Sensors::loop().
  void service();

  /// Public time-sync API (forwarded by Sensors for backwards compatibility).
  bool sendTime(uint32_t unixTime);
  bool syncTime();
  void setTimeSyncInterval(uint32_t intervalMs);

 private:
  bool sendTimeImpl(uint32_t unixTime);
  bool syncTimeImpl();
  uint32_t localEpoch(time_t utcNow) const;

  bool autoDetect();
  void collect();
  bool readIdentity(TwoWire &wire, uint8_t address, SensorIdentity &identity);
  bool readStatus(TwoWire &wire, uint8_t address, uint8_t &status);
  bool readMetadata(TwoWire &wire, uint8_t address, NodeMetadata &meta);
  bool readData(TwoWire &wire, uint8_t address, SensorData &out);
  bool devicePresent(TwoWire &wire, uint8_t address);
  void initWire();
};

#endif  // NoiseSensorDriver_h
