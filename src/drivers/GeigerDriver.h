#ifndef GeigerDriver_h
#define GeigerDriver_h

#include "SensorDriver.h"
#include "geiger.h"

/**
 * @brief CAJOE Geiger counter driver.
 *
 * Owns the pulse counter and exposes CPM and uSv/h. The public Sensors facade
 * keeps getGeigerCPM()/getGeigerMicroSievertHour() for backwards compatibility.
 */
class GeigerDriver : public SensorDriver {
 public:
  explicit GeigerDriver(Sensors &s) : SensorDriver(s) {}
  ~GeigerDriver() override = default;

  const char *name() const override { return "CAJOE"; }
  SENSORS type() const override { return SENSORS::SCAJOE; }

  /// Attach the counter to a GPIO. Returns false for an invalid pin.
  bool enable(int gpio);
  bool begin() override;
  bool read() override;
  bool enabled() const override;

  /// Release the counter (used before a full Sensors::init()).
  void detach();
  void clear();
  uint32_t cpm() const;
  float microSievertHour() const;

};

#endif  // GeigerDriver_h
