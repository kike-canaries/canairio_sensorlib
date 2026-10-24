#ifndef Co2Drivers_h
#define Co2Drivers_h

#include "../SensorDriver.h"

class Scd30Driver : public SensorDriver {
 public:
  explicit Scd30Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "SCD30"; }
  SENSORS type() const override { return SENSORS::SSCD30; }
  bool begin() override;
  bool read() override;

  void setTempOffset(float offset);
  float getTempOffset() const;
  void setAltitudeOffset(float offset);
};

class Scd4xDriver : public SensorDriver {
 public:
  explicit Scd4xDriver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "SCD4X"; }
  SENSORS type() const override { return SENSORS::SSCD4X; }
  bool begin() override;
  bool read() override;

  void setTempOffset(float offset);
  float getTempOffset() const;
  void setAltitudeOffset(float offset);
};

class Sen5xDriver : public SensorDriver {
 public:
  explicit Sen5xDriver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "SEN5X"; }
  SENSORS type() const override { return SENSORS::SSEN5X; }
  bool begin() override;
  bool read() override;

  void setTempOffset(float offset);
};

class Sgp41Driver : public SensorDriver {
 public:
  explicit Sgp41Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "SGP41"; }
  SENSORS type() const override { return SENSORS::SSGP41; }
  bool begin() override;
  bool read() override;
};

#endif  // Co2Drivers_h
