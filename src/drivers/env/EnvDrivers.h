#ifndef EnvDrivers_h
#define EnvDrivers_h

#include "../SensorDriver.h"

// Temperature / humidity / pressure sensors (I2C and the legacy DHT).
class Am2320Driver : public SensorDriver {
 public:
  explicit Am2320Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "AM232X"; }
  SENSORS type() const override { return SENSORS::SAM232X; }
  bool begin() override;
  bool read() override;
};

class Bme280Driver : public SensorDriver {
 public:
  explicit Bme280Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "BME280"; }
  SENSORS type() const override { return SENSORS::SBME280; }
  bool begin() override;
  bool read() override;
};

class Bmp280Driver : public SensorDriver {
 public:
  explicit Bmp280Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "BMP280"; }
  SENSORS type() const override { return SENSORS::SBMP280; }
  bool begin() override;
  bool read() override;
};

class Bme680Driver : public SensorDriver {
 public:
  explicit Bme680Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "BME680"; }
  SENSORS type() const override { return SENSORS::SBME680; }
  bool begin() override;
  bool read() override;
};

class AhtxxDriver : public SensorDriver {
 public:
  explicit AhtxxDriver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "AHTXX"; }
  SENSORS type() const override { return SENSORS::SAHTXX; }
  bool begin() override;
  bool read() override;
};

class Sht31Driver : public SensorDriver {
 public:
  explicit Sht31Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "SHT31"; }
  SENSORS type() const override { return SENSORS::SSHT31; }
  bool begin() override;
  bool read() override;
};

#ifdef DHT11_ENABLED
class Dht11Driver : public SensorDriver {
 public:
  explicit Dht11Driver(Sensors &s) : SensorDriver(s) {}
  const char *name() const override { return "DHTX"; }
  SENSORS type() const override { return SENSORS::SDHTX; }
  bool begin() override;
  bool read() override;
};
#endif

#endif  // EnvDrivers_h
