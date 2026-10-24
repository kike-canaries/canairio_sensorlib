# Driver architecture

The public `Sensors` class is a facade: it owns the registry, the public
getters/setters and the sampling loop. Sensor-specific work lives in driver
classes derived from `SensorDriver` (`src/drivers/SensorDriver.h`).

```
src/
  Sensors.hpp              public facade (API unchanged)
  SensorsTypes.hpp        UNIT/SENSORS enums, board pins, build defaults
  SensorsCore.cpp         lifecycle, init(), readAllSensors(), getters
  SensorsRegistry.cpp     sensor/unit registration and introspection
  SensorsPlatform.cpp     Wire/I2C setup, debug output, error callback
  SensorsNoise.cpp        noise public getters + time-sync facade
  SensorsGeiger.cpp       Geiger public facade
  SensorsCo2.cpp          generic CO2 altitude compensation helpers
  drivers/
    SensorDriver.h        base class: begin() / read() / enabled()
    env/EnvDrivers.*      AM232X, BME280, BMP280, BME680, AHTxx, SHT31, DHT
    co2/Co2Drivers.*      SCD30, SCD4x, SEN5x, SGP41 (+ offset setters)
    gas/GasDrivers.*      DFRobot CO / NH3 / NO2 / O3 (shared base class)
    pm/PmDrivers.*        SPS30 (I2C/UART) and GCJA5 (I2C)
    uart/UartDriver.*     UART autodetect and PM/CO2 protocols
    NoiseSensorDriver.*   I2C noise node protocol and clock policy
    GeigerDriver.*        CAJOE pulse counter
```

## Contract

Every driver implements:

* `const char *name()` and `SENSORS type()`.
* `bool begin()`: probe/initialise, register the sensor and return whether it
  is available. It must be idempotent because `Sensors::init()` can be called
  again.
* `bool read()`: read one sample, publish values through the facade, and return
  whether the sample was valid. Values must stay untouched on failure.

`SensorDriver::enabled()` defaults to `isSensorRegistered(type())`; override it
for drivers whose lifetime is not tied to the registry (UART selection, Geiger
pointer, noise node).

Drivers receive a `Sensors &` and are declared friends of the facade so they can
update the legacy measurement fields that back the public getters. This keeps
the external API (`sensors.getTemperature()`, `sensors.rad`, ...) untouched
while the implementation is split per sensor family.

## Adding a sensor

1. Add the type to `SENSORS_TYPES` in `SensorsTypes.hpp`.
2. Create a driver class in the matching `src/drivers/<family>/` header/cpp.
3. Add a member to `Sensors` and call `begin()`/`read()` from
   `SensorsCore.cpp` in the same place as the existing sensors.
4. Register the units from `read()`; use `tempRegister()` for temperature.
5. Add the driver to the friend list in `Sensors.hpp`.
6. Build with `pio run -e esp32 -e esp32c3 -e esp32s3` and test on hardware.

The CO2/gas/PM drivers are reference examples: they moved their protocol code
out of `Sensors` while keeping the exact public API.
