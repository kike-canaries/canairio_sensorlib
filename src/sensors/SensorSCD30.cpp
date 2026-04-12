#include "SensorSCD30.hpp"

bool SensorSCD30::init() {
#ifndef Wire1
  _available = _scd30.begin();
#else
  _available = _scd30.begin() || _scd30.begin(SCD30_I2CADDR_DEFAULT, &Wire1, SCD30_CHIP_ID);
#endif
  if (_available) delay(10);
  return _available;
}

bool SensorSCD30::read() {
  if (!_scd30.dataReady() || !_scd30.read()) return false;
  uint16_t rawCO2 = _scd30.CO2;
  if (rawCO2 == 0) return false;
  _co2 = rawCO2;
  _humidity = _scd30.relative_humidity;
  _temperature = _scd30.temperature;
  return true;
}

void SensorSCD30::setTemperatureOffset(float offset) { _scd30.setTemperatureOffset(offset); }

void SensorSCD30::setAltitude(float altitude) {
  _scd30.setAltitudeOffset(static_cast<uint16_t>(altitude));
}

float SensorSCD30::getTemperatureOffset() { return _scd30.getTemperatureOffset() / 100.0f; }

uint16_t SensorSCD30::getAltitudeOffset() { return _scd30.getAltitudeOffset(); }

void SensorSCD30::setMeasurementInterval(uint16_t seconds) {
  _scd30.setMeasurementInterval(seconds);
}

void SensorSCD30::forceRecalibration(int ppmValue) {
  _scd30.forceRecalibrationWithReference(ppmValue);
}
