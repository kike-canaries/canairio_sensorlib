#include "SensorSCD4x.hpp"

bool SensorSCD4x::init() {
  _scd4x.begin(Wire);
  uint16_t error = _scd4x.stopPeriodicMeasurement();
  if (error) return false;
  _available = true;
  return true;
}

bool SensorSCD4x::read() {
  uint16_t rawCO2 = 0;
  float rawTemp = 0.0f, rawHum = 0.0f;
  uint16_t error = _scd4x.readMeasurement(rawCO2, rawTemp, rawHum);
  if (error || rawCO2 == 0) return false;
  _co2 = rawCO2;
  _temperature = rawTemp;
  _humidity = rawHum;
  return true;
}

void SensorSCD4x::setTemperatureOffset(float offset) {
  _scd4x.stopPeriodicMeasurement();
  delay(510);
  _scd4x.setTemperatureOffset(offset);
  _scd4x.startPeriodicMeasurement();
}

void SensorSCD4x::setAltitude(float altitude) {
  _scd4x.stopPeriodicMeasurement();
  delay(510);
  _scd4x.setSensorAltitude(static_cast<uint16_t>(altitude));
  _scd4x.startPeriodicMeasurement();
}

float SensorSCD4x::getTemperatureOffset() {
  float offset = 0.0f;
  _scd4x.stopPeriodicMeasurement();
  _scd4x.getTemperatureOffset(offset);
  _scd4x.startPeriodicMeasurement();
  return offset;
}

uint16_t SensorSCD4x::getSensorAltitude() {
  uint16_t altitude = 0;
  _scd4x.getSensorAltitude(altitude);
  return altitude;
}

bool SensorSCD4x::calibrate(int ppmValue) {
  uint16_t frcCorrection = 0;
  _scd4x.stopPeriodicMeasurement();
  delay(510);
  uint16_t error =
      _scd4x.performForcedRecalibration(static_cast<uint16_t>(ppmValue), frcCorrection);
  delay(50);
  _scd4x.startPeriodicMeasurement();
  return (error == 0);
}

bool SensorSCD4x::startPeriodicMeasurement() { return (_scd4x.startPeriodicMeasurement() == 0); }
