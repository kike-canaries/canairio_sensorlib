// Sensirion CO2 / VOC sensors (SCD30, SCD4x, SEN5x, SGP41).
#include "Co2Drivers.h"

#include <math.h>

#include "../../Sensors.hpp"

bool Scd30Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SSCD30);
#ifndef Wire1
  if (!_sensors.scd30.begin()) return false;
#else
  if (!_sensors.scd30.begin() &&
      !_sensors.scd30.begin(SCD30_I2CADDR_DEFAULT, &Wire1, SCD30_CHIP_ID))
    return false;
#endif
  delay(10);

  _sensors.sensorRegister(SENSORS::SSCD30);

  _sensors.DEBUG("-->[SLIB] SCD30 Temp offset\t:",
                 String(_sensors.scd30.getTemperatureOffset()).c_str());
  _sensors.DEBUG("-->[SLIB] SCD30 Altitude offset\t:",
                 String(_sensors.scd30.getAltitudeOffset()).c_str());

  if (_sensors.scd30.getAltitudeOffset() != uint16_t(_sensors.altoffset)) {
    _sensors.DEBUG("-->[SLIB] SCD30 altitude offset to\t:",
                   String(_sensors.altoffset).c_str());
    setAltitudeOffset(_sensors.altoffset);
    delay(10);
  }

  if (uint16_t((_sensors.scd30.getTemperatureOffset())) !=
      (uint16_t(_sensors.toffset * 100))) {
    _sensors.DEBUG("-->[SLIB] SCD30 Temp offset to\t:", String(_sensors.toffset).c_str());
    // Adafruit_SCD30::setTemperatureOffset() expects hundredths of a degree C
    setTempOffset(_sensors.toffset * 100);
    delay(10);
  }
  return true;
}

bool Scd30Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SSCD30)) return false;
  if (!_sensors.scd30.dataReady() || !_sensors.scd30.read()) return false;
  uint16_t tCO2 = _sensors.scd30.CO2;  // we need temp var, without it override CO2
  if (tCO2 > 0) {
    _sensors.CO2Val = tCO2;
    _sensors.CO2humi = _sensors.scd30.relative_humidity;
    _sensors.CO2temp = _sensors.scd30.temperature;
    _sensors.dataReady = true;
    _sensors.DEBUG("-->[SLIB] SCD30 read\t\t: done!");
    _sensors.tempRegister(true);
    _sensors.unitRegister(UNIT::CO2);
    _sensors.unitRegister(UNIT::CO2HUM);
    return true;
  }
  return false;
}

void Scd30Driver::setTempOffset(float offset) {
  if (_sensors.isSensorRegistered(SENSORS::SSCD30)) {
    Serial.println("-->[SLIB] SCD30 new temp offset\t: " + String(offset));
    _sensors.scd30.setTemperatureOffset(offset);
  }
}

float Scd30Driver::getTempOffset() const {
  float offset = 0.0;
  if (_sensors.isSensorRegistered(SENSORS::SSCD30)) {
    // Cast away const because Adafruit_SCD30::getTemperatureOffset() is not const
    offset = const_cast<Adafruit_SCD30 &>(_sensors.scd30).getTemperatureOffset() / 100.0;
    Serial.println("-->[SLIB] SCD30 get temp offset\t: " + String(offset));
  }
  return offset;
}

void Scd30Driver::setAltitudeOffset(float offset) {
  if (_sensors.isSensorRegistered(SENSORS::SSCD30)) {
    Serial.println("-->[SLIB] SCD30 new altitude offset\t: " + String(offset));
    _sensors.scd30.setAltitudeOffset(uint16_t(offset));
  }
}

bool Scd4xDriver::begin() {
  _sensors.sensorAnnounce(SENSORS::SSCD4X);
  float tTemperatureOffset, offsetDifference;
  uint16_t tSensorAltitude;
  uint16_t error;
  _sensors.scd4x.begin(Wire);
  error = _sensors.scd4x.stopPeriodicMeasurement();
  if (error) return false;
  _sensors.sensorRegister(SENSORS::SSCD4X);
  _sensors.scd4x.getTemperatureOffset(tTemperatureOffset);
  _sensors.scd4x.getSensorAltitude(tSensorAltitude);
  _sensors.DEBUG("-->[SLIB] SCD4x Temp offset\t:", String(tTemperatureOffset).c_str());
  _sensors.DEBUG("-->[SLIB] SCD4x Alt offset \t:", String(tSensorAltitude).c_str());

  if (tSensorAltitude != uint16_t(_sensors.altoffset))
    setAltitudeOffset(_sensors.altoffset);

  offsetDifference = abs((_sensors.toffset * 100) - (tTemperatureOffset * 100));
  if (offsetDifference >
      0.5) {  // Accounts for SCD4x conversion rounding errors in temperature offset
    Serial.println("-->[SLIB] SCD4x new offset\t: Temp offset to " +
                   String(_sensors.toffset));
    setTempOffset(_sensors.toffset);
  }
  error = _sensors.scd4x.startPeriodicMeasurement();
  if (error)
    _sensors.DEBUG("[W][SLIB] SCD4x periodic measure\t: starting error:",
                   String(error).c_str());
  return true;
}

bool Scd4xDriver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SSCD4X)) return false;
  uint16_t tCO2 = 0;
  float tCO2temp, tCO2humi = 0;
  uint16_t error = _sensors.scd4x.readMeasurement(tCO2, tCO2temp, tCO2humi);
  if (error) return false;
  _sensors.CO2Val = tCO2;
  _sensors.CO2humi = tCO2humi;
  _sensors.CO2temp = tCO2temp;
  _sensors.dataReady = true;
  _sensors.DEBUG("-->[SLIB] SCD4x read\t\t: done!");
  _sensors.tempRegister(true);
  _sensors.unitRegister(UNIT::CO2);
  _sensors.unitRegister(UNIT::CO2HUM);
  return true;
}

void Scd4xDriver::setTempOffset(float offset) {
  if (_sensors.isSensorRegistered(SENSORS::SSCD4X)) {
    Serial.println("-->[SLIB] SCD4x new temperature offset\t: " + String(offset));
    _sensors.scd4x.stopPeriodicMeasurement();
    delay(510);
    _sensors.scd4x.setTemperatureOffset(offset);
    _sensors.scd4x.startPeriodicMeasurement();
  }
}

float Scd4xDriver::getTempOffset() const {
  float offset = 0.0;
  if (_sensors.isSensorRegistered(SENSORS::SSCD4X)) {
    uint16_t error = _sensors.scd4x.stopPeriodicMeasurement();
    if (error) {
      _sensors.DEBUG("[SLIB] SCD4x stopPeriodicMeasurement()\t: error:",
                     String(error).c_str());
    } else {
      _sensors.scd4x.getTemperatureOffset(offset);
    }
    _sensors.scd4x.startPeriodicMeasurement();
  }
  return offset;
}

void Scd4xDriver::setAltitudeOffset(float offset) {
  if (_sensors.isSensorRegistered(SENSORS::SSCD4X)) {
    Serial.println("-->[SLIB] SCD4x new altitude offset\t: " + String(offset));
    _sensors.scd4x.stopPeriodicMeasurement();
    delay(510);
    _sensors.scd4x.setSensorAltitude(uint16_t(offset));
    _sensors.scd4x.startPeriodicMeasurement();
  }
}

bool Sen5xDriver::begin() {
  _sensors.sensorAnnounce(SENSORS::SSEN5X);
  _sensors.sen5x.begin(Wire);
  uint16_t error;
  error = _sensors.sen5x.deviceReset();
  if (error) return false;
  float tempOffset = 0.0;
  _sensors.sen5x.getTemperatureOffsetSimple(tempOffset);
  _sensors.DEBUG("-->[SLIB] SEN5X Temp offset\t:", String(tempOffset).c_str());
  if (uint16_t((tempOffset * 100)) != (uint16_t(_sensors.toffset * 100))) {
    _sensors.sen5x.setTemperatureOffsetSimple(_sensors.toffset);
    delay(10);
  }
  error = _sensors.sen5x.startMeasurement();
  if (error) {
    _sensors.DEBUG("[E][SLIB] Error trying to execute startMeasurement():");
    return false;
  }
  _sensors.sensorRegister(SENSORS::SSEN5X);
  return true;
}

bool Sen5xDriver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SSEN5X)) return false;
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;

  uint16_t error = _sensors.sen5x.readMeasuredValues(
      massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
      massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex);

  if (error) {
    _sensors.DEBUG("[E][SLIB] SEN5x read error!");
    return false;
  }

  _sensors.pm1 = (u_int16_t)massConcentrationPm1p0;
  _sensors.pm25 = (u_int16_t)massConcentrationPm2p5;
  _sensors.pm4 = (u_int16_t)massConcentrationPm4p0;
  _sensors.pm10 = (u_int16_t)massConcentrationPm10p0;
  _sensors.voci = vocIndex;
  _sensors.noxi = noxIndex;
  _sensors.temp = ambientTemperature - _sensors.toffset;
  _sensors.humi = ambientHumidity;
  _sensors.dataReady = true;
  _sensors.DEBUG("-->[SLIB] SEN5x read\t\t: done!");
  _sensors.unitRegister(UNIT::PM1);
  _sensors.unitRegister(UNIT::PM25);
  _sensors.unitRegister(UNIT::PM4);
  _sensors.unitRegister(UNIT::PM10);
  _sensors.unitRegister(UNIT::TEMP);
  _sensors.unitRegister(UNIT::HUM);
  _sensors.unitRegister(UNIT::VOCI);
  _sensors.unitRegister(UNIT::NOXI);
  return true;
}

void Sen5xDriver::setTempOffset(float offset) {
  if (_sensors.isSensorRegistered(SENSORS::SSEN5X)) {
    Serial.println("-->[SLIB] SEN5x new temperature offset\t: " + String(offset));
    _sensors.sen5x.stopMeasurement();
    _sensors.sen5x.setTemperatureOffsetSimple(offset);
    delay(510);
    _sensors.sen5x.startMeasurement();
  }
}

bool Sgp41Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SSGP41);
  uint16_t error;
  uint16_t testResult;
  _sensors.sgp41.begin(Wire);
  error = _sensors.sgp41.executeSelfTest(testResult);
  if (error) {
    _sensors.DEBUG("-->[SLIB] sgp41 selftest error\t:", String(error).c_str());
    return false;
  } else if (testResult != 0xD400) {
    _sensors.DEBUG("-->[SLIB] sgp41 selfTest error\t:", String(testResult).c_str());
    return false;
  }
  _sensors.sensorRegister(SENSORS::SSGP41);
  return true;
}

bool Sgp41Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SSGP41)) return false;

  uint16_t error;
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;

  bool isConditioning = (_sensors.conditioning_s > 0);

  if (isConditioning) {
    // During NOx conditioning (10s) SRAW NOx will remain 0
    error = _sensors.sgp41.executeConditioning(defaultRh, defaultT, _sensors.voc);
    _sensors.conditioning_s--;
  } else {
    // Read Measurement
    error = _sensors.sgp41.measureRawSignals(defaultRh, defaultT, _sensors.voc, _sensors.nox);
  }

  if (error) {
    _sensors.DEBUG("-->[SLIB] sgp41 measureRaw error\t:", String(error).c_str());
    return false;
  }

  // Process raw signals through the Sensirion gas index algorithms
  _sensors.voci = _sensors.vocAlgorithm.process(_sensors.voc);
  if (!isConditioning) {
    // NOx raw signal is only valid after conditioning period
    _sensors.noxi = _sensors.noxAlgorithm.process(_sensors.nox);
  }
  _sensors.dataReady = true;
  _sensors.unitRegister(UNIT::VOC);
  _sensors.unitRegister(UNIT::NOX);
  _sensors.unitRegister(UNIT::VOCI);
  _sensors.unitRegister(UNIT::NOXI);
  return true;
}