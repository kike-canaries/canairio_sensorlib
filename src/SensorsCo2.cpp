// CanAirIO sensorlib — SensorsCo2.cpp
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include "Sensors.hpp"

#include <math.h>
#include <string.h>

void Sensors::CO2scd30Read() {
  if (!isSensorRegistered(SENSORS::SSCD30)) return;
  if (!scd30.dataReady() || !scd30.read()) return;
  uint16_t tCO2 = scd30.CO2;  // we need temp var, without it override CO2
  if (tCO2 > 0) {
    CO2Val = tCO2;
    CO2humi = scd30.relative_humidity;
    CO2temp = scd30.temperature;
    dataReady = true;
    DEBUG("-->[SLIB] SCD30 read\t\t: done!");
    tempRegister(true);
    unitRegister(UNIT::CO2);
    unitRegister(UNIT::CO2HUM);
  }
}

void Sensors::sgp41Read() {
  if (!isSensorRegistered(SENSORS::SSGP41)) return;

  uint16_t error;
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;

  bool isConditioning = (conditioning_s > 0);

  if (isConditioning) {
    // During NOx conditioning (10s) SRAW NOx will remain 0
    error = sgp41.executeConditioning(defaultRh, defaultT, voc);
    conditioning_s--;
  } else {
    // Read Measurement
    error = sgp41.measureRawSignals(defaultRh, defaultT, voc, nox);
  }

  if (error) {
    DEBUG("-->[SLIB] sgp41 measureRaw error\t:", String(error).c_str());
    return;
  } else {
    // Process raw signals through the Sensirion gas index algorithms
    voci = vocAlgorithm.process(voc);
    if (!isConditioning) {
      // NOx raw signal is only valid after conditioning period
      noxi = noxAlgorithm.process(nox);
    }
    dataReady = true;
    unitRegister(UNIT::VOC);
    unitRegister(UNIT::NOX);
    unitRegister(UNIT::VOCI);
    unitRegister(UNIT::NOXI);
  }
}

void Sensors::CO2scd4xRead() {
  if (!isSensorRegistered(SENSORS::SSCD4X)) return;
  uint16_t tCO2 = 0;
  float tCO2temp, tCO2humi = 0;
  uint16_t error = scd4x.readMeasurement(tCO2, tCO2temp, tCO2humi);
  if (error) return;
  CO2Val = tCO2;
  CO2humi = tCO2humi;
  CO2temp = tCO2temp;
  dataReady = true;
  DEBUG("-->[SLIB] SCD4x read\t\t: done!");
  tempRegister(true);
  unitRegister(UNIT::CO2);
  unitRegister(UNIT::CO2HUM);
}

void Sensors::sen5xRead() {
  if (!isSensorRegistered(SENSORS::SSEN5X)) return;
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;

  uint16_t error = sen5x.readMeasuredValues(
      massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
      massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex);

  if (error) {
    DEBUG("[E][SLIB] SEN5x read error!");
    return;
  }

  pm1 = (u_int16_t)massConcentrationPm1p0;
  pm25 = (u_int16_t)massConcentrationPm2p5;
  pm4 = (u_int16_t)massConcentrationPm4p0;
  pm10 = (u_int16_t)massConcentrationPm4p0;
  voci = vocIndex;
  noxi = noxIndex;
  temp = ambientTemperature - toffset;
  humi = ambientHumidity;
  dataReady = true;
  DEBUG("-->[SLIB] SEN5x read\t\t: done!");
  unitRegister(UNIT::PM1);
  unitRegister(UNIT::PM25);
  unitRegister(UNIT::PM4);
  unitRegister(UNIT::PM10);
  unitRegister(UNIT::TEMP);
  unitRegister(UNIT::HUM);
  unitRegister(UNIT::VOCI);
  unitRegister(UNIT::NOXI);
}

/// Sensirion SCD30 CO2/T/H sensor init
void Sensors::CO2scd30Init() {
  sensorAnnounce(SENSORS::SSCD30);
#ifndef Wire1
  if (!scd30.begin()) return;
#else
  if (!scd30.begin() && !scd30.begin(SCD30_I2CADDR_DEFAULT, &Wire1, SCD30_CHIP_ID)) return;
#endif
  delay(10);

  sensorRegister(SENSORS::SSCD30);

  DEBUG("-->[SLIB] SCD30 Temp offset\t:", String(scd30.getTemperatureOffset()).c_str());
  DEBUG("-->[SLIB] SCD30 Altitude offset\t:", String(scd30.getAltitudeOffset()).c_str());

  if (scd30.getAltitudeOffset() != uint16_t(altoffset)) {
    DEBUG("-->[SLIB] SCD30 altitude offset to\t:", String(altoffset).c_str());
    setSCD30AltitudeOffset(altoffset);
    delay(10);
  }

  if (uint16_t((scd30.getTemperatureOffset())) != (uint16_t(toffset * 100))) {
    DEBUG("-->[SLIB] SCD30 Temp offset to\t:", String(toffset).c_str());
    setSCD30TempOffset(toffset);
    delay(10);
  }
}

/// set SCD30 temperature compensation
void Sensors::setSCD30TempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD30)) {
    Serial.println("-->[SLIB] SCD30 new temp offset\t: " + String(offset));
    scd30.setTemperatureOffset(offset);
  }
}

/// get SCD30 temperature compensation
float Sensors::getSCD30TempOffset() const {
  float offset = 0.0;
  if (isSensorRegistered(SENSORS::SSCD30)) {
    // Cast away const because Adafruit_SCD30::getTemperatureOffset() is not const
    offset = const_cast<Adafruit_SCD30 &>(scd30).getTemperatureOffset() / 100.0;
    Serial.println("-->[SLIB] SCD30 get temp offset\t: " + String(offset));
  }
  return offset;
}

/// set SCD30 altitude compensation
void Sensors::setSCD30AltitudeOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD30)) {
    Serial.println("-->[SLIB] SCD30 new altitude offset\t: " + String(offset));
    scd30.setAltitudeOffset(uint16_t(offset));
  }
}

void Sensors::sgp41Init() {
  sensorAnnounce(SENSORS::SSGP41);
  uint16_t error;
  uint16_t testResult;
  sgp41.begin(Wire);
  error = sgp41.executeSelfTest(testResult);
  if (error) {
    DEBUG("-->[SLIB] sgp41 selftest error\t:", String(error).c_str());
    return;
  } else if (testResult != 0xD400) {
    DEBUG("-->[SLIB] sgp41 selfTest error\t:", String(testResult).c_str());
    return;
  }
  sensorRegister(SENSORS::SSGP41);
}

/// Sensirion SCD4X CO2 sensor init
void Sensors::CO2scd4xInit() {
  sensorAnnounce(SENSORS::SSCD4X);
  float tTemperatureOffset, offsetDifference;
  uint16_t tSensorAltitude;
  uint16_t error;
  scd4x.begin(Wire);
  error = scd4x.stopPeriodicMeasurement();
  if (error) return;
  sensorRegister(SENSORS::SSCD4X);
  scd4x.getTemperatureOffset(tTemperatureOffset);
  scd4x.getSensorAltitude(tSensorAltitude);
  DEBUG("-->[SLIB] SCD4x Temp offset\t:", String(tTemperatureOffset).c_str());
  DEBUG("-->[SLIB] SCD4x Alt offset \t:", String(tSensorAltitude).c_str());

  if (tSensorAltitude != uint16_t(altoffset)) setSCD4xAltitudeOffset(altoffset);

  offsetDifference = abs((toffset * 100) - (tTemperatureOffset * 100));
  if (offsetDifference >
      0.5) {  // Accounts for SCD4x conversion rounding errors in temperature offset
    Serial.println("-->[SLIB] SCD4x new offset\t: Temp offset to " + String(toffset));
    setSCD4xTempOffset(toffset);
  }
  error = scd4x.startPeriodicMeasurement();
  if (error) DEBUG("[W][SLIB] SCD4x periodic measure\t: starting error:", String(error).c_str());
}

/// set SCD4x temperature compensation
void Sensors::setSCD4xTempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    Serial.println("-->[SLIB] SCD4x new temperature offset\t: " + String(offset));
    scd4x.stopPeriodicMeasurement();
    delay(510);
    scd4x.setTemperatureOffset(offset);
    scd4x.startPeriodicMeasurement();
  }
}

/// get SCD4x temperature compensation
float Sensors::getSCD4xTempOffset() const {
  float offset = 0.0;
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    // We cannot call stop/start measurements here if we want this method to be const
    // because they are not const methods in the library.
    // However, if we really need to read it, we might have to use a cached value
    // or cast away const if we are sure it is safe.
    // For now, let's try to just read it without stopping if the lib allows,
    // but the library says it must be stopped.
    // Since this is a refactoring, maybe we should just not make it const if it has side effects.
    // BUT the calling method getTempOffset() IS const.
    // Let's use a workaround for now: cast away const for the sub-calls.
    auto *nonConstThis = const_cast<Sensors *>(this);
    uint16_t error = nonConstThis->scd4x.stopPeriodicMeasurement();
    if (error) {
      DEBUG("[SLIB] SCD4x stopPeriodicMeasurement()\t: error:", String(error).c_str());
    } else {
      nonConstThis->scd4x.getTemperatureOffset(offset);
    }
    nonConstThis->scd4x.startPeriodicMeasurement();
  }
  return offset;
}

/// set SCD4x altitude compensation
void Sensors::setSCD4xAltitudeOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    Serial.println("-->[SLIB] SCD4x new altitude offset\t: " + String(offset));
    scd4x.stopPeriodicMeasurement();
    delay(510);
    scd4x.setSensorAltitude(uint16_t(offset));
    scd4x.startPeriodicMeasurement();
  }
}

/// Panasonic SEN5X sensor init
void Sensors::sen5xInit() {
  sensorAnnounce(SENSORS::SSEN5X);
  sen5x.begin(Wire);
  uint16_t error;
  error = sen5x.deviceReset();
  if (error) return;
  float tempOffset = 0.0;
  sen5x.getTemperatureOffsetSimple(tempOffset);
  DEBUG("-->[SLIB] SEN5X Temp offset\t:", String(tempOffset).c_str());
  if (uint16_t((tempOffset * 100)) != (uint16_t(toffset * 100))) {
    sen5x.setTemperatureOffsetSimple(toffset);
    delay(10);
  }
  error = sen5x.startMeasurement();
  if (error) {
    DEBUG("[E][SLIB] Error trying to execute startMeasurement():");
    return;
  }
  sensorRegister(SENSORS::SSEN5X);
}

/// set SEN5X temperature compensation
void Sensors::setsen5xTempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSEN5X)) {
    Serial.println("-->[SLIB] SEN5x new temperature offset\t: " + String(offset));
    sen5x.stopMeasurement();
    sen5x.setTemperatureOffsetSimple(offset);
    delay(510);
    sen5x.startMeasurement();
  }
}

// Altitude compensation for CO2 sensors without Pressure atm or Altitude compensation
void Sensors::CO2correctionAlt() {
  DEBUG("-->[SLIB] CO2 altitude original\t:", String(CO2Val).c_str());
  float CO2cor = (0.016 * ((1013.25 - hpa) / 10) * (CO2Val - 400)) +
                 CO2Val;  // Increment of 1.6% for every hpa of difference at sea level
  CO2Val = round(CO2cor);
  DEBUG("-->[SLIB] CO2 compensated\t:", String(CO2Val).c_str());
}

/// hPa hectopascal calculation based on the altitude. See CO2AltitudeOffset setter
float Sensors::hpaCalculation(float altitude) {
  DEBUG("-->[SLIB] CO2 altitude offset\t:", String(altitude).c_str());
  float hpa =
      1012 - 0.118 * altitude +
      0.00000473 * altitude *
          altitude;  // Quadratic regression formula obtained PA (hpa) from high above the sea
  DEBUG("-->[SLIB] CO2 pressure (hPa)\t:", String(hpa).c_str());
  return hpa;
}
