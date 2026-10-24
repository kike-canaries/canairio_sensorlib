// DFRobot Gravity electrochemical gas sensors (CO, NH3, NO2, O3).
#include "GasDrivers.h"

#include <math.h>

#include "../../Sensors.hpp"

bool DfrobotGasDriver::beginGas(DFRobot_GAS_I2C &gas, uint8_t address, SENSORS sensor) {
  _sensors.sensorAnnounce(sensor);
  gas = DFRobot_GAS_I2C(&Wire, address);
  if (!gas.begin()) return false;
  // Mode of obtaining data: the main controller needs to request the sensor for data
  gas.changeAcquireMode(gas.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  if (hasExternalTempSensor())
    gas.setTempCompensation(gas.ON);
  else
    gas.setTempCompensation(gas.OFF);
  _sensors.sensorRegister(sensor);
  return true;
}

bool DfrobotGasDriver::readGas(DFRobot_GAS_I2C &gas, float &value, uint8_t gasType,
                               UNIT unit) {
  float rawPpm = gas.readGasConcentrationPPM();
  float internalTemp = gas.readTempC();
  bool external = hasExternalTempSensor();
  float compensationTemp = external ? _sensors.temp : (internalTemp - _sensors.toffset);

  value = tempCompensation(rawPpm, compensationTemp, gasType);
  if (external && _sensors.humi > 0.0f)
    value = humidityCompensation(value, _sensors.humi, gasType);
  if (_sensors.pres > 0.0) value = pressCompensation(value, _sensors.pres);
  _sensors.unitRegister(unit);
  _sensors.dataReady = true;
  if (!external) {
    _sensors.temp = internalTemp - _sensors.toffset;
    _sensors.tempRegister(false);
  }
  return true;
}

/**
 * @brief Check if any non-DFRobot sensor that provides ambient temperature
 *        is registered and read before the DFRobot sensors in readAllSensors().
 *
 * Unlike checking isUnitRegistered(UNIT::TEMP), this avoids a subtle bug:
 * the first DFRobot read in a cycle would register UNIT::TEMP itself, causing
 * all subsequent DFRobot reads (and all future cycles) to believe an external
 * sensor is present and stop refreshing the temperature from readTempC().
 */
bool DfrobotGasDriver::hasExternalTempSensor() const {
  return _sensors.isSensorRegistered(SENSORS::SBME280) ||
         _sensors.isSensorRegistered(SENSORS::SBMP280) ||
         _sensors.isSensorRegistered(SENSORS::SBME680) ||
         _sensors.isSensorRegistered(SENSORS::SSHT31) ||
         _sensors.isSensorRegistered(SENSORS::SAHTXX) ||
         _sensors.isSensorRegistered(SENSORS::SAM232X) ||
         _sensors.isSensorRegistered(SENSORS::SSEN5X) ||
         _sensors.isSensorRegistered(SENSORS::P5003T);
}

/**
 * @brief Temperature compensation for DFRobot gas sensors using external temperature.
 *
 * Reimplements the DFRobot library formulas (from DFRobot_MultiGasSensor.cpp)
 * so we can feed the current ambient temperature from an external sensor
 * (BME280, SHT31, etc.) instead of the stale onboard thermistor value that
 * the library captures only once at init.
 */
float DfrobotGasDriver::tempCompensation(float rawPpm, float temperature, uint8_t gasType) {
  static const float DFR_TEMP_MIN = -20.0f;
  static const float DFR_TEMP_MAX = 40.0f;

  if (temperature <= DFR_TEMP_MIN) temperature = DFR_TEMP_MIN + 0.01f;
  if (temperature > DFR_TEMP_MAX) temperature = DFR_TEMP_MAX;

  float gainDivisor = 1.0f;
  float baselineOffset = 0.0f;

  switch (gasType) {
    case DFRobot_GAS::CO:
      gainDivisor = 0.005f * temperature + 0.9f;
      if (temperature > 20) baselineOffset = 0.3f * temperature - 6.0f;
      break;

    case DFRobot_GAS::NH3:
      if (temperature <= 0) {
        gainDivisor = 0.006f * temperature + 0.95f;
        baselineOffset = -0.006f * temperature + 0.25f;
      } else if (temperature <= 20) {
        gainDivisor = 0.006f * temperature + 0.95f;
        baselineOffset = -0.012f * temperature + 0.25f;
      } else {
        gainDivisor = 0.005f * temperature + 1.08f;
        baselineOffset = -0.1f * temperature + 2.0f;
      }
      break;

    case DFRobot_GAS::NO2:
      gainDivisor = 0.005f * temperature + 0.9f;
      if (temperature <= 0) {
        baselineOffset = -0.0025f * temperature + 0.005f;
      } else if (temperature <= 20) {
        baselineOffset = 0.005f * temperature + 0.005f;
      } else {
        baselineOffset = 0.0025f * temperature + 0.1f;
      }
      break;

    case DFRobot_GAS::O3:
      if (temperature <= 0) {
        gainDivisor = 0.015f * temperature + 1.1f;
        baselineOffset = 0.05f;
      } else if (temperature <= 20) {
        gainDivisor = 1.1f;
        baselineOffset = 0.01f * temperature;
      } else {
        gainDivisor = 1.1f;
        baselineOffset = -0.005f * temperature + 0.3f;
      }
      break;

    default:
      break;
  }

  float scaled = rawPpm / gainDivisor;
  float compensated = scaled - baselineOffset;

  if (compensated < 0) return (scaled > 0) ? scaled : 0.0f;
  return compensated;
}

/**
 * @brief Pressure compensation for electrochemical gas sensors.
 *
 * Corrects for the effect of barometric pressure on the partial pressure of
 * the target gas. At higher pressure more molecules reach the electrode,
 * inflating the apparent concentration.
 */
float DfrobotGasDriver::pressCompensation(float ppm, float pressure) {
  static const float STANDARD_PRESSURE_HPA = 1013.25f;
  if (pressure <= 0.0f) return ppm;
  return ppm * (STANDARD_PRESSURE_HPA / pressure);
}

/**
 * @brief Humidity compensation for DFRobot electrochemical gas sensors.
 *
 * DFRobot does NOT publish official RH coefficients, so by default this
 * function is a NO-OP (coefficients = 0) and returns the input unchanged.
 * Fill the coefficients once derived from field regression.
 *
 * Model: ppm_out = (ppm / (1 + k_gain*(RH-REF))) - k_off*(RH-REF)
 */
float DfrobotGasDriver::humidityCompensation(float ppm, float humidity, uint8_t gasType) {
  static const float REF_HUMI = 50.0f;  // factory reference RH (%)
  if (humidity <= 0.0f || humidity > 100.0f) return ppm;

  const float delta = humidity - REF_HUMI;

  // Default coefficients = 0 (NO-OP). Replace with your regression results.
  float kGain = 0.0f;  // sensitivity drift per %RH
  float kOff = 0.0f;   // baseline drift per %RH

  switch (gasType) {
    case DFRobot_GAS::NH3:
      // kGain = 0.0015f; kOff = 0.002f;
      break;
    case DFRobot_GAS::NO2:
      // kGain = 0.0010f; kOff = 0.001f;
      break;
    case DFRobot_GAS::O3:
      // kGain = 0.0020f; kOff = 0.0015f;
      break;
    case DFRobot_GAS::CO:
      // kGain = 0.0005f; kOff = 0.0f;
      break;
    default:
      break;
  }

  if (kGain == 0.0f && kOff == 0.0f) return ppm;  // nothing to do

  const float gainDivisor = 1.0f + kGain * delta;
  const float scaled = (gainDivisor != 0.0f) ? (ppm / gainDivisor) : ppm;
  const float compensated = scaled - kOff * delta;

  if (compensated < 0.0f) return (scaled > 0.0f) ? scaled : 0.0f;
  return compensated;
}

bool DfrobotCoDriver::begin() {
  return beginGas(_sensors.dfrCO, DFROBOT_CO_I2C_ADDR, SENSORS::SDFRCO);
}
bool DfrobotCoDriver::read() {
  if (!enabled()) return false;
  return readGas(_sensors.dfrCO, _sensors.co, DFRobot_GAS::CO, UNIT::CO);
}

bool DfrobotNh3Driver::begin() {
  return beginGas(_sensors.dfrNH3, DFROBOT_NH3_I2C_ADDR, SENSORS::SDFRNH3);
}
bool DfrobotNh3Driver::read() {
  if (!enabled()) return false;
  return readGas(_sensors.dfrNH3, _sensors.nh3, DFRobot_GAS::NH3, UNIT::NH3);
}

bool DfrobotNo2Driver::begin() {
  return beginGas(_sensors.dfrNO2, DFROBOT_NO2_I2C_ADDR, SENSORS::SDFRNO2);
}
bool DfrobotNo2Driver::read() {
  if (!enabled()) return false;
  return readGas(_sensors.dfrNO2, _sensors.no2, DFRobot_GAS::NO2, UNIT::NO2);
}

bool DfrobotO3Driver::begin() {
  return beginGas(_sensors.dfrO3, DFROBOT_O3_I2C_ADDR, SENSORS::SDFRO3);
}
bool DfrobotO3Driver::read() {
  if (!enabled()) return false;
  return readGas(_sensors.dfrO3, _sensors.o3, DFRobot_GAS::O3, UNIT::O3);
}
