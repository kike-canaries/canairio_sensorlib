// CanAirIO sensorlib — SensorsGas.cpp
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include "Sensors.hpp"

#include <math.h>
#include <string.h>

void Sensors::DFRobotNH3Read() {
  if (!isSensorRegistered(SENSORS::SDFRNH3)) return;
  float rawPpm = dfrNH3.readGasConcentrationPPM();
  float dfrInternalTemp = dfrNH3.readTempC();
  bool hasExternalTempSensor = dfrHasExternalTempSensor();
  float compensationTemp = hasExternalTempSensor ? temp : (dfrInternalTemp - toffset);
  nh3 = dfrGasTempCompensation(rawPpm, compensationTemp, DFRobot_GAS::NH3);
  if (hasExternalTempSensor && humi > 0.0f)
    nh3 = dfrGasHumiCompensation(nh3, humi, DFRobot_GAS::NH3);
  if (pres > 0.0) nh3 = dfrGasPressCompensation(nh3, pres);
  unitRegister(UNIT::NH3);
  dataReady = true;
  if (!hasExternalTempSensor) {
    temp = dfrInternalTemp - toffset;
    tempRegister(false);
  }
}

void Sensors::DFRobotCORead() {
  if (!isSensorRegistered(SENSORS::SDFRCO)) return;
  float rawPpm = dfrCO.readGasConcentrationPPM();
  float dfrInternalTemp = dfrCO.readTempC();
  bool hasExternalTempSensor = dfrHasExternalTempSensor();
  float compensationTemp = hasExternalTempSensor ? temp : (dfrInternalTemp - toffset);
  co = dfrGasTempCompensation(rawPpm, compensationTemp, DFRobot_GAS::CO);
  if (hasExternalTempSensor && humi > 0.0f) co = dfrGasHumiCompensation(co, humi, DFRobot_GAS::CO);
  if (pres > 0.0) co = dfrGasPressCompensation(co, pres);
  unitRegister(UNIT::CO);
  dataReady = true;
  if (!hasExternalTempSensor) {
    temp = dfrInternalTemp - toffset;
    tempRegister(false);
  }
}

void Sensors::DFRobotNO2Read() {
  if (!isSensorRegistered(SENSORS::SDFRNO2)) return;
  float rawPpm = dfrNO2.readGasConcentrationPPM();
  float dfrInternalTemp = dfrNO2.readTempC();
  bool hasExternalTempSensor = dfrHasExternalTempSensor();
  float compensationTemp = hasExternalTempSensor ? temp : (dfrInternalTemp - toffset);
  no2 = dfrGasTempCompensation(rawPpm, compensationTemp, DFRobot_GAS::NO2);
  if (hasExternalTempSensor && humi > 0.0f)
    no2 = dfrGasHumiCompensation(no2, humi, DFRobot_GAS::NO2);
  if (pres > 0.0) no2 = dfrGasPressCompensation(no2, pres);
  unitRegister(UNIT::NO2);
  dataReady = true;
  if (!hasExternalTempSensor) {
    temp = dfrInternalTemp - toffset;
    tempRegister(false);
  }
}

void Sensors::DFRobotO3Read() {
  if (!isSensorRegistered(SENSORS::SDFRO3)) return;
  float rawPpm = dfrO3.readGasConcentrationPPM();
  float dfrInternalTemp = dfrO3.readTempC();
  bool hasExternalTempSensor = dfrHasExternalTempSensor();
  float compensationTemp = hasExternalTempSensor ? temp : (dfrInternalTemp - toffset);
  o3 = dfrGasTempCompensation(rawPpm, compensationTemp, DFRobot_GAS::O3);
  if (hasExternalTempSensor && humi > 0.0f) o3 = dfrGasHumiCompensation(o3, humi, DFRobot_GAS::O3);
  if (pres > 0.0) o3 = dfrGasPressCompensation(o3, pres);
  unitRegister(UNIT::O3);
  dataReady = true;
  if (!hasExternalTempSensor) {
    temp = dfrInternalTemp - toffset;
    tempRegister(false);
  }
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
bool Sensors::dfrHasExternalTempSensor() const {
  return isSensorRegistered(SENSORS::SBME280) || isSensorRegistered(SENSORS::SBMP280) ||
         isSensorRegistered(SENSORS::SBME680) || isSensorRegistered(SENSORS::SSHT31) ||
         isSensorRegistered(SENSORS::SAHTXX) || isSensorRegistered(SENSORS::SAM232X) ||
         isSensorRegistered(SENSORS::SSEN5X) || isSensorRegistered(SENSORS::P5003T);
}

/**
 * @brief Temperature compensation for DFRobot gas sensors using external temperature.
 *
 * Reimplements the DFRobot library formulas (from DFRobot_MultiGasSensor.cpp)
 * so we can feed the current ambient temperature from an external sensor
 * (BME280, SHT31, etc.) instead of the stale onboard thermistor value that
 * the library captures only once at init.
 *
 * Each formula is split into a gain divisor (corrects sensitivity drift) and
 * a baseline offset (corrects zero-point drift).  When the offset would make
 * the result negative — common for low ambient NO2/O3/CO concentrations — we
 * fall back to the gain-only correction so the reading stays meaningful.
 *
 * @param rawPpm  Uncompensated gas concentration from readGasConcentrationPPM()
 * @param temperature  Ambient temperature in °C (from external sensor)
 * @param gasType  DFRobot gas type constant (DFRobot_GAS::CO, ::NH3, ::NO2, ::O3)
 * @return Compensated gas concentration in PPM
 */
float Sensors::dfrGasTempCompensation(float rawPpm, float temperature, uint8_t gasType) {
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
 * the target gas.  At higher pressure more molecules reach the electrode,
 * inflating the apparent concentration.
 *
 * @param ppm  Gas concentration after temperature compensation
 * @param pressure  Current barometric pressure in hPa (from BME280/BMP280/BME680)
 * @return Pressure-compensated gas concentration in PPM
 */
float Sensors::dfrGasPressCompensation(float ppm, float pressure) {
  static const float STANDARD_PRESSURE_HPA = 1013.25f;
  if (pressure <= 0.0f) return ppm;
  return ppm * (STANDARD_PRESSURE_HPA / pressure);
}

/**
 * @brief Humidity compensation for DFRobot electrochemical gas sensors.
 *
 * Electrochemical cells drift with relative humidity (cross-sensitivity),
 * most noticeably for NO2 and NH3. DFRobot does NOT publish official RH
 * coefficients, so by default this function is a NO-OP (coefficients = 0)
 * and returns the input unchanged — it will not introduce noise.
 *
 * Once you have the SHT31 logging in parallel with the official reference
 * station, derive real per-gas coefficients by regression and either fill
 * them in below or define SLIB_DFR_HUMI_COMP with your fitted values.
 *
 * Model: ppm_out = (ppm / (1 + k_gain*(RH-REF))) - k_off*(RH-REF)
 *
 * @param ppm   Gas concentration after temperature compensation
 * @param humidity  Relative humidity in % (from SHT31/BME280/etc.)
 * @param gasType   DFRobot gas type constant (::CO, ::NH3, ::NO2, ::O3)
 * @return Humidity-compensated gas concentration in PPM
 */
float Sensors::dfrGasHumiCompensation(float ppm, float humidity, uint8_t gasType) {
  static const float REF_HUMI = 50.0f;  // factory reference RH (%)
  if (humidity <= 0.0f || humidity > 100.0f) return ppm;

  const float delta = humidity - REF_HUMI;

  // Default coefficients = 0 (NO-OP). Replace with your regression results.
  float kGain = 0.0f;  // sensitivity drift per %RH
  float kOff = 0.0f;   // baseline drift per %RH

  switch (gasType) {
    case DFRobot_GAS::NH3:
      // kGain = 0.0015f; kOff = 0.002f;   // example placeholders
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

/// DFRobot GAS (CO) sensors init
void Sensors::DFRobotCOInit() {
  sensorAnnounce(SENSORS::SDFRCO);
  dfrCO = DFRobot_GAS_I2C(&Wire, DFROBOT_CO_I2C_ADDR);
  if (!dfrCO.begin()) return;
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrCO.changeAcquireMode(dfrCO.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  if (dfrHasExternalTempSensor())
    dfrCO.setTempCompensation(dfrCO.ON);
  else
    dfrCO.setTempCompensation(dfrCO.OFF);
  sensorRegister(SENSORS::SDFRCO);
}

/// DFRobot GAS (NH3) sensors init
void Sensors::DFRobotNH3Init() {
  sensorAnnounce(SENSORS::SDFRNH3);
  dfrNH3 = DFRobot_GAS_I2C(&Wire, DFROBOT_NH3_I2C_ADDR);
  if (!dfrNH3.begin()) return;
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrNH3.changeAcquireMode(dfrNH3.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  if (dfrHasExternalTempSensor())
    dfrNH3.setTempCompensation(dfrNH3.ON);
  else
    dfrNH3.setTempCompensation(dfrNH3.OFF);
  sensorRegister(SENSORS::SDFRNH3);
}

/// DFRobot GAS (NO2) sensors init
void Sensors::DFRobotNO2Init() {
  sensorAnnounce(SENSORS::SDFRNO2);
  dfrNO2 = DFRobot_GAS_I2C(&Wire, DFROBOT_NO2_I2C_ADDR);
  if (!dfrNO2.begin()) return;
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrNO2.changeAcquireMode(dfrNO2.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  if (dfrHasExternalTempSensor())
    dfrNO2.setTempCompensation(dfrNO2.ON);
  else
    dfrNO2.setTempCompensation(dfrNO2.OFF);
  sensorRegister(SENSORS::SDFRNO2);
}

/// DFRobot GAS (O3) sensors init
void Sensors::DFRobotO3Init() {
  sensorAnnounce(SENSORS::SDFRO3);
  dfrO3 = DFRobot_GAS_I2C(&Wire, DFROBOT_O3_I2C_ADDR);
  if (!dfrO3.begin()) return;
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrO3.changeAcquireMode(dfrO3.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  if (dfrHasExternalTempSensor())
    dfrO3.setTempCompensation(dfrO3.ON);
  else
    dfrO3.setTempCompensation(dfrO3.OFF);
  sensorRegister(SENSORS::SDFRO3);
}
