#include "SensorDFRobotGas.hpp"

SensorDFRobotGas::SensorDFRobotGas(uint8_t address, SENSORS type, const char* name, uint8_t gasType,
                                   TwoWire* wire)
    : _driver(wire, address), _type(type), _name(name), _gasType(gasType) {}

bool SensorDFRobotGas::init() {
  if (!_driver.begin()) return false;
  _driver.changeAcquireMode(_driver.PASSIVITY);
  delay(500);
  _driver.setTempCompensation(_driver.OFF);
  _available = true;
  return true;
}

bool SensorDFRobotGas::read() {
  float rawPpm = _driver.readGasConcentrationPPM();
  _internalTemp = _driver.readTempC();

  float compensationTemp = _hasAmbient ? _ambientTemp : _internalTemp;
  _concentration = tempCompensation(rawPpm, compensationTemp);

  if (_hasAmbient && _ambientPres > 0.0f) {
    _concentration = pressCompensation(_concentration, _ambientPres);
  }
  return true;
}

void SensorDFRobotGas::setAmbientData(float temp, float hum, float pres) {
  (void)hum;
  _ambientTemp = temp;
  _ambientPres = pres;
  _hasAmbient = true;
}

/**
 * Reimplements DFRobot library temperature compensation formulas so we can
 * use an external temperature source.  Each formula splits into a gain
 * divisor (sensitivity drift) and a baseline offset (zero-point drift).
 */
float SensorDFRobotGas::tempCompensation(float rawPpm, float temperature) {
  static constexpr float TEMP_MIN = -20.0f;
  static constexpr float TEMP_MAX = 40.0f;

  if (temperature <= TEMP_MIN) temperature = TEMP_MIN + 0.01f;
  if (temperature > TEMP_MAX) temperature = TEMP_MAX;

  float gainDivisor = 1.0f;
  float baselineOffset = 0.0f;

  switch (_gasType) {
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

float SensorDFRobotGas::pressCompensation(float ppm, float pressure) {
  static constexpr float STANDARD_PRESSURE_HPA = 1013.25f;
  if (pressure <= 0.0f) return ppm;
  return ppm * (STANDARD_PRESSURE_HPA / pressure);
}
