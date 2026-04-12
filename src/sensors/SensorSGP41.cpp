#include "SensorSGP41.hpp"

bool SensorSGP41::init() {
  _sgp41.begin(Wire);
  uint16_t testResult;
  uint16_t error = _sgp41.executeSelfTest(testResult);
  if (error || testResult != 0xD400) return false;
  _available = true;
  return true;
}

bool SensorSGP41::read() {
  uint16_t error;
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;

  if (_conditioningSeconds > 0) {
    error = _sgp41.executeConditioning(defaultRh, defaultT, _vocRaw);
    _conditioningSeconds--;
  } else {
    error = _sgp41.measureRawSignals(defaultRh, defaultT, _vocRaw, _noxRaw);
  }
  return (error == 0);
}
