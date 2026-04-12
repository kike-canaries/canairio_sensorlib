#include "SensorSEN5x.hpp"

bool SensorSEN5x::init() {
  _sen5x.begin(Wire);
  uint16_t error = _sen5x.deviceReset();
  if (error) return false;
  delay(100);
  error = _sen5x.startMeasurement();
  if (error) return false;
  _available = true;
  return true;
}

bool SensorSEN5x::read() {
  float pm1, pm25, pm4, pm10, humidity, temperature, vocIdx, noxIdx;
  uint16_t error =
      _sen5x.readMeasuredValues(pm1, pm25, pm4, pm10, humidity, temperature, vocIdx, noxIdx);
  if (error) return false;
  _pm1 = static_cast<uint16_t>(pm1);
  _pm25 = static_cast<uint16_t>(pm25);
  _pm4 = static_cast<uint16_t>(pm4);
  _pm10 = static_cast<uint16_t>(pm10);
  _temp = temperature;
  _humi = humidity;
  _vocIndex = vocIdx;
  _noxIndex = noxIdx;
  return true;
}

void SensorSEN5x::setTemperatureOffset(float offset) { _sen5x.setTemperatureOffsetSimple(offset); }
