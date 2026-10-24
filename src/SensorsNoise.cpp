// CanAirIO sensorlib — noise public getters and time-sync facade.
#include "Sensors.hpp"

float Sensors::getNoise() const { return noiseInstant; }

float Sensors::getNoisePeak() const { return noisePeakValue; }

float Sensors::getNoiseLegalAverage() const { return noiseAvgLegalValue; }

float Sensors::getNoiseL90() const { return noiseL90Value; }

float Sensors::getNoiseLASmax() const { return noiseLASmaxValue; }

float Sensors::getNoiseLCpeak() const { return noiseLCpeakValue; }

float Sensors::getNoiseLd() const { return noiseLdValue; }

float Sensors::getNoiseLe() const { return noiseLeValue; }

float Sensors::getNoiseLn() const { return noiseLnValue; }

float Sensors::getNoiseLden() const { return noiseLdenValue; }

bool Sensors::sendNoiseSensorTime(uint32_t unixTime) {
  return _noise.sendTime(unixTime);
}

bool Sensors::syncNoiseSensorTime() { return _noise.syncTime(); }

void Sensors::setNoiseSensorTimeSyncInterval(uint32_t intervalMs) {
  _noise.setTimeSyncInterval(intervalMs);
}
