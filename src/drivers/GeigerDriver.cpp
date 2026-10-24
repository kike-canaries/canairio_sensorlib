#include "GeigerDriver.h"

#include "../Sensors.hpp"

bool GeigerDriver::enable(int gpio) {
  _sensors.sensorAnnounce(SENSORS::SCAJOE);
  if (gpio < 0) {
    if (_sensors.devmode)
      Serial.printf("[W][SLIB] undefined Geiger pin\t: %i\r\n", gpio);
    return false;
  }
  if (_sensors.rad != nullptr) delete _sensors.rad;
  _sensors.rad = new GEIGER(gpio, _sensors.devmode);
  _sensors.sensorRegister(SENSORS::SCAJOE);
  return true;
}

bool GeigerDriver::begin() { return _sensors.rad != nullptr; }

bool GeigerDriver::read() {
  if (_sensors.rad != nullptr && _sensors.rad->read()) {
    _sensors.unitRegister(UNIT::CPM);
    _sensors.unitRegister(UNIT::RAD);
    return true;
  }
  return false;
}

bool GeigerDriver::enabled() const { return _sensors.rad != nullptr; }

void GeigerDriver::detach() {
  if (_sensors.rad != nullptr) {
    delete _sensors.rad;
    _sensors.rad = nullptr;
  }
}

void GeigerDriver::clear() {
  if (_sensors.rad != nullptr) _sensors.rad->clear();
}

uint32_t GeigerDriver::cpm() const { return _sensors.rad != nullptr ? _sensors.rad->getTics() : 0; }

float GeigerDriver::microSievertHour() const { return _sensors.rad != nullptr ? _sensors.rad->getUSvh() : 0.0f; }
