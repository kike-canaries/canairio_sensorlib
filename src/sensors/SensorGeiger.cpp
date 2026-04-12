#include "SensorGeiger.hpp"

SensorGeiger::SensorGeiger(int gpio, bool debug) {
  if (gpio >= 0) _driver = new GEIGER(gpio, debug);
}

SensorGeiger::~SensorGeiger() {
  if (_driver) delete _driver;
}

bool SensorGeiger::init() { return _driver != nullptr; }

bool SensorGeiger::read() {
  if (!_driver) return false;
  return _driver->read();
}

uint32_t SensorGeiger::getCPM() const {
  if (!_driver) return 0;
  return _driver->getTics();
}

float SensorGeiger::getMicroSievertHour() const {
  if (!_driver) return 0.0f;
  return _driver->getUSvh();
}

void SensorGeiger::clear() {
  if (_driver) _driver->clear();
}
