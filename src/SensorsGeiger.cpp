// CanAirIO sensorlib — SensorsGeiger.cpp
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include "Sensors.hpp"

#include <math.h>
#include <string.h>

void Sensors::geigerRead() {
  if (rad != nullptr && rad->read()) {
    unitRegister(UNIT::CPM);
    unitRegister(UNIT::RAD);
  }
}

/**
 * @brief Enable Geiger sensor on specific pin
 * @param gpio number or pin.
 */
void Sensors::enableGeigerSensor(int gpio) {
  sensorAnnounce(SENSORS::SCAJOE);
  if (gpio < 0) {
    if (devmode) Serial.printf("[W][SLIB] undefined Geiger pin\t: %i\r\n", gpio);
    return;
  }
  rad = new GEIGER(gpio, devmode);
  sensorRegister(SENSORS::SCAJOE);
}

/**
 * @brief get Geiger count. Tics in the last 60secs
 * @return CPM
 */
uint32_t Sensors::getGeigerCPM(void) const {
  if (rad == nullptr)
    return 0;
  else
    return rad->getTics();
}

/**
 * @brief get Geiger count in uSv/h units
 * @return CPM * J305 conversion factor
 */
float Sensors::getGeigerMicroSievertHour(void) const {
  if (rad == nullptr)
    return 0;
  else
    return rad->getUSvh();
}
