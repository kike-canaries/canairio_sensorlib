// CanAirIO sensorlib — CAJOE Geiger public facade.
#include "Sensors.hpp"

/**
 * @brief Enable Geiger sensor on specific pin
 * @param gpio number or pin.
 */
void Sensors::enableGeigerSensor(int gpio) { _geiger.enable(gpio); }

/**
 * @brief get Geiger count. Tics in the last 60secs
 * @return CPM
 */
uint32_t Sensors::getGeigerCPM(void) const { return _geiger.cpm(); }

/**
 * @brief get Geiger count in uSv/h units
 * @return CPM * J305 conversion factor
 */
float Sensors::getGeigerMicroSievertHour(void) const { return _geiger.microSievertHour(); }
