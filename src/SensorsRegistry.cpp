// CanAirIO sensorlib — SensorsRegistry.cpp
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include <math.h>
#include <string.h>

#include "Sensors.hpp"

// Units and sensors registers

#define X(unit, symbol, name) symbol,
const char *unit_symbol[] = {SENSOR_UNITS};
#undef X

#define X(unit, symbol, name) name,
char const *unit_name[] = {SENSOR_UNITS};
#undef X

uint8_t units_registered[UCOUNT];

#define X(utype, uname, umaintype) uname,
char const *sensors_device_names[] = {SENSORS_TYPES};
#undef X

#define X(utype, uname, umaintype) umaintype,
int sensors_device_types[] = {SENSORS_TYPES};
#undef X

uint8_t sensors_registered[SCOUNT];

/**
 * @brief Read and check the sensors status on initialization
 * @param sensor (mandatory) SENSORS enum value.
 * @return True if the sensor is registered, false otherwise.
 */
bool Sensors::isSensorRegistered(SENSORS sensor) const {
  for (u_int i = 0; i < SCOUNT; i++) {
    if (sensors_registered[i] == sensor) return true;
  }
  return false;
}

/**
 * @brief get the sensor name
 * @param sensor (mandatory) SENSORS enum value.
 * @return String with the sensor name.
 */
String Sensors::getSensorName(SENSORS sensor) const {
  if (sensor < 0 || sensor > SENSORS::SCOUNT) return "";
  return String(sensors_device_names[sensor]);
}

/**
 * @brief get the sensor group type
 * @param sensor (mandatory) SENSORS enum value.
 * @return Sensor group int with the sensor type.
 *
 * if the sensor is not in a group, return 0.
 * if the sensor is in a group, return 1 (PM), 2 (CO2), 3 (ENV).
 */
SensorGroup Sensors::getSensorGroup(SENSORS sensor) const {
  return (SensorGroup)sensors_device_types[sensor];
}

/**
 * @brief get the sensor registry for retrieve the sensor names
 * @return pointer to the sensor registry.
 *
 * See the <a href="https://bit.ly/3qVQYYy">Advanced Multivariable example</a>
 */
uint8_t *Sensors::getSensorsRegistered() { return sensors_registered; }

/**
 * @brief get the sensor unit status on the registry
 * @return True if the sensor unit is available, false otherwise.
 *
 * See the <a href="https://bit.ly/3qVQYYy">Advanced Multivariable example</a>
 */
bool Sensors::isUnitRegistered(UNIT unit) const {
  if (unit == UNIT::NUNIT) return false;
  for (u_int i = 0; i < UCOUNT; i++) {
    if (units_registered[i] == unit) return true;
  }
  return false;
}

/**
 * @brief get the sensor units registry for retrieve the unit name, unit type and symbol. See
 * getNextUnit()
 * @return pointer to the sensor units registry
 *
 * See the <a href="https://bit.ly/3qVQYYy">Advanced Multivariable example</a>
 */
uint8_t *Sensors::getUnitsRegistered() { return units_registered; }

/// get device sensors units detected count
uint8_t Sensors::getUnitsRegisteredCount() const { return units_registered_count; }

/**
 * @brief get the sensor unit name
 * @param unit (mandatory) UNIT enum value.
 * @return String with the unit name.
 */
String Sensors::getUnitName(UNIT unit) const {
  if (unit < 0 || unit > UCOUNT) return "";
  return String(unit_name[unit]);
}

/**
 * @brief get the sensor unit symbol
 * @param unit (mandatory) UNIT enum value.
 * @return String with the unit symbol.
 */
String Sensors::getUnitSymbol(UNIT unit) const { return String(unit_symbol[unit]); }

/**
 * @brief get the next sensor unit available
 * @return UNIT enum value.
 */
UNIT Sensors::getNextUnit() {
  for (u_int i = current_unit; i < UCOUNT; i++) {
    if (units_registered[i] != 0) {
      current_unit = i + 1;
      return (UNIT)units_registered[i];
    }
  }
  current_unit = 0;
  return (UNIT)0;
}

/**
 * @brief reset the sensor units registry.
 *
 * This function is useful to reset the units registry after a sensor unit is removed.
 * but it is **Not necessary** to call this function.
 */
void Sensors::resetUnitsRegister() {
  units_registered_count = 0;
  for (u_int i = 0; i < UCOUNT; i++) {
    units_registered[i] = 0;
  }
}

/**
 * @brief reset the sensor registry.
 *
 * This function is useful to reset the sensors registry after a sensor is removed.
 * It should be called before the initialization of the sensors but
 * it is **Not necessary** to call this function.
 */
void Sensors::resetSensorsRegister() {
  sensors_registered_count = 0;
  for (u_int i = 0; i < SCOUNT; i++) {
    sensors_registered[i] = 0;
  }
}

/**
 * @brief reset the next sensor unit counter.
 *
 * This function is useful to reset the counter to review the sensor units again.
 * but it is not necessary to call this function.
 */
void Sensors::resetNextUnit() { current_unit = 0; }

/**
 * @brief get the sensor unit value (float)
 * @param unit (mandatory) UNIT enum value.
 * @return float value of the each unit (RAW).
 *
 * Also you can use the specific primitive like getTemperature(),
 * getHumidity(), getGas(), getAltitude(), getPressure()
 */
float Sensors::getUnitValue(UNIT unit) {
  switch (unit) {
    case PM1:
      return pm1;
    case PM25:
      return pm25;
    case PM4:
      return pm4;
    case PM10:
      return pm10;
    case TEMP:
      return temp;
    case TEMPK:
      return temp + 273.15;
    case TEMPF:
      return temp * 1.8 + 32;
    case HUM:
      return humi;
    case CO2:
      return CO2Val;
    case CO2TEMP:
      return CO2temp;
    case CO2TEMPK:
      return CO2temp + 273.15;
    case CO2TEMPF:
      return CO2temp * 1.8 + 32;
    case CO2HUM:
      return CO2humi;
    case PRESS:
      return pres;
    case ALT:
      return alt;
    case GAS:
      return gas;
    case CPM:
      return getGeigerCPM();
    case RAD:
      return getGeigerMicroSievertHour();
    case NH3:
      return nh3;
    case CO:
      return co;
    case NO2:
      return no2;
    case O3:
      return o3;
    case NOISE:
      return noiseInstant;
    case NOISEPEAK:
      return noisePeakValue;
    case NOISEAVGLEGAL:
      return noiseAvgLegalValue;
    case NOISEL90:
      return noiseL90Value;
    case NOISELASMAX:
      return noiseLASmaxValue;
    case NOISELCPEAK:
      return noiseLCpeakValue;
    case NOISELD:
      return noiseLdValue;
    case NOISELE:
      return noiseLeValue;
    case NOISELN:
      return noiseLnValue;
    case NOISELDEN:
      return noiseLdenValue;
    case VOC:
      return voc;
    case NOX:
      return nox;
    case VOCI:
      return voci;
    case NOXI:
      return noxi;
    default:
      return 0.0;
  }
}

/**
 * @brief print the sensor units names available
 * @param debug optional boolean to set the debug mode flag.
 */
void Sensors::printUnitsRegistered(bool debug) {
  if (!debug) return;
  Serial.printf("-->[SLIB] sensors units count\t: %i (", units_registered_count);
  int i = 0;
  while (units_registered[i++] != 0) {
    Serial.print(unit_name[units_registered[i - 1]]);
    Serial.print(",");
  }
  Serial.println(")");
}

/**
 * @brief print the sensor names detected
 * @param debug optional boolean to set the debug mode flag.
 */
void Sensors::printSensorsRegistered(bool debug) {
  if (!debug) return;
  int i = 0;
  Serial.printf("-->[SLIB] sensors count  \t: %i (", sensors_registered_count);
  if (sensors_registered_count > 0 && sensors_registered[0] == SENSORS::Auto) {
    Serial.printf("%s,", sensors_device_names[sensors_registered[0]]);
    i = 1;
  }
  while (sensors_registered[i++] != 0) {
    Serial.printf("%s,", sensors_device_names[sensors_registered[i - 1]]);
  }
  Serial.println(")");
}

/// Print preview of the current variables detected by the sensors
void Sensors::printValues() {
  if (!devmode) return;
  Serial.print("-->[SLIB] sensors values  \t: ");
  for (u_int i = 0; i < UCOUNT; i++) {
    if (units_registered[i] != 0) {
      UNIT unit = (UNIT)units_registered[i];
      Serial.print(getUnitName(unit));
      Serial.print(":");
      bool isGasPpm = (unit == NH3 || unit == CO || unit == NO2 || unit == O3);
      Serial.printf(isGasPpm ? "%02.2f " : "%02.2f ", getUnitValue(unit));
    }
  }
  Serial.println();
}

/// utility to notify on the debug output a possible sensor.
void Sensors::sensorAnnounce(SENSORS sensor) {
  DEBUG("-->[SLIB] attempt enable sensor\t:", getSensorName(sensor).c_str());
}

/**
 * @brief register the sensor type.
 * @param receive SENSORS enum param.
 *
 * Each sensor should be registered and also its units. With that we will able to have
 * dynamic calls of the sensors and its units on the GUI or implementation.
 */
void Sensors::sensorRegister(SENSORS sensor) {
  if (isSensorRegistered(sensor) && sensor != SENSORS::Auto) {
    return;
  }
  Serial.printf("-->[SLIB] sensor registered\t: %s  \t:D\r\n", getSensorName(sensor).c_str());
  sensors_registered[sensors_registered_count++] = sensor;
}

/**
 * @brief register the unit type.
 * @param receive UNIT enum param.
 *
 * Each sensor unit should be registered. For temperature sensors
 * please use tempRegister() method.
 */
void Sensors::unitRegister(UNIT unit) {
  if (isUnitRegistered(unit)) return;
  if (unit == UNIT::NUNIT) return;
  units_registered[units_registered_count++] = unit;
}

/// reset all library variables (generic sensors units)
void Sensors::resetAllVariables() {
  pm1 = 0;
  pm25 = 0;
  pm10 = 0;
  CO2Val = 0;
  CO2humi = 0.0;
  CO2temp = 0.0;
  humi = 0.0;
  temp = 0.0;
  alt = 0.0;
  gas = 0.0;
  pres = 0.0;
  nh3 = 0.0;
  co = 0;
  no2 = 0.0;
  o3 = 0.0;
  voc = 0;
  nox = 0;
  voci = 0.0;
  noxi = 0.0;
  noiseInstant = 0.0;
  noisePeakValue = 0.0;
  noiseAvgLegalValue = 0.0;
  noiseL90Value = 0.0;
  noiseLASmaxValue = 0.0;
  noiseLCpeakValue = 0.0;
  noiseLdValue = 0.0;
  noiseLeValue = 0.0;
  noiseLnValue = 0.0;
  noiseLdenValue = 0.0;
  _geiger.clear();
}
