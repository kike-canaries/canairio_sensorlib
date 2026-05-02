#include "Sensors.hpp"

#include <math.h>

/** Default I2C clock (Hz) when many devices share the bus; override with -D
 * SLIB_I2C_CLOCK_HZ=400000 */
#ifndef SLIB_I2C_CLOCK_HZ
#define SLIB_I2C_CLOCK_HZ 100000
#endif

static void dfrGasBeginFailed(const char *gasName, uint8_t i2cAddr) {
  Serial.printf("[W][SLIB] DFRobot %s begin failed — I2C 0x%02X\r\n", gasName, i2cAddr);
}

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

/***********************************************************************************
 *  P U B L I C   M E T H O D S
 * *********************************************************************************/

Sensors::Sensors()
    : devmode(false),
      sample_time(10),
      toffset(0.0),
      altoffset(0.0),
      sealevel(1013.25),
      hpa(0.0),
      i2conly(false),
      cm1106(nullptr),
      s8(nullptr),
      pm1006(nullptr),
      rad(nullptr),
      pm5003t(nullptr),
      _serial(nullptr),
      dataReady(false),
      sensors_registered_count(0),
      units_registered_count(0),
      current_unit(0) {
  resetSensorsRegister();
  resetUnitsRegister();
}

Sensors::~Sensors() {
  if (cm1106) delete cm1106;
  if (s8) delete s8;
  if (pm1006) delete pm1006;
  if (pm5003t) delete pm5003t;
  if (rad) delete rad;
}

/**
 * @brief Main sensors loop.
 * All sensors are read here, please call it on main loop.
 */
void Sensors::loop() {
#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseSensorService();
#endif
  static uint32_t pmLoopTimeStamp = 0;  // timestamp for sensor loop check data
  if ((millis() - pmLoopTimeStamp >
       sample_time * (uint32_t)1000)) {  // sample time for each capture
    pmLoopTimeStamp = millis();
    readAllSensors();

    if (!dataReady) DEBUG("-->[SLIB] any data from sensors\t: ? check your wirings!");

    if (dataReady && (_onDataCb != nullptr)) {
      _onDataCb();  // if any sensor reached any data, dataReady is true.
    } else if (!dataReady && (_onErrorCb != nullptr))
      _onErrorCb("[W][SLIB] sensorslib error msg\t: No data from any sensor!");
  }

#ifdef DHT11_ENABLED
  dhtRead();  // DHT2x sensors need check fastest
#endif
}

void Sensors::printHumTemp() {
  Serial.printf("-->[SLIB] sensorlib \t\t: T:%02.1f, H:%02.1f\r\n", humi, temp);
}

/**
 * @brief Read all sensors but use only one time or use loop() instead.
 * All sensors are read here. Use it carefully, better use sensors.loop()
 */
bool Sensors::readAllSensors() {
  readAllComplete = false;
  if (!i2conly && dev_uart_type >= 0) {
    dataReady = pmSensorRead();
    DEBUG("-->[SLIB] UART data ready \t:", dataReady ? "true" : "false");
  }
  enableWire1();

  CO2scd30Read();
  GCJA5Read();
  CO2scd4xRead();
  if (!sps30Read()) {
    sen5xRead();
  }
  am2320Read();
  sht31Read();
  bme280Read();
  bmp280Read();
  bme680Read();
  aht10Read();
  DFRobotCORead();
  DFRobotNH3Read();
  DFRobotNO2Read();
  DFRobotO3Read();
  geigerRead();
  sgp41Read();

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseSensorCollect();
#endif

#ifdef DHT11_ENABLED
  dhtRead();
#endif

  disableWire1();

  printValues();
  printSensorsRegistered(devmode);
  printUnitsRegistered(devmode);

  readAllComplete = dataReady;
  return dataReady;
}

/**
 * @brief All sensors init.
 * @param pms_type (optional) UART PMS type, please see DEVICE_TYPE enum.
 * @param pms_rx (optional) UART PMS RX pin.
 * @param pms_tx (optional) UART PMS TX pin.
 */
void Sensors::init(u_int pms_type, int pms_rx, int pms_tx) {
  // cleanup previous UART sensors if any (in case of re-init)
  if (cm1106) {
    delete cm1106;
    cm1106 = nullptr;
  }
  if (s8) {
    delete s8;
    s8 = nullptr;
  }
  if (pm1006) {
    delete pm1006;
    pm1006 = nullptr;
  }
  if (pm5003t) {
    delete pm5003t;
    pm5003t = nullptr;
  }
  if (rad) {
    delete rad;
    rad = nullptr;
  }

// override with debug INFO level (>=3)
#ifdef CORE_DEBUG_LEVEL
  if (CORE_DEBUG_LEVEL >= 3) devmode = true;
#endif
  if (devmode) {
    Serial.printf("-->[SLIB] CanAirIO SensorsLib\t: v%sr%d\r\n", CSL_VERSION, CSL_REVISION);
    Serial.printf("-->[SLIB] sensorslib devmod\t: %s\r\n", devmode ? "true" : "false");
  }

  Serial.println("-->[SLIB] temperature offset\t: " + String(toffset));
  Serial.println("-->[SLIB] altitude offset   \t: " + String(altoffset));
  Serial.println("-->[SLIB] sea level pressure\t: " + String(sealevel) + " hPa");
  Serial.printf("-->[SLIB] only i2c sensors  \t: %s\r\n", i2conly ? "true" : "false");

  if (!i2conly && !sensorSerialInit(pms_type, pms_rx, pms_tx)) {
    DEBUG("-->[SLIB] UART sensors detected\t:", "0");
  }

  startI2C();
  CO2scd30Init();
  GCJA5Init();
  CO2scd4xInit();
  if (!sps30I2CInit()) {
    sen5xInit();
  }
  bmp280Init();
  bme280Init();
  bme680Init();
  am2320Init();
  sht31Init();
  aht10Init();
  DFRobotCOInit();
  DFRobotNH3Init();
  DFRobotNO2Init();
  DFRobotO3Init();
  sgp41Init();

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseSensorAutoDetect();
#endif

#ifdef DHT11_ENABLED
  dhtInit();
#endif

  printSensorsRegistered(true);
}

/// set loop time interval for each sensor sample
void Sensors::setSampleTime(int seconds) {
  sample_time = seconds;
  if (devmode) Serial.println("-->[SLIB] new sample time\t: " + String(seconds));
  if (isSensorRegistered(SENSORS::SSCD30)) {
    scd30.setMeasurementInterval(seconds);
    if (devmode) Serial.println("-->[SLIB] SCD30 interval time\t: " + String(seconds));
  }
}

/**
 * @brief set CO2 recalibration PPM value (400 to 2000)
 * @param ppmValue the ppm value to set, normally 400.
 *
 * This method is used to set the CO2 recalibration value, please use it only on outdoor conditions.
 * Please see the documentation of each sensor for more information.
 */
void Sensors::setCO2RecalibrationFactor(int ppmValue) {
  if (isSensorRegistered(SENSORS::SSCD30)) {
    Serial.println("-->[SLIB] SCD30 calibration to\t: " + String(ppmValue));
    scd30.forceRecalibrationWithReference(ppmValue);
  }
  if (isSensorRegistered(SENSORS::SCM1106)) {
    Serial.println("-->[SLIB] CM1106 calibration to\t: " + String(ppmValue));
    cm1106->start_calibration(ppmValue);
  }
  if (isSensorRegistered(SENSORS::SMHZ19)) {
    Serial.println("-->[SLIB] MH-Z19 calibration to\t: " + String(ppmValue));
    mhz19.calibrate();
  }
  if (isSensorRegistered(SENSORS::SAIRS8)) {
    Serial.println("-->[SLIB] SAIRS8 calibration to\t: " + String(ppmValue));
    if (s8->manual_calibration()) Serial.println("-->[SLIB] S8 calibration ready.");
  }
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    Serial.println("-->[SLIB] SCD4x calibration to\t: " + String(ppmValue));
    uint16_t frcCorrection = 0;
    uint16_t error = 0;
    scd4x.stopPeriodicMeasurement();
    delay(510);
    error = scd4x.performForcedRecalibration(ppmValue, frcCorrection);
    if (error) Serial.printf("-->[SLIB] SCD4X recalibration\t: error frc:%d\r\n", frcCorrection);
    delay(50);
    scd4x.startPeriodicMeasurement();
  }
}

/**
 * @brief set CO2 altitude offset (m)
 * @param altitude (m).
 *
 * This method is used to compensate the CO2 value with the altitude. Recommended on high
 * altitude.
 */
void Sensors::setCO2AltitudeOffset(float altitude) {
  this->altoffset = altitude;
  this->hpa = hpaCalculation(altitude);  // hPa hectopascal calculation based on altitude

  if (isSensorRegistered(SENSORS::SSCD30)) {
    setSCD30AltitudeOffset(altoffset);
  }
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    scd4x.stopPeriodicMeasurement();
    delay(510);
    scd4x.setSensorAltitude(altoffset);
    delay(100);
    scd4x.startPeriodicMeasurement();
  }
}

/**
 * @brief set the sea level pressure (hPa)
 * @param hpa (hPa).
 *
 * This method is used to set the sea level pressure for some sensors that need it.
 */
void Sensors::setSeaLevelPressure(float hpa) { sealevel = hpa; }

/// restart and re-init all sensors (not recommended)
void Sensors::restart() {
  _serial->flush();
  init();
  delay(100);
}

/**
 * @brief Get sensor data.
 * @param cb (mandatory) callback function to be called when data is ready.
 */
void Sensors::setOnDataCallBack(voidCbFn cb) { _onDataCb = cb; }

/**
 * @brief Optional callback for get the sensors errors
 * @param cb callback function to be called when any warning or error happens.
 */
void Sensors::setOnErrorCallBack(errorCbFn cb) { _onErrorCb = cb; }

/**
 * @brief Optional for increase the debug level
 * @param enable true to enable debug mode, false to disable debug mode.
 */
void Sensors::setDebugMode(bool enable) { devmode = enable; }

/// get the sensor status
bool Sensors::isDataReady() { return readAllComplete; }

/// get PM1.0 ug/m3 value
uint16_t Sensors::getPM1() const { return pm1; }

/// get PM2.5 ug/m3 value
uint16_t Sensors::getPM25() const { return pm25; }

/// get PM4 ug/m3 value
uint16_t Sensors::getPM4() const { return pm4; }

/// get PM10 ug/m3 value
uint16_t Sensors::getPM10() const { return pm10; }

/// get CO2 ppm value
uint16_t Sensors::getCO2() const { return CO2Val; }

/// get humidity % value of CO2 sensor device
float Sensors::getCO2humi() const { return CO2humi; }

/// get humidity % value of environment sensor
float Sensors::getHumidity() const { return humi; }

/**
 * @brief set the temperature type unit
 * @param tunit celciuse, kelvin or fahrenheit.
 */
void Sensors::setTemperatureUnit(TEMPUNIT tunit) {
  temp_unit = tunit;
  String tunit_symbol;
  switch (temp_unit) {
    case TEMPUNIT::CELSIUS:
      tunit_symbol = getUnitSymbol(TEMP);
      break;
    case TEMPUNIT::KELVIN:
      tunit_symbol = getUnitSymbol(TEMPK);
      break;
    case TEMPUNIT::FAHRENHEIT:
      tunit_symbol = getUnitSymbol(TEMPF);
      break;
    default:
      tunit_symbol = getUnitSymbol(TEMP);
  }
  Serial.printf("-->[SLIB] temperature unit\t: %s\r\n", tunit_symbol.c_str());
}

/// get temperature value from the CO2 sensor device
float Sensors::getCO2temp() const {
  switch (temp_unit) {
    case TEMPUNIT::CELSIUS:
      return CO2temp;
    case TEMPUNIT::KELVIN:
      return CO2temp + 273.15;
    case TEMPUNIT::FAHRENHEIT:
      return CO2temp * 1.8 + 32;
  }
  return CO2temp;
}

/// get temperature value from environment sensor
float Sensors::getTemperature() const {
  switch (temp_unit) {
    case TEMPUNIT::CELSIUS:
      return temp;
    case TEMPUNIT::KELVIN:
      return temp + 273.15;
    case TEMPUNIT::FAHRENHEIT:
      return temp * 1.8 + 32;
  }
  return temp;
}

/**
 * @brief Temperature unit register (auto)
 * @param isCO2temp temperature unit register for CO2 sensors.
 *
 * This method should register the right unit regarding setTemperatureUnit() method.
 */
void Sensors::tempRegister(bool isCO2temp) {
  switch (temp_unit) {
    case (TEMPUNIT::CELSIUS):
      if (isCO2temp)
        unitRegister(UNIT::CO2TEMP);
      else
        unitRegister(UNIT::TEMP);
      break;
    case (TEMPUNIT::KELVIN):
      if (isCO2temp)
        unitRegister(UNIT::CO2TEMPK);
      else
        unitRegister(UNIT::TEMPK);
      break;
    case (TEMPUNIT::FAHRENHEIT):
      if (isCO2temp)
        unitRegister(UNIT::CO2TEMPF);
      else
        unitRegister(UNIT::TEMPF);
      break;
  }
}

/**
 * @brief Initialize internal temperature offset to be used on startup
 *
 * Positive value for offset to be subtracetd to the temperature.
 * Mush be called before the initialization of the sensors.
 */
void Sensors::initTOffset(float offset) { toffset = offset; }

/**
 * @brief Get sensorlib actual internal temperature offset
 * @return float with the temperature offset.
 * Positive value for offset to be subtracetd to the temperature.
 */
float Sensors::getTOffset() const { return toffset; }

/**
 * @brief Set temperature offset for all temperature sensors
 *
 * Positive value for offset to be subtracetd to the temperature.
 */
void Sensors::setTempOffset(float offset) {
  toffset = offset;
  setSCD30TempOffset(toffset * 100);
  setSCD4xTempOffset(toffset);
  setsen5xTempOffset(toffset);
}

/**
 * @brief Get temperature offset for Sensirion sensors (from internal sensor in SCD4x and SCD30)
 * @return float with the temperature offset.
 * Positive value for offset to be subtracetd to the temperature.
 */
float Sensors::getTempOffset() const {
  float toffset = 0.0;
  if (isSensorRegistered(SENSORS::SSCD30)) {
    toffset = getSCD30TempOffset();
  }
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    toffset = getSCD4xTempOffset();
  }
  return toffset;
}

/// get Gas resistance value of BMP680 sensor
float Sensors::getGas() const { return gas; }

/// get Altitude value in meters
float Sensors::getAltitude() const { return alt; }

/// get Pressure value in hPa
float Sensors::getPressure() const { return pres; }

/// get NH3 value in ppm
float Sensors::getNH3() const { return nh3; }

/// get CO value in ppm
float Sensors::getCO() const { return co; }

/// get NO2 value in ppm
float Sensors::getNO2() const { return no2; }

/// get O3 value in ppm
float Sensors::getO3() const { return o3; }

#ifdef CSL_NOISE_SENSOR_SUPPORTED
float Sensors::getNoise() const { return noiseInstant; }

float Sensors::getNoiseAverage() const { return noiseAvgValue; }

float Sensors::getNoisePeak() const { return noisePeakValue; }

float Sensors::getNoiseMin() const { return noiseMinValue; }

float Sensors::getNoiseLegalAverage() const { return noiseAvgLegalValue; }

float Sensors::getNoiseLegalMaximum() const { return noiseAvgLegalMaxValue; }
float Sensors::getNoiseLd() const { return noiseLdValue; }
float Sensors::getNoiseLe() const { return noiseLeValue; }
float Sensors::getNoiseLn() const { return noiseLnValue; }
float Sensors::getNoiseLden() const { return noiseLdenValue; }
#endif

/**
 * @brief UART only: check if the UART sensor is registered
 * @return bool true if the UART sensor is registered, false otherwise.
 */
bool Sensors::isUARTSensorConfigured() const { return dev_uart_type >= 0; }

/**
 * @brief UART only: get the UART sensor type. See SENSORS enum. Also getDeviceName()
 * @return SENSORS enum value.
 */
int Sensors::getUARTDeviceTypeSelected() const { return dev_uart_type; }

/**
 * @brief Forced to enable I2C sensors only.
 * Recommended to use only if you are using a I2C sensor and improve the performance.
 */
void Sensors::detectI2COnly(bool enable) { i2conly = enable; }

/// returns the CanAirIO Sensorslib version
String Sensors::getLibraryVersion() const { return String(CSL_VERSION); }

/// return the current revision code number
int16_t Sensors::getLibraryRevision() const { return CSL_REVISION; }

/// get device sensors detected count
uint8_t Sensors::getSensorsRegisteredCount() const { return sensors_registered_count; }

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
    case NOISEAVG:
      return noiseAvgValue;
    case NOISEPEAK:
      return noisePeakValue;
    case NOISEMIN:
      return noiseMinValue;
    case NOISEAVGLEGAL:
      return noiseAvgLegalValue;
    case NOISEAVGLEGALMAX:
      return noiseAvgLegalMaxValue;
    case NOISELD:
      return noiseLdValue;
    case NOISELE:
      return noiseLeValue;
    case NOISELN:
      return noiseLnValue;
    case NOISELDEN:
      return noiseLdenValue;
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

/******************************************************************************
 *  S E N S O R   P R I V A T E   M E T H O D S
 ******************************************************************************/

/**
 *  @brief PMS sensor generic read. Supported: Honeywell & Plantower sensors
 *  @return true if header and sensor data is right.
 */
bool Sensors::pmGenericRead() {
  int length_buffer = 32;
  String txtMsg = hwSerialRead(length_buffer);
  if (txtMsg[0] == 66) {
    if (txtMsg[1] == 77) {
      DEBUG("-->[SLIB] UART PMGENERIC read!\t: :D");
      pm25 = txtMsg[6] * 256 + (char)(txtMsg[7]);
      pm10 = txtMsg[8] * 256 + (char)(txtMsg[9]);

      unitRegister(UNIT::PM25);
      unitRegister(UNIT::PM10);

      if (pm25 > 1000 && pm10 > 1000) {
        onSensorError("[E][SLIB] UART PMGENERIC error\t: out of range pm25 > 1000");
      } else
        return true;
    } else {
      onSensorError("[E][SLIB] UART PMGENERIC error\t: invalid header");
    }
  }
  return false;
}

/**
 *  @brief Panasonic GCJA5 particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */
bool Sensors::pmGCJA5Read() {
  int length_buffer = 32;
  String txtMsg = hwSerialRead(length_buffer);
  if (txtMsg[0] == 02) {
    DEBUG("-->[SLIB] UART GCJA5 read\t: done!");
    pm1 = txtMsg[2] * 256 + (char)(txtMsg[1]);
    pm25 = txtMsg[6] * 256 + (char)(txtMsg[5]);
    pm10 = txtMsg[10] * 256 + (char)(txtMsg[9]);

    unitRegister(UNIT::PM1);
    unitRegister(UNIT::PM25);
    unitRegister(UNIT::PM10);

    if (pm25 > 2000 && pm10 > 2000) {
      onSensorError("[W][SLIB] GCJA5 UART msg  \t: out of range pm25 > 2000");
    } else
      return true;
  } else {
    onSensorError("[W][SLIB] GCJA5 UART msg  \t: invalid header");
  }
  return false;
}

/**
 *  @brief Nova SDS011 particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */
bool Sensors::pmSDS011Read() {
  int length_buffer = 10;
  String txtMsg = hwSerialRead(length_buffer);
  if (txtMsg[0] == 170) {
    if (txtMsg[1] == 192) {
      DEBUG("-->[SLIB] SDS011 read \t\t: done!");
      pm25 = (txtMsg[3] * 256 + (char)(txtMsg[2])) / 10;
      pm10 = (txtMsg[5] * 256 + (char)(txtMsg[4])) / 10;

      unitRegister(UNIT::PM25);
      unitRegister(UNIT::PM10);

      if (pm25 > 1000 && pm10 > 1000) {
        onSensorError("[W][SLIB] SDS011 UART msg\t: out of range pm25 > 1000");
      } else
        return true;
    } else {
      onSensorError("[W][SLIB] SDS011 UART msg\t: invalid header");
    }
  }
  return false;
}

/**
 *  @brief IKEA Vindriktning particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */

bool Sensors::pm1006Read() {
  uint16_t pm2_5;
  if (pm1006->read_pm25(&pm2_5)) {
    pm25 = pm2_5;
    unitRegister(UNIT::PM25);
    return true;
  }
  return false;
}

/**
 *  @brief PMS5003T particulate meter, T&H, sensors read.
 *  @return true if header and sensor data is right.
 */

bool Sensors::pm5003TRead() {
  if (!isSensorRegistered(SENSORS::P5003T)) return false;
  pm5003t->handle();
  pm1 = pm5003t->getPm01Ae();
  pm25 = pm5003t->getPm25Ae();
  pm10 = pm5003t->getPm10Ae();
  temp = pm5003t->getTemperature() - toffset;
  humi = pm5003t->getRelativeHumidity();
  unitRegister(UNIT::PM1);
  unitRegister(UNIT::PM25);
  unitRegister(UNIT::PM10);
  unitRegister(UNIT::HUM);
  unitRegister(UNIT::TEMP);
  return true;
}

/**
 * @brief PMSensor Serial read to basic string
 * @param SENSOR_RETRY attempts before failure
 * @return String buffer.
 **/
String Sensors::hwSerialRead(unsigned int length_buffer) {
  unsigned int try_sensor_read = 0;
  String txtMsg = "";
  while (txtMsg.length() < length_buffer && try_sensor_read++ < SENSOR_RETRY) {
    while (_serial->available() > 0) {
      char inChar = _serial->read();
      txtMsg += inChar;
    }
  }
  if (try_sensor_read > SENSOR_RETRY) {
    DEBUG("-->[SLIB] UART detection msg\t: no data");
  }
  return txtMsg;
}

/**
 *  @brief Sensirion SPS30 particulate meter sensor read.
 *  @return true if reads success.
 */
bool Sensors::sps30Read() {
  if (!isSensorRegistered(SENSORS::SSPS30)) return false;
  uint8_t ret, error_cnt = 0;
  delay(35);  // Delay for synchronization

  do {
    ret = sps30.GetValues(&val);
    if (ret == SPS30_ERR_DATALENGTH) {
      if (error_cnt++ > 3) {
        DEBUG("[W][SLIB] SPS30 setup message \t: error on values\t: ", String(ret).c_str());
        return false;
      }
      delay(500);
    } else if (ret != SPS30_ERR_OK) {
      sps30ErrToMess((char *)"[W][SLIB] SPS30 setup message \t: error on values\t: ", ret);
      return false;
    }
  } while (ret != SPS30_ERR_OK);

  DEBUG("-->[SLIB] SPS30 read \t\t: done!");

  pm1 = round(val.MassPM1);
  pm25 = round(val.MassPM2);
  pm4 = round(val.MassPM4);
  pm10 = round(val.MassPM10);

  unitRegister(UNIT::PM1);
  unitRegister(UNIT::PM25);
  unitRegister(UNIT::PM4);
  unitRegister(UNIT::PM10);

  if (pm25 > 1000 && pm10 > 1000) {
    onSensorError("[W][SLIB] SPS30 setup message \t: out of range pm25 > 1000");
    return false;
  }

  dataReady = true;

  return true;
}

bool Sensors::CO2Mhz19Read() {
  CO2Val = mhz19.getCO2();                     // Request CO2 (as ppm)
  CO2temp = mhz19.getTemperature() - toffset;  // Request Temperature (as Celsius)
  if (CO2Val > 0) {
    if (altoffset != 0) CO2correctionAlt();
    dataReady = true;
    DEBUG("-->[SLIB] MHZ14-9 read  \t: done!");
    tempRegister(true);
    unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

bool Sensors::CO2CM1106Read() {
  CO2Val = cm1106->get_co2();
  if (CO2Val > 0) {
    dataReady = true;
    if (altoffset != 0) CO2correctionAlt();
    DEBUG("-->[SLIB] CM1106 read   \t: done!");
    unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

bool Sensors::senseAirS8Read() {
  CO2Val = s8->get_co2();  // Request CO2 (as ppm)
  if (CO2Val > 0) {
    if (altoffset != 0) CO2correctionAlt();
    dataReady = true;
    DEBUG("-->[SLIB] SENSEAIRS8 read   \t: done!");
    unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

/**
 * @brief read sensor data. Sensor selected.
 * @return true if data is loaded from sensor.
 */
bool Sensors::pmSensorRead() {
  switch (dev_uart_type) {
    case Auto:
      return pmGenericRead();
      break;

    case SGCJA5:
      return pmGCJA5Read();
      break;

    case SDS011:
      return pmSDS011Read();
      break;

    case IKEAVK:
      return pm1006Read();
      break;

    case SMHZ19:
      return CO2Mhz19Read();
      break;

    case SCM1106:
      return CO2CM1106Read();
      break;

    case SAIRS8:
      return senseAirS8Read();
      break;

    case P5003T:
      return pm5003TRead();
      break;

    default:
      return false;
      break;
  }
}

/******************************************************************************
 *  I 2 C   S E N S O R   R E A D   M E T H O D S
 ******************************************************************************/

void Sensors::am2320Read() {
  if (!isSensorRegistered(SENSORS::SAM232X)) return;
  if (!am2320.isConnected()) return;
  int status = am2320.read();
  if (status != AM232X_OK) return;
  float humi1 = am2320.getHumidity();
  float temp1 = am2320.getTemperature();
  if (!isnan(humi1)) humi = humi1;
  if (!isnan(temp1)) {
    temp = temp1 - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] AM2320 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::bme280Read() {
  if (!isSensorRegistered(SENSORS::SBME280)) return;
  float humi1 = bme280.readHumidity();
  float temp1 = bme280.readTemperature();
  if (isnan(humi1) || humi1 == 0 || isnan(temp1)) return;
  humi = humi1;
  temp = temp1 - toffset;
  pres = bme280.readPressure();
  alt = bme280.readAltitude(sealevel);
  dataReady = true;
  DEBUG("-->[SLIB] BME280 read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::HUM);
  unitRegister(UNIT::ALT);
}

void Sensors::bmp280Read() {
  if (!isSensorRegistered(SENSORS::SBMP280)) return;
  float temp1 = bmp280.readTemperature();
  float press1 = bmp280.readPressure();
  float alt1 = bmp280.readAltitude(sealevel);
  if (press1 == 0 || isnan(temp1) || isnan(alt1)) return;
  temp = temp1 - toffset;
  pres = press1 / 100;  // convert to hPa
  alt = alt1;
  dataReady = true;
  DEBUG("-->[SLIB] BMP280 read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::PRESS);
  unitRegister(UNIT::ALT);
}

void Sensors::bme680Read() {
  if (!isSensorRegistered(SENSORS::SBME680)) return;
  if (!bme680.performReading()) return;
  float temp1 = bme680.temperature;
  temp = temp1 - toffset;
  humi = bme680.humidity;
  pres = bme680.pressure / 100.0;
  gas = bme680.gas_resistance / 1000.0;
  alt = bme680.readAltitude(sealevel);
  dataReady = true;
  DEBUG("-->[SLIB] BME680 read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::HUM);
  unitRegister(UNIT::PRESS);
  unitRegister(UNIT::GAS);
  unitRegister(UNIT::ALT);
}

void Sensors::aht10Read() {
  if (!isSensorRegistered(SENSORS::SAHTXX)) return;
  float temp1 = aht10.readTemperature();
  if (temp1 != AHTXX_ERROR) {
    float humi1 = aht10.readHumidity();
    if (humi1 != AHTXX_ERROR) humi = humi1;
    temp = temp1 - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] AHT10 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::sht31Read() {
  if (!isSensorRegistered(SENSORS::SSHT31)) return;
  float humi1 = sht31.readHumidity();
  float temp1 = sht31.readTemperature();
  if (!isnan(humi1)) humi = humi1;
  if (!isnan(temp1)) {
    temp = temp1 - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] SHT31 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::CO2scd30Read() {
  if (!isSensorRegistered(SENSORS::SSCD30)) return;
  if (!scd30.dataReady() || !scd30.read()) return;
  uint16_t tCO2 = scd30.CO2;  // we need temp var, without it override CO2
  if (tCO2 > 0) {
    CO2Val = tCO2;
    CO2humi = scd30.relative_humidity;
    CO2temp = scd30.temperature;
    dataReady = true;
    DEBUG("-->[SLIB] SCD30 read\t\t: done!");
    tempRegister(true);
    unitRegister(UNIT::CO2);
    unitRegister(UNIT::CO2HUM);
  }
}

void Sensors::sgp41Read() {
  if (!isSensorRegistered(SENSORS::SSGP41)) return;

  uint16_t error;
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;

  if (conditioning_s > 0) {
    // During NOx conditioning (10s) SRAW NOx will remain 0
    error = sgp41.executeConditioning(defaultRh, defaultT, voc);
    conditioning_s--;
  } else {
    // Read Measurement
    error = sgp41.measureRawSignals(defaultRh, defaultT, voc, nox);
  }

  if (error) {
    Serial.print("Error trying to execute (): ");
    DEBUG("-->[SLIB] sgp41 measureRaw error\t:", String(error).c_str());
    return;
  } else {
    unitRegister(UNIT::VOC);
    unitRegister(UNIT::NOX);
  }
}

void Sensors::CO2scd4xRead() {
  if (!isSensorRegistered(SENSORS::SSCD4X)) return;
  uint16_t tCO2 = 0;
  float tCO2temp, tCO2humi = 0;
  uint16_t error = scd4x.readMeasurement(tCO2, tCO2temp, tCO2humi);
  if (error) return;
  CO2Val = tCO2;
  CO2humi = tCO2humi;
  CO2temp = tCO2temp;
  dataReady = true;
  DEBUG("-->[SLIB] SCD4x read\t\t: done!");
  tempRegister(true);
  unitRegister(UNIT::CO2);
  unitRegister(UNIT::CO2HUM);
}

void Sensors::sen5xRead() {
  if (!isSensorRegistered(SENSORS::SSEN5X)) return;
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;

  uint16_t error = sen5x.readMeasuredValues(
      massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
      massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex);

  if (error) {
    DEBUG("[E][SLIB] SEN5x read error!");
    return;
  }

  pm1 = (u_int16_t)massConcentrationPm1p0;
  pm25 = (u_int16_t)massConcentrationPm2p5;
  pm4 = (u_int16_t)massConcentrationPm4p0;
  pm10 = (u_int16_t)massConcentrationPm4p0;
  voci = vocIndex;
  noxi = noxIndex;
  temp = ambientTemperature - toffset;
  humi = ambientHumidity;
  dataReady = true;
  DEBUG("-->[SLIB] SEN5x read\t\t: done!");
  unitRegister(UNIT::PM1);
  unitRegister(UNIT::PM25);
  unitRegister(UNIT::PM4);
  unitRegister(UNIT::PM10);
  unitRegister(UNIT::TEMP);
  unitRegister(UNIT::HUM);
  unitRegister(UNIT::VOCI);
  unitRegister(UNIT::NOXI);
}

void Sensors::GCJA5Read() {
  if (dev_uart_type == SENSORS::SGCJA5) return;
  if (!isSensorRegistered(SENSORS::SGCJA5)) return;
  if (!pmGCJA5.isConnected()) return;
  uint16_t _pm1 = pmGCJA5.getPM1_0();
  uint16_t _pm25 = pmGCJA5.getPM2_5();
  uint16_t _pm10 = pmGCJA5.getPM10();
  if (_pm1 > 1000 || _pm25 > 1000 || _pm10 > 1000) return;
  pm1 = _pm1;
  pm25 = _pm25;
  pm10 = _pm10;
  dataReady = true;
  DEBUG("-->[SLIB] GCJA5 read\t\t: done!");
  unitRegister(UNIT::PM1);
  unitRegister(UNIT::PM25);
  unitRegister(UNIT::PM10);
}

void Sensors::DFRobotNH3Read() {
  if (!isSensorRegistered(SENSORS::SDFRNH3)) return;
  float rawPpm = dfrNH3.readGasConcentrationPPM();
  float dfrInternalTemp = dfrNH3.readTempC();
  bool hasExternalTempSensor = dfrHasExternalTempSensor();
  float compensationTemp = hasExternalTempSensor ? temp : (dfrInternalTemp - toffset);
  nh3 = dfrGasTempCompensation(rawPpm, compensationTemp, DFRobot_GAS::NH3);
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
  if (pres > 0.0) o3 = dfrGasPressCompensation(o3, pres);
  unitRegister(UNIT::O3);
  dataReady = true;
  if (!hasExternalTempSensor) {
    temp = dfrInternalTemp - toffset;
    tempRegister(false);
  }
}

#if (CSL_NOISE_SENSOR_SUPPORTED==1)
bool Sensors::noiseSensorAutoDetect() {
  if (noiseSensorEnabled) return true;
  if (noiseScanDone && (millis() - noiseLastScanMs < noiseScanRetryMs)) return false;
  noiseScanDone = true;
  noiseLastScanMs = millis();

  noiseSensorInitWire();
  if (noiseWire == nullptr) {
    DEBUG("-->[SLIB] Noise sensor detect:\t", "no i2c");
    return false;
  }

  for (uint8_t addr = MIN_I2C_ADDRESS; addr <= MAX_I2C_ADDRESS; addr++) {
    if (devmode && (addr == MIN_I2C_ADDRESS || (addr % 16 == 0)))
      Serial.printf("-->[SLIB] Scanning I2C addr: 0x%02X\r\n", addr);
    bool present = noiseSensorDevicePresent(*noiseWire, addr);
    if (devmode && addr == MIN_I2C_ADDRESS)
      Serial.printf("-->[SLIB] Probe 0x%02X: %s\r\n", addr, present ? "ACK" : "NACK");
    if (!present) continue;
    if (devmode) Serial.printf("-->[SLIB] Found device at: 0x%02X, reading identity...\r\n", addr);

    uint8_t status = 0xFF;
    if (!noiseSensorReadStatus(*noiseWire, addr, status)) {
      if (devmode) Serial.printf("-->[SLIB] Failed to read status at: 0x%02X\r\n", addr);
      continue;
    }
    if (status > 0x07) {
      if (devmode) Serial.printf("-->[SLIB] Wrong status at: 0x%02X (0x%02X)\r\n", addr, status);
      continue;
    }

    noiseSensorEnabled = true;
    noiseSensorAddress = addr;
    sensorAnnounce(SENSORS::SNOISE);
    sensorRegister(SENSORS::SNOISE);
    DEBUG("-->[SLIB] Noise sensor detect:\t", "ok");
    return true;
  }

  DEBUG("-->[SLIB] Noise sensor detect:\t", "not found");
  return false;
}

void Sensors::noiseSensorService() {
  if (noiseSensorEnabled || noiseScanDone) return;
  noiseSensorAutoDetect();
}

void Sensors::noiseSensorCollect() {
  if (!noiseSensorEnabled) return;

  if (noiseWire == nullptr || !noiseSensorEnabled) return;

  if (noiseSensorAddress == 0) return;
  if (!noiseSensorDevicePresent(*noiseWire, noiseSensorAddress)) {
    noiseSensorEnabled = false;
    noiseSensorAddress = 0;
    return;
  }
  if (!noiseSensorReadData(*noiseWire, noiseSensorAddress, noiseSensorData)) return;
  noiseInstant =
      noiseSensorData
          .noiseAvgDb;  // Use DB as instant? Or raw? I'll use noiseAvgDb for 'NOISE' unit usually
  noiseAvgValue = noiseSensorData.noiseAvgDb;
  noisePeakValue = noiseSensorData.noisePeakDb;
  noiseMinValue = noiseSensorData.noiseMinDb;
  noiseAvgLegalValue = noiseSensorData.noiseAvgLegalDb;        // Use DB version
  noiseAvgLegalMaxValue = noiseSensorData.noiseAvgLegalMaxDb;  // Use DB version

  // Ld/Le/Ln/Lden: sensor returns 0 when period has no samples (e.g. Ld/Le=0 at night).
  // Preserve last valid value so UI shows recent data instead of 0.
  static constexpr float NOISE_PERIOD_MIN_VALID_DB = 15.0f;  // below this, 0 = "no data"
  auto updateIfValid = [](float newVal, float &stored) {
    if (!isnan(newVal) && newVal >= NOISE_PERIOD_MIN_VALID_DB) {
      stored = newVal;
    }
  };
  updateIfValid(noiseSensorData.Ld, noiseLdValue);
  updateIfValid(noiseSensorData.Le, noiseLeValue);
  updateIfValid(noiseSensorData.Ln, noiseLnValue);
  updateIfValid(noiseSensorData.noiseLden, noiseLdenValue);

  unitRegister(UNIT::NOISE);
  dataReady = true;

  unitRegister(UNIT::NOISEAVG);
  unitRegister(UNIT::NOISEPEAK);
  unitRegister(UNIT::NOISEMIN);
  unitRegister(UNIT::NOISEAVGLEGAL);
  unitRegister(UNIT::NOISEAVGLEGALMAX);
  unitRegister(UNIT::NOISELD);
  unitRegister(UNIT::NOISELE);
  unitRegister(UNIT::NOISELN);
  unitRegister(UNIT::NOISELDEN);
}

bool Sensors::noiseSensorReadStatus(TwoWire &wire, uint8_t address, uint8_t &status) {
  int retries = 3;
  while (retries-- > 0) {
    wire.beginTransmission(address);
    wire.write(CMD_GET_STATUS);
    if (wire.endTransmission(true) == 0) {
      delay(10);
      uint8_t got = wire.requestFrom(address, (uint8_t)1);
      if (got == 1) {
        status = wire.read();
        return true;
      }
    }
    delay(100);
  }
  return false;
}

bool Sensors::noiseSensorReadData(TwoWire &wire, uint8_t address, SensorData &out) {
  int retries = 3;
  while (retries-- > 0) {
    wire.beginTransmission(address);
    wire.write(CMD_GET_DATA);
    if (wire.endTransmission(true) == 0) {
      delay(5);
      uint8_t buffer[sizeof(SensorData)] = {0};
      uint8_t got = wire.requestFrom(address, (uint8_t)sizeof(SensorData));
      if (got == sizeof(SensorData)) {
        wire.readBytes(buffer, sizeof(SensorData));
        memcpy(&out, buffer, sizeof(SensorData));
        return true;
      }
    }
    delay(100);
  }
  return false;
}

bool Sensors::noiseSensorDevicePresent(TwoWire &wire, uint8_t address) {
  wire.beginTransmission(address);
  return (wire.endTransmission() == 0);
}

void Sensors::noiseSensorInitWire() {
  if (noiseWireReady) return;
  noiseWire = &Wire;
  noiseWireReady = true;
}

#endif

#ifdef DHT11_ENABLED
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
/**
 * @deprecated Please don't use this sensor anymore
 */
bool Sensors::dhtIsReady(float *temperature, float *humidity) {
  static unsigned long measurement_timestamp = millis();
  if (millis() - measurement_timestamp > sample_time * (uint32_t)1000) {
    if (dht_sensor.measure(temperature, humidity) == true) {
      measurement_timestamp = millis();
      return (true);
    }
  }
  return (false);
}

/**
 * @deprecated Please don't use this sensor anymore
 */
void Sensors::dhtInit() {
  sensorAnnounce(SENSORS::SDHTX);
  dhtRead();
}

/**
 * @deprecated Please don't use this sensor anymore
 */
void Sensors::dhtRead() {
  if (dhtIsReady(&dhttemp, &dhthumi) != true) return;
  temp = dhttemp - toffset;
  humi = dhthumi;
  dataReady = true;
  sensorRegister(SENSORS::SDHTX);
  DEBUG("-->[SLIB] DHTXX read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::HUM);
}
#endif

void Sensors::onSensorError(const char *msg) {
  DEBUG(msg);
  if (_onErrorCb != nullptr) _onErrorCb(msg);
}

void Sensors::sps30ErrToMess(char *mess, uint8_t r) {
  char buf[80];
  sps30.GetErrDescription(r, buf, 80);
  DEBUG("[E][SLIB] SPS30 error msg\t:", buf);
}

void Sensors::sps30Errorloop(char *mess, uint8_t r) {
  if (r)
    sps30ErrToMess(mess, r);
  else
    DEBUG(mess);
}

/**
 * Particule meter sensor (PMS) init.
 *
 * Hardware serial init for multiple PM sensors, like
 * Honeywell, Plantower, Panasonic, Sensirion, etc.
 *
 * @param pms_type PMS type, please see DEVICE_TYPE enum.
 * @param pms_rx PMS RX pin.
 * @param pms_tx PMS TX pin.
 **/
bool Sensors::sensorSerialInit(u_int pms_type, int pms_rx, int pms_tx) {
  // set UART for autodetection sensors (Honeywell, Plantower)
  if (pms_type == SENSORS::Auto) {
    DEBUG("-->[SLIB] UART detecting type\t: Auto");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  }
  // set UART for custom sensors
  else if (pms_type == SENSORS::SGCJA5) {
    DEBUG("-->[SLIB] UART detecting type\t: GCJA5");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SSPS30) {
    DEBUG("-->[SLIB] UART detecting type\t: SSPS30");
    if (!serialInit(pms_type, 115200, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SDS011) {
    DEBUG("-->[SLIB] UART detecting type\t: SDS011");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SMHZ19) {
    DEBUG("-->[SLIB] UART detecting type\t: Mhz19");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SCM1106) {
    DEBUG("-->[SLIB] UART detecting type\t: CM1106");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SAIRS8) {
    DEBUG("-->[SLIB] UART detecting type\t: SENSEAIRS8");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::IKEAVK) {
    DEBUG("-->[SLIB] UART detecting type\t: SENSEAIRS8");
    if (!serialInit(pms_type, PM1006::BIT_RATE, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::P5003T) {
    DEBUG("-->[SLIB] UART detecting type\t: PMS5003T");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  }

  // starting auto detection loop
  int try_sensor_init = 0;
  while (!pmSensorAutoDetect(pms_type) && try_sensor_init++ < 2) {
  }

  // get device selected..
  if (dev_uart_type >= 0) {
    DEBUG("-->[SLIB] UART sensor detected \t:", getSensorName((SENSORS)dev_uart_type).c_str());
    sensorRegister((SENSORS)dev_uart_type);
    return true;
  }

  return false;
}
/**
 * @brief Generic PM sensor auto detection.
 *
 * In order UART config, this method looking up for
 * special header on Serial stream
 **/
bool Sensors::pmSensorAutoDetect(u_int pms_type) {
  delay(1000);  // sync serial

  if (pms_type == SENSORS::SSPS30) {
    if (sps30UARTInit()) {
      dev_uart_type = SENSORS::SSPS30;
      return true;
    }
  }

  if (pms_type == SENSORS::SDS011) {
    if (pmSDS011Read()) {
      dev_uart_type = SENSORS::SDS011;
      return true;
    }
  }

  if (pms_type == SENSORS::IKEAVK) {
    if (PM1006Init()) {
      dev_uart_type = SENSORS::IKEAVK;
      return true;
    }
  }

  if (pms_type == SENSORS::P5003T) {
    if (PM5003TInit()) {
      dev_uart_type = SENSORS::P5003T;
      return true;
    }
  }

  if (pms_type == SENSORS::SMHZ19) {
    if (CO2Mhz19Init()) {
      dev_uart_type = SENSORS::SMHZ19;
      return true;
    }
  }

  if (pms_type == SENSORS::SCM1106) {
    if (CO2CM1106Init()) {
      dev_uart_type = SENSORS::SCM1106;
      return true;
    }
  }

  if (pms_type == SENSORS::SAIRS8) {
    if (senseAirS8Init()) {
      dev_uart_type = SENSORS::SAIRS8;
      return true;
    }
  }

  if (pms_type <= SENSORS::SGCJA5) {
    if (pmGenericRead()) {
      dev_uart_type = SENSORS::Auto;
      return true;
    }
    delay(1000);  // sync serial
    if (pmGCJA5Read()) {
      dev_uart_type = SENSORS::SGCJA5;
      return true;
    }
  }

  return false;
}

bool Sensors::CO2Mhz19Init() {
  mhz19.begin(*_serial);
  mhz19.autoCalibration(false);
  delay(100);
  int co2 = mhz19.getCO2();
  if (co2 == 0) return false;
  sensorRegister(SENSORS::SMHZ19);
  return true;
}

bool Sensors::PM1006Init() {
  pm1006 = new PM1006(*_serial);
  sensorRegister(SENSORS::IKEAVK);
  return pm1006Read();
}

bool Sensors::PM5003TInit() {
  pm5003t = new PMS5003T(*_serial);
  if (!pm5003t->begin()) return false;
  sensorRegister(SENSORS::P5003T);
  return true;
}

bool Sensors::CO2CM1106Init() {
  DEBUG("-->[SLIB] try to enable sensor\t: CM1106..");
  cm1106 = new CM1106_UART(*_serial);

  // Check if CM1106 is available
  cm1106->get_software_version(cm1106sensor.softver);
  int len = strlen(cm1106sensor.softver);
  if (len > 0) {
    if (len >= 10 && !strncmp(cm1106sensor.softver + len - 5, "SL-NS", 5)) {
      DEBUG("-->[SLIB] CM1106 version detected :D\t: CM1106SL-NS");
    } else if (!strncmp(cm1106sensor.softver, "CM", 2)) {
      DEBUG("-->[SLIB] CM1106 version detected :D\t: CM1106");
    } else {
      DEBUG("-->[SLIB] CM1106 version detected :D\t: unknown");
    }
  } else {
    DEBUG("[E][SLIB] CM1106 not detected!");
    return false;
  }

  // Show sensor info
  cm1106->get_serial_number(cm1106sensor.sn);
  DEBUG("-->[SLIB] CM1106 Serial number\t:", cm1106sensor.sn);
  DEBUG("-->[SLIB] CM1106 Software version\t:", cm1106sensor.softver);

  // Setup ABC parameters
  DEBUG("-->[SLIB] CM1106 Setting ABC parameters...");
  cm1106->set_ABC(CM1106_ABC_OPEN, 7, 415);  // 7 days cycle, 415 ppm for base

  // Force mode continous B for CM1106SL-NS
  cm1106->set_working_status(1);

  // Getting ABC parameters
  if (cm1106->get_ABC(&abc)) {
    DEBUG("-->[SLIB] CM1106 ABC parameters:");
    if (abc.open_close == CM1106_ABC_OPEN) {
      DEBUG("-->[SLIB] CM1106 Auto calibration is enabled");
    } else if (abc.open_close == CM1106_ABC_CLOSE) {
      DEBUG("-->[SLIB] CM1106 Auto calibration is disabled");
    }
    DEBUG("-->[SLIB] CM1106 Calibration cycle\t:", String(abc.cycle).c_str());
    DEBUG("-->[SLIB] CM1106 Calibration baseline\t:", String(abc.base).c_str());
  }

  return true;
}

bool Sensors::senseAirS8Init() {
  s8 = new S8_UART(*_serial);
  // Check if S8 is available
  s8->get_firmware_version(s8sensor.firm_version);
  int len = strlen(s8sensor.firm_version);
  if (len == 0) {
    DEBUG("[E][SLIB] SENSEAIR S8 not detected!");
    return false;
  }
  // Show S8 sensor info

  Serial.println("-->[SLIB] UART sensor detected \t: SenseAir S8");
  if (devmode) {
    Serial.printf("-->[SLIB] S8 Software version\t: %s\r\n", s8sensor.firm_version);
    Serial.printf("-->[SLIB] S8 Sensor type\t: 0x%08x\r\n", s8->get_sensor_type_ID());
    Serial.printf("-->[SLIB] S8 Sensor ID\t: %08x\r\n", s8->get_sensor_ID());
    Serial.printf("-->[SLIB] S8 Memory ver\t: 0x%04x\r\n", s8->get_memory_map_version());
    Serial.printf("-->[SLIB] S8 ABC period\t: %d hours\r\n", s8->get_ABC_period());
  }
  DEBUG("-->[SLIB] S8 Disabling ABC period");
  s8->set_ABC_period(0);
  delay(100);
  if (devmode) Serial.printf("-->[SLIB] S8 ABC period\t: %d hours\r\n", s8->get_ABC_period());

  DEBUG("-->[SLIB] S8 ABC period \t: 180 hours");
  s8->set_ABC_period(180);
  delay(100);
  if (devmode) Serial.printf("-->[SLIB] S8 ABC period\t: %d hours\r\n", s8->get_ABC_period());

  s8->get_meter_status();
  s8->get_alarm_status();
  s8->get_output_status();
  s8->get_acknowledgement();

  return true;
}

bool Sensors::sps30UARTInit() {
  sensorAnnounce(SENSORS::SSPS30);
  // set driver debug level
  if (CORE_DEBUG_LEVEL > 0) {
    sps30.EnableDebugging(true);
  }
  // Begin communication channel (non-ESP32: use Stream* e.g. SoftwareSerial; ESP32: SENSOR_COMMS)
#if defined(ARDUINO_ARCH_ESP32)
  if (!sps30.begin(SENSOR_COMMS)) {
#else
  if (_serial == nullptr || !sps30.begin(*_serial)) {
#endif
    sps30Errorloop((char *)"[E][SLIB] UART SPS30 could not initialize communication channel.", 0);
    return false;
  }

  if (!sps30tests()) return false;

  // start measurement
  if (sps30.start() == true) {
    DEBUG("-->[SLIB] SPS30 Measurement OK");
    sensorRegister(SENSORS::SSPS30);
    return true;
  } else
    sps30Errorloop((char *)"[E][SLIB] UART SPS30 Could NOT start measurement", 0);

  return false;
}

bool Sensors::sps30I2CInit() {
  if (dev_uart_type == SENSORS::SSPS30) return false;
  sensorAnnounce(SENSORS::SSPS30);
  // set driver debug level
  // if (CORE_DEBUG_LEVEL > 0) sps30.EnableDebugging(true);
  // Begin communication channel;
  if (sps30.begin(&Wire) == false) {
    sps30Errorloop((char *)"[E][SLIB] I2C SPS30 could not set channel.", 0);
    return false;
  }

  if (!sps30tests()) return false;

  DEBUG("-->[SLIB] SPS30 Detected via\t: I2C");

  // start measurement
  if (sps30.start()) {
    DEBUG("-->[SLIB] SPS30 measurement \t: OK");
    if (sps30.I2C_expect() == 4)
      DEBUG("[W][SLIB] SPS30 setup message\t: I2C buffersize only PM values  \r\n");
    sensorRegister(SENSORS::SSPS30);
    return true;
  } else
    sps30Errorloop((char *)"[W][SLIB] I2C SPS30 message \t: Could NOT start measurement.", 0);

  return false;
}

bool Sensors::sps30tests() {
  // check for SPS30 connection
  if (!sps30.probe()) {
    sps30Errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not probe.", 0);
    return false;
  } else {
    sps30DeviceInfo();
  }
  // reset SPS30 connection
  if (!sps30.reset()) {
    sps30Errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not reset.", 0);
    return false;
  }
  return true;
}

/**
 * @brief : read and display Sensirion device info.
 */
void Sensors::sps30DeviceInfo() {
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  // try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK) {
    if (strlen(buf) > 0)
      DEBUG("-->[SLIB] SPS30 Serial number\t: ", buf);
    else
      DEBUG("[SLIB] SPS30 could not get serial number");
  } else
    DEBUG("[SLIB] SPS30 could not get serial number");

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK) {
    if (strlen(buf) > 0)
      DEBUG("-->[SLIB] SPS30 product name\t: ", buf);
    else
      DEBUG("[SLIB] SPS30 could not get product name.");
  } else
    DEBUG("[SLIB] SPS30 could not get product name.");

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK) {
    DEBUG("[SLIB] SPS30 can not read version info");
    return;
  }
  sprintf(buf, "%d.%d", v.major, v.minor);
  DEBUG("-->[SLIB] SPS30 firmware level\t: ", buf);

  if (SENSOR_COMMS != I2C_COMMS) {
    sprintf(buf, "%d.%d", v.SHDLC_major, v.SHDLC_minor);
    DEBUG("-->[SLIB] SPS30 Hardware level\t:", String(v.HW_version).c_str());
    DEBUG("-->[SLIB] SPS30 SHDLC protocol\t:", buf);
  }

  sprintf(buf, "%d.%d", v.DRV_major, v.DRV_minor);
  DEBUG("-->[SLIB] SPS30 Library level\t:", buf);
}

void Sensors::am2320Init() {
  sensorAnnounce(SENSORS::SAM232X);
#ifndef Wire1
  if (!am2320.begin()) return;
#else
  am2320 = AM232X(&Wire);
  if (!am2320.begin()) {
    am2320 = AM232X(&Wire1);
    if (!am2320.begin()) return;
  }
#endif
  am2320.wakeUp();
  sensorRegister(SENSORS::SAM232X);
}

void Sensors::sht31Init() {
  sensorAnnounce(SENSORS::SSHT31);
  sht31 = Adafruit_SHT31();
#ifndef Wire1
  if (!sht31.begin()) return;
#else
  if (!sht31.begin()) {
    sht31 = Adafruit_SHT31(&Wire1);
    if (!sht31.begin()) return;
  }
#endif
  sensorRegister(SENSORS::SSHT31);
}

void Sensors::bme280Init() {
  sensorAnnounce(SENSORS::SBME280);
#ifndef Wire1
  if (!bme280.begin() && !bme280.begin(BME280_ADDRESS_ALTERNATE)) return;
#else
  if (!bme280.begin() && !bme280.begin(BME280_ADDRESS_ALTERNATE) &&
      !bme280.begin(BME280_ADDRESS, &Wire1) && !bme280.begin(BME280_ADDRESS_ALTERNATE, &Wire1))
    return;
#endif
  sensorRegister(SENSORS::SBME280);
}

/// Environment BMP280 sensor init
void Sensors::bmp280Init() {
  sensorAnnounce(SENSORS::SBMP280);
#ifndef Wire1
  if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) return;
#else
  if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) {
    bmp280 = Adafruit_BMP280(&Wire1);
    if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) return;
  }
#endif
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Operating Mode.
                     Adafruit_BMP280::SAMPLING_X2,      // Temp. oversampling
                     Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                     Adafruit_BMP280::FILTER_X16,       // Filtering.
                     Adafruit_BMP280::STANDBY_MS_500);  // Standby time.
#if CORE_DEBUG_LEVEL >= 3
  Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();
  if (devmode) bmp_temp->printSensorDetails();
  if (devmode) bmp_pressure->printSensorDetails();
#endif
  sensorRegister(SENSORS::SBMP280);
}

/// Bosch BME680 sensor init
void Sensors::bme680Init() {
  sensorAnnounce(SENSORS::SBME680);
#ifndef Wire1
  if (!bme680.begin()) return;
#else
  if (bme680.begin() == false) {
    bme680 = Adafruit_BME680(&Wire1);
    if (!bme680.begin()) return;
  }
#endif
  bme680.setTemperatureOversampling(BME680_OS_8X);
  bme680.setHumidityOversampling(BME680_OS_2X);
  bme680.setPressureOversampling(BME680_OS_4X);
  bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme680.setGasHeater(320, 150);  // 320*C for 150 ms
  sensorRegister(SENSORS::SBME680);
}

/// AHTXX sensors init
void Sensors::aht10Init() {
  sensorAnnounce(SENSORS::SAHTXX);
  // TODO: this sensor only works in Wire0
  aht10 = AHTxx(AHTXX_ADDRESS_X38, AHT1x_SENSOR);
#ifdef M5STICKCPLUS  // issue: https://github.com/enjoyneering/AHTxx/issues/11
  if (!aht10.begin(EXT_I2C_SDA, EXT_I2C_SCL, 100000, 50000)) return;
#else
  if (!aht10.begin()) return;
#endif
  sensorRegister(SENSORS::SAHTXX);
}

/// Sensirion SCD30 CO2/T/H sensor init
void Sensors::CO2scd30Init() {
  sensorAnnounce(SENSORS::SSCD30);
#ifndef Wire1
  if (!scd30.begin()) return;
#else
  if (!scd30.begin() && !scd30.begin(SCD30_I2CADDR_DEFAULT, &Wire1, SCD30_CHIP_ID)) return;
#endif
  delay(10);

  sensorRegister(SENSORS::SSCD30);

  DEBUG("-->[SLIB] SCD30 Temp offset\t:", String(scd30.getTemperatureOffset()).c_str());
  DEBUG("-->[SLIB] SCD30 Altitude offset\t:", String(scd30.getAltitudeOffset()).c_str());

  if (scd30.getAltitudeOffset() != uint16_t(altoffset)) {
    DEBUG("-->[SLIB] SCD30 altitude offset to\t:", String(altoffset).c_str());
    setSCD30AltitudeOffset(altoffset);
    delay(10);
  }

  if (uint16_t((scd30.getTemperatureOffset())) != (uint16_t(toffset * 100))) {
    DEBUG("-->[SLIB] SCD30 Temp offset to\t:", String(toffset).c_str());
    setSCD30TempOffset(toffset);
    delay(10);
  }
}

/// set SCD30 temperature compensation
void Sensors::setSCD30TempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD30)) {
    Serial.println("-->[SLIB] SCD30 new temp offset\t: " + String(offset));
    scd30.setTemperatureOffset(offset);
  }
}

/// get SCD30 temperature compensation
float Sensors::getSCD30TempOffset() const {
  float offset = 0.0;
  if (isSensorRegistered(SENSORS::SSCD30)) {
    // Cast away const because Adafruit_SCD30::getTemperatureOffset() is not const
    offset = const_cast<Adafruit_SCD30 &>(scd30).getTemperatureOffset() / 100.0;
    Serial.println("-->[SLIB] SCD30 get temp offset\t: " + String(offset));
  }
  return offset;
}

/// set SCD30 altitude compensation
void Sensors::setSCD30AltitudeOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD30)) {
    Serial.println("-->[SLIB] SCD30 new altitude offset\t: " + String(offset));
    scd30.setAltitudeOffset(uint16_t(offset));
  }
}

void Sensors::sgp41Init() {
  sensorAnnounce(SENSORS::SSGP41);
  uint16_t error;
  uint16_t testResult;
  sgp41.begin(Wire);
  error = sgp41.executeSelfTest(testResult);
  if (error) {
    DEBUG("-->[SLIB] sgp41 selftest error\t:", String(error).c_str());
    return;
  } else if (testResult != 0xD400) {
    DEBUG("-->[SLIB] sgp41 selfTest error\t:", String(testResult).c_str());
    return;
  }
  sensorRegister(SENSORS::SSGP41);
}

/// Sensirion SCD4X CO2 sensor init
void Sensors::CO2scd4xInit() {
  sensorAnnounce(SENSORS::SSCD4X);
  float tTemperatureOffset, offsetDifference;
  uint16_t tSensorAltitude;
  uint16_t error;
  scd4x.begin(Wire);
  error = scd4x.stopPeriodicMeasurement();
  if (error) return;
  sensorRegister(SENSORS::SSCD4X);
  scd4x.getTemperatureOffset(tTemperatureOffset);
  scd4x.getSensorAltitude(tSensorAltitude);
  DEBUG("-->[SLIB] SCD4x Temp offset\t:", String(tTemperatureOffset).c_str());
  DEBUG("-->[SLIB] SCD4x Alt offset \t:", String(tSensorAltitude).c_str());

  if (tSensorAltitude != uint16_t(altoffset)) setSCD4xAltitudeOffset(altoffset);

  offsetDifference = abs((toffset * 100) - (tTemperatureOffset * 100));
  if (offsetDifference >
      0.5) {  // Accounts for SCD4x conversion rounding errors in temperature offset
    Serial.println("-->[SLIB] SCD4x new offset\t: Temp offset to " + String(toffset));
    setSCD4xTempOffset(toffset);
  }
  error = scd4x.startPeriodicMeasurement();
  if (error) DEBUG("[W][SLIB] SCD4x periodic measure\t: starting error:", String(error).c_str());
}

/// set SCD4x temperature compensation
void Sensors::setSCD4xTempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    Serial.println("-->[SLIB] SCD4x new temperature offset\t: " + String(offset));
    scd4x.stopPeriodicMeasurement();
    delay(510);
    scd4x.setTemperatureOffset(offset);
    scd4x.startPeriodicMeasurement();
  }
}

/// get SCD4x temperature compensation
float Sensors::getSCD4xTempOffset() const {
  float offset = 0.0;
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    // We cannot call stop/start measurements here if we want this method to be const
    // because they are not const methods in the library.
    // However, if we really need to read it, we might have to use a cached value
    // or cast away const if we are sure it is safe.
    // For now, let's try to just read it without stopping if the lib allows,
    // but the library says it must be stopped.
    // Since this is a refactoring, maybe we should just not make it const if it has side effects.
    // BUT the calling method getTempOffset() IS const.
    // Let's use a workaround for now: cast away const for the sub-calls.
    auto *nonConstThis = const_cast<Sensors *>(this);
    uint16_t error = nonConstThis->scd4x.stopPeriodicMeasurement();
    if (error) {
      DEBUG("[SLIB] SCD4x stopPeriodicMeasurement()\t: error:", String(error).c_str());
    } else {
      nonConstThis->scd4x.getTemperatureOffset(offset);
    }
    nonConstThis->scd4x.startPeriodicMeasurement();
  }
  return offset;
}

/// set SCD4x altitude compensation
void Sensors::setSCD4xAltitudeOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSCD4X)) {
    Serial.println("-->[SLIB] SCD4x new altitude offset\t: " + String(offset));
    scd4x.stopPeriodicMeasurement();
    delay(510);
    scd4x.setSensorAltitude(uint16_t(offset));
    scd4x.startPeriodicMeasurement();
  }
}

/// Panasonic SEN5X sensor init
void Sensors::sen5xInit() {
  sensorAnnounce(SENSORS::SSEN5X);
  sen5x.begin(Wire);
  uint16_t error;
  error = sen5x.deviceReset();
  if (error) return;
  float tempOffset = 0.0;
  sen5x.getTemperatureOffsetSimple(tempOffset);
  DEBUG("-->[SLIB] SEN5X Temp offset\t:", String(tempOffset).c_str());
  if (uint16_t((tempOffset * 100)) != (uint16_t(toffset * 100))) {
    sen5x.setTemperatureOffsetSimple(toffset);
    delay(10);
  }
  error = sen5x.startMeasurement();
  if (error) {
    DEBUG("[E][SLIB] Error trying to execute startMeasurement():");
    return;
  }
  sensorRegister(SENSORS::SSEN5X);
}

/// set SEN5X temperature compensation
void Sensors::setsen5xTempOffset(float offset) {
  if (isSensorRegistered(SENSORS::SSEN5X)) {
    Serial.println("-->[SLIB] SEN5x new temperature offset\t: " + String(offset));
    sen5x.stopMeasurement();
    sen5x.setTemperatureOffsetSimple(offset);
    delay(510);
    sen5x.startMeasurement();
  }
}

/// Panasonic GCJA5 sensor init
void Sensors::GCJA5Init() {
  sensorAnnounce(SENSORS::SGCJA5);
#ifndef Wire1
  if (!pmGCJA5.begin()) return;
#else
  if (pmGCJA5.begin() == false) {
    if (!pmGCJA5.begin(Wire1)) return;
  }
#endif
  sensorRegister(SENSORS::SGCJA5);
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

/// DFRobot GAS (CO) sensors init
void Sensors::DFRobotCOInit() {
  sensorAnnounce(SENSORS::SDFRCO);
  dfrCO = DFRobot_GAS_I2C(&Wire, DFROBOT_CO_I2C_ADDR);
  if (!dfrCO.begin()) {
    dfrGasBeginFailed("CO", DFROBOT_CO_I2C_ADDR);
    return;
  }
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrCO.changeAcquireMode(dfrCO.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  dfrCO.setTempCompensation(dfrCO.OFF);
  sensorRegister(SENSORS::SDFRCO);
}

/// DFRobot GAS (NH3) sensors init
void Sensors::DFRobotNH3Init() {
  sensorAnnounce(SENSORS::SDFRNH3);
  dfrNH3 = DFRobot_GAS_I2C(&Wire, DFROBOT_NH3_I2C_ADDR);
  if (!dfrNH3.begin()) {
    dfrGasBeginFailed("NH3", DFROBOT_NH3_I2C_ADDR);
    return;
  }
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrNH3.changeAcquireMode(dfrNH3.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  dfrNH3.setTempCompensation(dfrNH3.OFF);
  sensorRegister(SENSORS::SDFRNH3);
}

/// DFRobot GAS (NO2) sensors init
void Sensors::DFRobotNO2Init() {
  sensorAnnounce(SENSORS::SDFRNO2);
  dfrNO2 = DFRobot_GAS_I2C(&Wire, DFROBOT_NO2_I2C_ADDR);
  if (!dfrNO2.begin()) {
    dfrGasBeginFailed("NO2", DFROBOT_NO2_I2C_ADDR);
    return;
  }
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrNO2.changeAcquireMode(dfrNO2.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  dfrNO2.setTempCompensation(dfrNO2.OFF);
  sensorRegister(SENSORS::SDFRNO2);
}

/// DFRobot GAS (O3) sensors init
void Sensors::DFRobotO3Init() {
  sensorAnnounce(SENSORS::SDFRO3);
  dfrO3 = DFRobot_GAS_I2C(&Wire, DFROBOT_O3_I2C_ADDR);
  if (!dfrO3.begin()) {
    dfrGasBeginFailed("O3", DFROBOT_O3_I2C_ADDR);
    return;
  }
  // Mode of obtaining data: the main controller needs to request the sensor for data
  dfrO3.changeAcquireMode(dfrO3.PASSIVITY);
  delay(500);  // Required for PASSIVITY mode to stabilize (see DFRobot example)
  // Disable internal compensation: we apply our own using external T/P sensors
  dfrO3.setTempCompensation(dfrO3.OFF);
  sensorRegister(SENSORS::SDFRO3);
}

// Altitude compensation for CO2 sensors without Pressure atm or Altitude compensation
void Sensors::CO2correctionAlt() {
  DEBUG("-->[SLIB] CO2 altitude original\t:", String(CO2Val).c_str());
  float CO2cor = (0.016 * ((1013.25 - hpa) / 10) * (CO2Val - 400)) +
                 CO2Val;  // Increment of 1.6% for every hpa of difference at sea level
  CO2Val = round(CO2cor);
  DEBUG("-->[SLIB] CO2 compensated\t:", String(CO2Val).c_str());
}

/// hPa hectopascal calculation based on the altitude. See CO2AltitudeOffset setter
float Sensors::hpaCalculation(float altitude) {
  DEBUG("-->[SLIB] CO2 altitude offset\t:", String(altitude).c_str());
  float hpa =
      1012 - 0.118 * altitude +
      0.00000473 * altitude *
          altitude;  // Quadratic regression formula obtained PA (hpa) from high above the sea
  DEBUG("-->[SLIB] CO2 pressure (hPa)\t:", String(hpa).c_str());
  return hpa;
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
#ifdef CSL_NOISE_SENSOR_SUPPORTED
  noiseInstant = 0.0;
  noiseAvgValue = 0.0;
  noisePeakValue = 0.0;
  noiseMinValue = 0.0;
  noiseAvgLegalValue = 0.0;
  noiseAvgLegalMaxValue = 0.0;
  noiseLdValue = 0.0;
  noiseLeValue = 0.0;
  noiseLnValue = 0.0;
  noiseLdenValue = 0.0;
#endif
  if (rad != nullptr) rad->clear();
}

// #########################################################################

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

// #########################################################################

void Sensors::DEBUG(const char *text, const char *textb) const {
  if (devmode) {
    _debugPort.print(text);
    if (textb) {
      _debugPort.print(" ");
      _debugPort.print(textb);
    }
    _debugPort.println();
  }
}

//***********************************************************************************//

void Sensors::startI2C() {
#if defined(M5STICKCPLUS) || defined(M5COREINK)
  Wire.begin(EXT_I2C_SDA, EXT_I2C_SCL);  // M5CoreInk Ext port (default for all sensors)
  enableWire1();
#endif
#ifdef M5ATOM
  Wire.begin();
  enableWire1();
#endif
#if defined(SLIB_I2C_SDA) && defined(SLIB_I2C_SCL)
  Wire.begin(SLIB_I2C_SDA, SLIB_I2C_SCL);
  if (devmode)
    Serial.printf("-->[SLIB] I2C Wire started (custom) SDA:%d, SCL:%d\r\n", SLIB_I2C_SDA,
                  SLIB_I2C_SCL);
#elif defined(ESP32C3)
  Wire.begin(19, 18);
#elif defined(ESP32S2)
  Wire.begin(33, 35);
#elif defined(ESP32S3)
  Wire.begin();
  if (devmode) Serial.printf("-->[SLIB] I2C Wire started (S3) SDA:%d, SCL:%d\r\n", SDA, SCL);
#elif defined(ARDUINO_ARCH_ESP32)
  Wire.begin();
  if (devmode) Serial.printf("-->[SLIB] I2C Wire started (ESP32) SDA:%d, SCL:%d\r\n", SDA, SCL);
#elif defined(ARDUINO_ARCH_ESP8266)
  Wire.begin(SDA, SCL);
  if (devmode) Serial.printf("-->[SLIB] I2C Wire started (ESP8266) SDA:%d, SCL:%d\r\n", SDA, SCL);
#endif
#ifdef TTGO_T7S3
  Wire.begin(GROVE_SDA, GROVE_SCL);
  enableWire1();
#endif
#ifdef M5AIRQ
  Wire.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);
  enableWire1();
#endif
#ifdef AG_OPENAIR
  Wire.begin(AIRG_SDA, AIRG_SCL);
  delay(1000);
#endif
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  Wire.setClock(SLIB_I2C_CLOCK_HZ);
  if (devmode)
    Serial.printf("-->[SLIB] I2C clock set to %lu Hz\r\n", (unsigned long)SLIB_I2C_CLOCK_HZ);
#endif
}

void Sensors::enableWire1() {
#ifdef M5STICKCPLUS
  Wire1.flush();
  Wire1.begin(HAT_I2C_SDA, HAT_I2C_SCL);  // M5CoreInk hat pines (header on top)
#endif
#ifdef M5COREINK
  Wire1.flush();
  Wire1.begin(25, 26);  // M5CoreInk hat pines (header on top)
#endif
#ifdef M5ATOM
  Wire1.flush();
  Wire1.begin(26, 32);  // M5CoreInk Ext port (default for all sensors)
#endif
#ifdef M5AIRQ
  Wire1.flush();
  Wire1.begin(GROVE_SDA, GROVE_SCL);
#endif
#ifdef TTGO_T7S3
  Wire1.flush();
  Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);  // Alternative I2C port
#endif
}

void Sensors::disableWire1() {
#ifdef M5STICKCPLUS
  Wire1.flush();
  Wire1.begin(21, 22);  // Restore AXP192 I2C pins (failed after some time)
#endif
#ifdef M5COREINK
  Wire1.flush();
  Wire1.begin(21, 22);  // M5CoreInk hat pines (header on top)
#endif
}

bool Sensors::serialInit(u_int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx) {
  if (devmode)
    Serial.printf("-->[SLIB] UART init with speed\t: %lu TX:%i RX:%i\r\n", speed_baud, pms_tx,
                  pms_rx);
#if ARDUINO_USB_CDC_ON_BOOT  // Serial used for USB CDC
  Serial0.begin(9600, SERIAL_8N1);
  _serial = &Serial0;
  return true;
#endif
  switch (SENSOR_COMMS) {
    case SERIALPORT:
      Serial.begin(speed_baud);
      _serial = &Serial;
      break;
#if defined(ARDUINO_ARCH_ESP32)
    // on a Sparkfun ESP32 Thing the default pins for serial1 are used for acccessing flash memory
    // you have to define different pins upfront in order to use serial1 port.
    case SERIALPORT1:
      DEBUG("-->[SLIB] UART COMM port \t: Serial1");
      if (pms_rx == 0 || pms_tx == 0) {
        DEBUG("-->[SLIB] TX/RX line not defined");
        return false;
      }
      Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
      _serial = &Serial1;
      break;
    case SERIALPORT2:
#if SOC_UART_NUM > 2
      DEBUG("-->[SLIB] UART COMM port \t: Serial2");
      if (pms_type == SENSORS::SSPS30)
        Serial2.begin(speed_baud);
      else
        Serial2.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
      _serial = &Serial2;
#else
      DEBUG("-->[SLIB] Force UART port \t: Serial1");
      Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx);
      _serial = &Serial1;
#endif
      break;
#endif
    default:

      if (pms_rx == 0 || pms_tx == 0) {
        DEBUG("-->[SLIB] TX/RX line not defined");
        return false;
      }
      // In case RX and TX are both pin 8, try Serial1 anyway.
      // A way to force-enable Serial1 on some boards.
      if (pms_rx == 8 && pms_tx == 8) {
        Serial1.begin(speed_baud);
        _serial = &Serial1;
      }

      else {
#if defined(INCLUDE_SOFTWARE_SERIAL)
        DEBUG("-->[SLIB] swSerial init on pin\t:", String(pms_rx).c_str());
        static SoftwareSerial swSerial(pms_rx, pms_tx);
        if (pms_type == SENSORS::SSPS30)
          swSerial.begin(speed_baud);
        else if (pms_type == SENSORS::SGCJA5)
          swSerial.begin(speed_baud, SWSERIAL_8E1, pms_rx, pms_tx, false);
        else
          swSerial.begin(speed_baud, SWSERIAL_8N1, pms_rx, pms_tx, false);
        _serial = &swSerial;
#else
        DEBUG("-->[SLIB] UART SoftwareSerial \t: disable");
        return (false);
#endif  // INCLUDE_SOFTWARE_SERIAL
      }
      break;
  }

  delay(10);
  return true;
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SENSORSHANDLER)
Sensors sensors;
#endif
