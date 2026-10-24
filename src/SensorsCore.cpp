// CanAirIO sensorlib — Sensors core: construction, lifecycle, init and getters.
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include "Sensors.hpp"

#include <math.h>
#include <string.h>

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
      _serial2(nullptr),
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
  noiseSensorService();
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

#ifdef AG_OPENAIR
  senseAirS8Read();
#endif

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

  noiseSensorCollect();

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

  s8_uart2 = false;
  _serial2 = nullptr;

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

#ifdef AG_OPENAIR
  pms_type = PMS_TYPE;
#endif

  if (!i2conly && !sensorSerialInit(pms_type, pms_rx, pms_tx)) {
    DEBUG("-->[SLIB] UART sensors detected\t:", "0");
  }

#ifdef AG_OPENAIR
  sensorAnnounce(SENSORS::SAIRS8);
  if (senseAirS8Init()) sensorRegister(SENSORS::SAIRS8);
#endif

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

  noiseSensorAutoDetect();

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

float Sensors::getVOC() const { return voc; }

float Sensors::getNOX() const { return nox; }

float Sensors::getVOCI() const { return voci; }

float Sensors::getNOXI() const { return noxi; }

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

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SENSORSHANDLER)
Sensors sensors;
#endif
