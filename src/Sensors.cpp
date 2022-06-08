#include "Sensors.hpp"

DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

// Units and sensors registers

#define X(unit, symbol, name) symbol, 
const char *unit_symbol[] = { SENSOR_UNITS };
#undef X

#define X(unit, symbol, name) name,
char const *unit_name[] = { SENSOR_UNITS }; 
#undef X

uint8_t units_registered [UCOUNT];

#define X(utype, uname, umaintype) uname, 
char const *sensors_device_names[] = { SENSORS_TYPES };
#undef X

#define X(utype, uname, umaintype) umaintype,
int sensors_device_types[] = { SENSORS_TYPES };
#undef X

uint8_t sensors_registered [SCOUNT];

/***********************************************************************************
 *  P U B L I C   M E T H O D S
 * *********************************************************************************/

/**
 * @brief Main sensors loop.
 * All sensors are read here, please call it on main loop.
 */
void Sensors::loop() {
    static uint32_t pmLoopTimeStamp = 0;                 // timestamp for sensor loop check data
    if ((millis() - pmLoopTimeStamp > sample_time * (uint32_t)1000)) {  // sample time for each capture
        pmLoopTimeStamp = millis();
        readAllSensors();

        if (!dataReady) DEBUG("-->[SLIB] Any data from sensors\t: ? check your wirings!");

        if (dataReady && (_onDataCb != nullptr)) {
            _onDataCb();  // if any sensor reached any data, dataReady is true.
        } else if (!dataReady && (_onErrorCb != nullptr))
            _onErrorCb("[W][SLIB] Sensorslib error msg\t: No data from any sensor!"); 
    }

    dhtRead();  // DHT2x sensors need check fastest
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
    sps30Read();
    CO2scd4xRead();
    am2320Read();
    sht31Read();
    aht10Read();
    bme280Read();
    bmp280Read();
    bme680Read();
    dhtRead();
    disableWire1();
    geigerLoop();;
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
void Sensors::init(int pms_type, int pms_rx, int pms_tx) {
// override with debug INFO level (>=3)
#ifdef CORE_DEBUG_LEVEL
    if (CORE_DEBUG_LEVEL >= 3) devmode = true;
#endif
    if (devmode) {
        Serial.printf("-->[SLIB] CanAirIO SensorsLib\t: v%sr%d\n", CSL_VERSION, CSL_REVISION);
        Serial.printf("-->[SLIB] sensorslib devmod\t: %s\n", devmode ? "true" : "false");
    }
 
    Serial.println("-->[SLIB] temperature offset\t: " + String(toffset));
    Serial.println("-->[SLIB] altitude offset   \t: " + String(altoffset));
    Serial.println("-->[SLIB] sea level pressure\t: " + String(sealevel) + " hPa");
    Serial.printf("-->[SLIB] only i2c sensors  \t: %s\n", i2conly ? "true" : "false");

    if (!i2conly && !sensorSerialInit(pms_type, pms_rx, pms_tx)) {
        DEBUG("-->[SLIB] UART sensors detected\t:", "0");
    }
    startI2C();
    CO2scd30Init();
    sps30I2CInit();
    GCJA5Init();
    CO2scd4xInit();
    bme680Init();
    bmp280Init();
    bme280Init();
    am2320Init();
    sht31Init();
    aht10Init();
    dhtInit();
    geigerInit();
    printSensorsRegistered(true);
}

/// set loop time interval for each sensor sample
void Sensors::setSampleTime(int seconds) {
    sample_time = seconds;
    Serial.println("-->[SLIB] new sample time\t: " + String(seconds));
    if(isSensorRegistered(SENSORS::SSCD30)) {
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
        if (error) Serial.printf("-->[SLIB] SCD4X recalibration\t: error frc:%d\n",frcCorrection);
        delay(50);
        scd4x.startPeriodicMeasurement();
    }
}

/**
 * @brief set CO2 altitude offset (m)
 * @param altitude (m)
 * 
 * This method is used to compensate the CO2 value with the altitude. Recommended on high altitude.
 */
void Sensors::setCO2AltitudeOffset(float altitude){
    this->altoffset = altitude;
    this->hpa = hpaCalculation(altitude);       //hPa hectopascal calculation based on altitude

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
 * @param hpa (hPa)
 * 
 * This method is used to set the sea level pressure for some sensors that need it.
 */
void Sensors::setSeaLevelPressure(float hpa) {
    sealevel = hpa;
}

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
void Sensors::setOnDataCallBack(voidCbFn cb) {
    _onDataCb = cb;
}

/**
 * @brief Optional callback for get the sensors errors
 * @param cb callback function to be called when any warnning or error happens.
 */
void Sensors::setOnErrorCallBack(errorCbFn cb) {
    _onErrorCb = cb;
}

/**
 * @brief Optional for increase the debug level
 * @param enable true to enable debug mode, false to disable debug mode.
 */
void Sensors::setDebugMode(bool enable) {
    devmode = enable;
}

/// get the sensor status 
bool Sensors::isDataReady() {
    return readAllComplete;
}

/// get PM1.0 ug/m3 value
uint16_t Sensors::getPM1() {
    return pm1;
}

/// @deprecated get PM1.0 ug/m3 formated value
String Sensors::getStringPM1() {
    char output[5];
    sprintf(output, "%03d", getPM1());
    return String(output);
}

/// get PM2.5 ug/m3 value
uint16_t Sensors::getPM25() {
    return pm25;
}

/// @deprecated get PM2.5 ug/m3 formated value
String Sensors::getStringPM25() {
    char output[5];
    sprintf(output, "%03d", getPM25());
    return String(output);
}

/// get PM4 ug/m3 value
uint16_t Sensors::getPM4() {
    return pm4;
}

/// @deprecated get PM4 ug/m3 formated value
String Sensors::getStringPM4() {
    char output[5];
    sprintf(output, "%03d", getPM4());
    return String(output);
}

/// get PM10 ug/m3 value
uint16_t Sensors::getPM10() {
    return pm10;
}

/// @deprecated get PM10 ug/m3 formated value
String Sensors::getStringPM10() {
    char output[5];
    sprintf(output, "%03d", getPM10());
    return String(output);
}

/// get CO2 ppm value
uint16_t Sensors::getCO2() {
    return CO2Val;
}

/// @deprecated get CO2 ppm formated value
String Sensors::getStringCO2() {
    char output[5];
    sprintf(output, "%04d", getCO2());
    return String(output);
}

/// get humidity % value of CO2 sensor device
float Sensors::getCO2humi() {
    return CO2humi;
}

/// get temperature °C value of CO2 sensor device
float Sensors::getCO2temp() {
    return CO2temp;
}

/// get humidity % value of environment sensor 
float Sensors::getHumidity() {
    return humi;
}

/// get temperature °C value of environment sensor
float Sensors::getTemperature() {
    return temp;
}

/**
 * @brief Set temperature offset for all temperature sensors
 * @param offset temperature offset in °C (default 0). 
 * 
 * Positive value for offset to be subtracetd to the temperature.
 */
void Sensors::setTempOffset(float offset){
    toffset = offset;
    setSCD30TempOffset(toffset);
    setSCD4xTempOffset(toffset);
}

/// get Gas resistance value of BMP680 sensor
float Sensors::getGas() {
    return gas;
}

/// get Altitude value in meters
float Sensors::getAltitude() {
    return alt;
}

/// get Pressure value in hPa
float Sensors::getPressure() {
    return pres;
}

/**
 * @brief UART only: check if the UART sensor is registered
 * @return bool true if the UART sensor is registered, false otherwise.
 */
bool Sensors::isUARTSensorConfigured() {
    return dev_uart_type >= 0;
}

/**
 * @brief UART only: get the UART sensor type. See SENSORS enum. Also getDeviceName()
 * @return SENSORS enum value.
 */
int Sensors::getUARTDeviceTypeSelected() {
    return dev_uart_type;
}

/**
 * @brief Forced to enable I2C sensors only
 * Recommended to use only if you are using a I2C sensor and improve the performance.
 */
void Sensors::detectI2COnly(bool enable) {
    i2conly = enable;
}

String Sensors::getLibraryVersion() {
    return String(CSL_VERSION);
}

int16_t Sensors::getLibraryRevision() {
    return CSL_REVISION;
}

/// get device sensors detected count
uint8_t Sensors::getSensorsRegisteredCount() {
    return sensors_registered_count;
}

/**
 * @brief Read and check the sensors status on initialization
 * @param sensor (mandatory) SENSORS enum value.
 * @return True if the sensor is registered, false otherwise.
 */
bool Sensors::isSensorRegistered(SENSORS sensor) {
    for (int i = 0; i < SCOUNT; i++) {
        if (sensors_registered[i] == sensor) return true;
    }
    return false;
}

/**
 * @brief get the sensor name
 * @param sensor (mandatory) SENSORS enum value.
 * @return String with the sensor name.
 */
String Sensors::getSensorName(SENSORS sensor) {
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
SensorGroup Sensors::getSensorGroup(SENSORS sensor) {
    return (SensorGroup) sensors_device_types[sensor];
}

/**
 * @brief get the sensor registry for retrieve the sensor names
 * @return pointer to the sensor registry
 * @link https://bit.ly/3qVQYYy
 * 
 * See the multivariable example: https://bit.ly/2XzZ9yw
 */
uint8_t * Sensors::getSensorsRegistered() {
    return sensors_registered;
}

/**
 * @brief get the sensor unit status on the registry
 * @return True if the sensor unit is available, false otherwise.
 * @link https://bit.ly/3qVQYYy
 * 
 * See the multivariable example: https://bit.ly/2XzZ9yw
 */
bool Sensors::isUnitRegistered(UNIT unit) {
    if (unit == UNIT::NUNIT) return false;
    for (int i = 0; i < UCOUNT; i++) {
        if (units_registered[i] == unit) return true;
    }
    return false;
}

/**
 * @brief get the sensor units registry for retrieve the unit name, unit type and symbol. See getNextUnit()
 * @return pointer to the sensor units registry
 * @link https://bit.ly/3qVQYYy
 * 
 * See the multivariable example: https://bit.ly/2XzZ9yw
 */
uint8_t * Sensors::getUnitsRegistered() {
    return units_registered;
}

/// get device sensors units detected count
uint8_t Sensors::getUnitsRegisteredCount() {
    return units_registered_count;
}

/**
 * @brief get the sensor unit name
 * @param unit (mandatory) UNIT enum value.
 * @return String with the unit name.
 */
String Sensors::getUnitName(UNIT unit) {
    if (unit < 0 || unit > UCOUNT) return "";
    return String(unit_name[unit]);
}

/**
 * @brief get the sensor unit symbol
 * @param unit (mandatory) UNIT enum value.
 * @return String with the unit symbol.
 */
String Sensors::getUnitSymbol(UNIT unit) {
    return String(unit_symbol[unit]);
}

/**
 * @brief get the next sensor unit available
 * @return UNIT enum value.
 */
UNIT Sensors::getNextUnit() {
    for (int i = current_unit; i < UCOUNT; i++) {
        if (units_registered[i] != 0) {
            current_unit = i + 1;
            return (UNIT) units_registered[i];
        }
    }
    current_unit = 0;
    return (UNIT) 0;
}

/**
 * @brief reset the sensor units registry
 * 
 * This function is useful to reset the units registry after a sensor unit is removed.
 * but it is **Not necessary** to call this function.
 */
void Sensors::resetUnitsRegister() {
    units_registered_count = 0;
    for (int i = 0; i < UCOUNT; i++) {
        units_registered[i] = 0;
    }
}
/**
 * @brief reset the sensor registry
 * 
 * This function is useful to reset the sensors registry after a sensor is removed.
 * It should be called before the initialization of the sensors but
 * it is **Not necessary** to call this function.
 */
void Sensors::resetSensorsRegister() {
    sensors_registered_count = 0;
    for (int i = 0; i < SCOUNT; i++) {
        sensors_registered[i] = 0;
    }
}

/**
 * @brief reset the next sensor unit counter
 * 
 * This function is useful to reset the counter to review the sensor units again.
 * but it is not necessary to call this function.
 */
void Sensors::resetNextUnit() {
    current_unit = 0;
}

/**
 * @brief get the sensor unit value (float)
 * @param unit (mandatory) UNIT enum value.
 * @return float value of the each unit (RAW)
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
        case HUM:
            return humi;
        case CO2:
            return CO2Val;
        case CO2TEMP:
            return CO2temp;
        case CO2HUM:
            return CO2humi;
        case PRESS:
            return pres;
        case ALT:
            return alt;
        case GAS:
            return gas;
        default:
            return 0.0;
    }
}

/**
 * @brief print the sensor units names available
 * @param debug optional boolean to set the debug mode flag
 */
void Sensors::printUnitsRegistered(bool debug) { 
    if (!debug) return;
    Serial.printf("-->[SLIB] Sensors units count\t: %i (", units_registered_count);
    int i = 0;
    while (units_registered[i++] != 0) {
        Serial.print(unit_name[units_registered[i-1]]);
        Serial.print(",");
    }
    Serial.println(")");
}

/**
 * @brief print the sensor names detected
 * @param debug optional boolean to set the debug mode flag
 */
void Sensors::printSensorsRegistered(bool debug) { 
    if (!debug) return;
    Serial.printf("-->[SLIB] Sensors devices count\t: %i (", sensors_registered_count);
    int i = 0;
    while (sensors_registered[i++] != 0) {
        Serial.print(sensors_device_names[sensors_registered[i-1]]);
        Serial.print(",");
    }
    Serial.println(")");
}

/// Print preview of the current variables detected by the sensors
void Sensors::printValues() {
    if (!devmode) return;
    Serial.print("-->[SLIB] Preview sensors values\t: ");
    for (int i = 0; i < UCOUNT; i++) {
        if (units_registered[i] != 0) {
            Serial.print(getUnitName((UNIT)units_registered[i]));
            Serial.print(":");
            Serial.printf("%02.1f ", getUnitValue((UNIT)units_registered[i]));
        }
    }
    Serial.println();
}

/******************************************************************************
*  S E N S O R   P R I V A T E   M E T H O D S
******************************************************************************/

/**
 *  @brief PMS sensor generic read. Supported: Honeywell & Plantower sensors
 *  @return true if header and sensor data is right
 */
bool Sensors::pmGenericRead() {
    int lenght_buffer = 32;
    String txtMsg = hwSerialRead(lenght_buffer);
    if (txtMsg[0] == 66) {
        if (txtMsg[1] == 77) {
            DEBUG("-->[SLIB] UART PMGENERIC read \t\t: done!");
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
 *  @return true if header and sensor data is right
 */
bool Sensors::pmGCJA5Read() {
    int lenght_buffer = 32;
    String txtMsg = hwSerialRead(lenght_buffer);
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
 *  @return true if header and sensor data is right
 */
bool Sensors::pmSDS011Read() {
    int lenght_buffer = 10;
    String txtMsg = hwSerialRead(lenght_buffer);
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
 * @brief PMSensor Serial read to basic string
 * 
 * @param SENSOR_RETRY attempts before failure
 * @return String buffer
 **/
String Sensors::hwSerialRead(unsigned int lenght_buffer) {
    unsigned int try_sensor_read = 0;
    String txtMsg = "";
    while (txtMsg.length() < lenght_buffer && try_sensor_read++ < SENSOR_RETRY) {
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
 *  @return true if reads succes
 */
bool Sensors::sps30Read() {
    if (!isSensorRegistered(SENSORS::SSPS30)) return false;
    uint8_t ret, error_cnt = 0;
    delay(35);  //Delay for sincronization
    
    do {
        ret = sps30.GetValues(&val);
        if (ret == ERR_DATALENGTH) {
            if (error_cnt++ > 3) {
                DEBUG("[W][SLIB] SPS30 setup message \t: error on values\t: ", String(ret).c_str());
                return false;
            }
            delay(500);
        } else if (ret != ERR_OK) {
            sps30ErrToMess((char *)"[W][SLIB] SPS30 setup message \t: error on values\t: ", ret);
            return false;
        }
    } while (ret != ERR_OK);

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
    CO2Val = mhz19.getCO2();              // Request CO2 (as ppm)
    CO2temp = mhz19.getTemperature()-toffset;  // Request Temperature (as Celsius)
    if (CO2Val > 0) {
        if(altoffset != 0) CO2correctionAlt();
        dataReady = true;
        DEBUG("-->[SLIB] MHZ14-9 read  \t: done!");
        unitRegister(UNIT::CO2);
        unitRegister(UNIT::CO2TEMP);
        return true;
    }
    return false;
}

bool Sensors::CO2CM1106Read() {
    CO2Val = cm1106->get_co2();
    if (CO2Val > 0) {
        dataReady = true;
        if(altoffset != 0) CO2correctionAlt();
        DEBUG("-->[SLIB] CM1106 read   \t: done!");
        unitRegister(UNIT::CO2);
        return true;
    }
    return false;
}

bool Sensors::senseAirS8Read() {
    CO2Val = s8->get_co2();      // Request CO2 (as ppm)
    if (CO2Val > 0) {
        if(altoffset != 0) CO2correctionAlt();
        dataReady = true;
        DEBUG("-->[SLIB] SENSEAIRS8 read   \t: done!");
        unitRegister(UNIT::CO2);
        return true;
    }
    return false;
}

/**
 * @brief read sensor data. Sensor selected.
 * @return true if data is loaded from sensor
 */
bool Sensors::pmSensorRead() {
    switch (dev_uart_type) {
        case Auto:
            return pmGenericRead();
            break;

        case SGCJA5:
            return pmGCJA5Read();
            break;

        // case SSPS30:  CHECK: we don't need more this read here
        //     return sps30Read();
        //     break;

        case SDS011:
            return pmSDS011Read();
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

        default:
            return false;
            break;
    }
}

/******************************************************************************
*  I 2 C   S E N S O R   R E A D   M E T H O D S
******************************************************************************/

void Sensors::am2320Read() {
    if (!am2320.isConnected())return; 
    int status = am2320.read();
    if (status != AM232X_OK) return;
    float humi1 = am2320.getHumidity();
    float temp1 = am2320.getTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) {
        temp = temp1-toffset;
        dataReady = true;
        DEBUG("-->[SLIB] AM2320 read\t\t: done!");
        unitRegister(UNIT::TEMP);
        unitRegister(UNIT::HUM);
    }
}

void Sensors::bme280Read() {
    float humi1 = bme280.readHumidity();
    float temp1 = bme280.readTemperature();
    if (isnan(humi1) || humi1 == 0 || isnan(temp1)) return; 
    humi = humi1;
    temp = temp1-toffset;
    pres = bme280.readPressure();
    alt = bme280.readAltitude(sealevel);
    dataReady = true;
    DEBUG("-->[SLIB] BME280 read\t\t: done!");
    unitRegister(UNIT::TEMP);
    unitRegister(UNIT::HUM);
    unitRegister(UNIT::ALT);
}

void Sensors::bmp280Read() {
    float temp1 = bmp280.readTemperature();
    float press1 = bmp280.readPressure();
    float alt1 = bmp280.readAltitude(sealevel);
    if (press1 == 0 || isnan(temp1) || isnan(alt1)) return;
    temp = temp1-toffset;
    pres = press1/100; // convert to hPa
    alt = alt1;
    dataReady = true;
    DEBUG("-->[SLIB] BMP280 read\t\t: done!");
    unitRegister(UNIT::TEMP);
    unitRegister(UNIT::PRESS);
    unitRegister(UNIT::ALT);
}

void Sensors::bme680Read() {
    unsigned long endTime = bme680.beginReading();
    if (endTime == 0) return;
    if (!bme680.endReading()) return;
    float temp1 = bme680.temperature;
    temp = temp1 - toffset;
    humi = bme680.humidity;
    pres = bme680.pressure / 100.0;
    gas = bme680.gas_resistance / 1000.0;
    alt = bme680.readAltitude(sealevel);
    dataReady = true;
    DEBUG("-->[SLIB] BME680 read\t\t: done!");
    unitRegister(UNIT::TEMP);
    unitRegister(UNIT::HUM);
    unitRegister(UNIT::PRESS);
    unitRegister(UNIT::GAS);
    unitRegister(UNIT::ALT);
}

void Sensors::aht10Read() {
    float humi1 = aht10.readHumidity();
    float temp1 = aht10.readTemperature();
    if (humi1 != 255) humi = humi1;
    if (temp1 != 255) {
        temp = temp1-toffset;
        dataReady = true;
        DEBUG("-->[SLIB] AHT10 read\t\t: done!");
        unitRegister(UNIT::TEMP);
        unitRegister(UNIT::HUM);
    }
}

void Sensors::sht31Read() {
    float humi1 = sht31.readHumidity();
    float temp1 = sht31.readTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) { 
        temp = temp1-toffset;
        dataReady = true;
        DEBUG("-->[SLIB] SHT31 read\t\t: done!");
        unitRegister(UNIT::TEMP);
        unitRegister(UNIT::HUM);
    }
}

void Sensors::CO2scd30Read() {
    if (!scd30.dataReady() || !scd30.read()) return;
    uint16_t tCO2 = scd30.CO2;  // we need temp var, without it override CO2
    if (tCO2 > 0) {
        CO2Val = tCO2;
        CO2humi = scd30.relative_humidity;
        CO2temp = scd30.temperature;
        dataReady = true;
        DEBUG("-->[SLIB] SCD30 read\t\t: done!");
        unitRegister(UNIT::CO2);
        unitRegister(UNIT::CO2TEMP);
        unitRegister(UNIT::CO2HUM);
    }
}

void Sensors::CO2scd4xRead() {
    uint16_t tCO2 = 0;
    float tCO2temp, tCO2humi = 0;
    uint16_t error = scd4x.readMeasurement(tCO2, tCO2temp, tCO2humi);
    if (error) return;
    CO2Val = tCO2;
    CO2humi = tCO2humi;
    CO2temp = tCO2temp;
    dataReady = true;
    DEBUG("-->[SLIB] SCD4x read\t\t: done!");
    unitRegister(UNIT::CO2);
    unitRegister(UNIT::CO2TEMP);
    unitRegister(UNIT::CO2HUM);
}

void Sensors::GCJA5Read() {
    if (dev_uart_type == SENSORS::SGCJA5) return;
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

void Sensors::setDHTparameters(int dht_sensor_pin, int dht_sensor_type) {
    DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
}

void Sensors::dhtRead() {
    if (dhtIsReady(&dhttemp, &dhthumi) != true) return;
    temp = dhttemp - toffset;
    humi = dhthumi;
    dataReady = true;
    sensorRegister(SENSORS::SDHTX);
    DEBUG("-->[SLIB] DHTXX read\t\t: done!");
    unitRegister(UNIT::TEMP);
    unitRegister(UNIT::HUM);
}

void Sensors::onSensorError(const char *msg) {
    DEBUG(msg);
    if (_onErrorCb != nullptr ) _onErrorCb(msg);
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
bool Sensors::sensorSerialInit(int pms_type, int pms_rx, int pms_tx) {
    // set UART for autodetection sensors (Honeywell, Plantower)
    if (pms_type == Auto) {
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
    }

    // starting auto detection loop
    int try_sensor_init = 0;
    while (!pmSensorAutoDetect(pms_type) && try_sensor_init++ < 2);

    // get device selected..
    if (dev_uart_type >= 0) {
        DEBUG("-->[SLIB] UART sensor detected \t:", getSensorName((SENSORS)dev_uart_type).c_str());
        sensorRegister((SENSORS) dev_uart_type);
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
bool Sensors::pmSensorAutoDetect(int pms_type) {
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
    if (co2 == 0 ) return false;
    sensorRegister(SENSORS::SMHZ19);
    return true;
}

bool Sensors::CO2CM1106Init() {
    DEBUG("-->[SLIB] try to enable sensor\t: CM1106..");
    cm1106 = new CM1106_UART(*_serial);

    // Check if CM1106 is available
    cm1106->get_software_version(cm1106sensor.softver);
    int len = strlen(cm1106sensor.softver);
    if (len > 0) {
        if (len >= 10 && !strncmp(cm1106sensor.softver+len-5, "SL-NS", 5)) {
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
    cm1106->set_ABC(CM1106_ABC_OPEN, 7, 415);    // 7 days cycle, 415 ppm for base

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
        Serial.printf("-->[SLIB] S8 Software version\t: %s\n", s8sensor.firm_version);
        Serial.printf("-->[SLIB] S8 Sensor type\t: 0x%08x\n", s8->get_sensor_type_ID());
        Serial.printf("-->[SLIB] S8 Sensor ID\t: %08x\n", s8->get_sensor_ID());
        Serial.printf("-->[SLIB] S8 Memory ver\t: 0x%04x\n", s8->get_memory_map_version());
        Serial.printf("-->[SLIB] S8 ABC period\t: %d hours\n", s8->get_ABC_period());
    }
    DEBUG("-->[SLIB] S8 Disabling ABC period");
    s8->set_ABC_period(0);
    delay(100);
    if (devmode) Serial.printf("-->[SLIB] S8 ABC period\t: %d hours\n", s8->get_ABC_period());

    DEBUG("-->[SLIB] S8 ABC period \t: 180 hours");
    s8->set_ABC_period(180);
    delay(100);
    if (devmode) Serial.printf("-->[SLIB] S8 ABC period\t: %d hours\n", s8->get_ABC_period());

    s8->get_meter_status();
    s8->get_alarm_status();
    s8->get_output_status();
    s8->get_acknowledgement();

    return true;
}

bool Sensors::sps30UARTInit() {
    sensorAnnounce(SENSORS::SSPS30);
    // set driver debug level
    if (CORE_DEBUG_LEVEL > 0) sps30.EnableDebugging(true);
    // Begin communication channel;
    if (!sps30.begin(SENSOR_COMMS)) {
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
    if (CORE_DEBUG_LEVEL > 0) sps30.EnableDebugging(true);
    // Begin communication channel;
    if (sps30.begin(&Wire) == false) {
        sps30Errorloop((char *)"[E][SLIB] I2C SPS30 could not set channel.", 0);
        return false;
    }

    if (!sps30tests()) return false;

    DEBUG("-->[SLIB] SPS30 Detected SPS30 via I2C.");

    // start measurement
    if (sps30.start()) {
        DEBUG("-->[SLIB] SPS30 Measurement OK");
        if (sps30.I2C_expect() == 4) DEBUG("[W][SLIB] SPS30 setup message\t: I2C buffersize only PM values  \n");
        sensorRegister(SENSORS::SSPS30);
        return true;
    }
    else
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
 * @brief : read and display Sensirion device info
 */
void Sensors::sps30DeviceInfo() {
    char buf[32];
    uint8_t ret;
    SPS30_version v;

    //try to read serial number
    ret = sps30.GetSerialNumber(buf, 32);
    if (ret == ERR_OK) {
        if (strlen(buf) > 0)
            DEBUG("-->[SLIB] SPS30 Serial number\t: ", buf);
        else
            DEBUG("[SLIB] SPS30 could not get serial number");
    } else
        DEBUG("[SLIB] SPS30 could not get serial number");

    // try to get product name
    ret = sps30.GetProductName(buf, 32);
    if (ret == ERR_OK) {
        if (strlen(buf) > 0)
            DEBUG("-->[SLIB] SPS30 product name\t: ", buf);
        else
            DEBUG("[SLIB] SPS30 could not get product name.");
    } else
        DEBUG("[SLIB] SPS30 could not get product name.");

    // try to get version info
    ret = sps30.GetVersion(&v);
    if (ret != ERR_OK) {
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
    #ifdef ESP32
    am2320 = AM232X(&Wire);
    if (!am2320.begin()) {
        am2320 = AM232X(&Wire1);
        if (!am2320.begin()) return;
    }
    #else
    if (!am2320.begin()) return;
    #endif
    am2320.wakeUp();
    sensorRegister(SENSORS::SAM232X);
}

void Sensors::sht31Init() {
    sensorAnnounce(SENSORS::SSHT31);
    sht31 = Adafruit_SHT31();
    #ifdef ESP32
    if (!sht31.begin()) {
        sht31 = Adafruit_SHT31(&Wire1);
        if (!sht31.begin()) return; 
    }
    #else
    if (!sht31.begin()) return;
    #endif
    sensorRegister(SENSORS::SSHT31);
}

void Sensors::bme280Init() {
    sensorAnnounce(SENSORS::SBME280);
    #ifdef ESP32
    if (!bme280.begin() && !bme280.begin(BME280_ADDRESS,&Wire1)) return; 
    #else
    if (!bme280.begin()) return;
    #endif
    sensorRegister(SENSORS::SBME280);
}

void Sensors::bmp280Init() {
    sensorAnnounce(SENSORS::SBMP280);
    #ifdef ESP32
    if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) {
        bmp280 = Adafruit_BMP280(&Wire1);
        if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) return;
    }
    #else
    if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) return;
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

void Sensors::bme680Init() {
    sensorAnnounce(SENSORS::SBME680);
    if (!bme680.begin()) return;
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);  // 320*C for 150 ms
    sensorRegister(SENSORS::SBME680);
}

void Sensors::aht10Init() {
    sensorAnnounce(SENSORS::SAHT10);
    aht10 = AHT10(AHT10_ADDRESS_0X38);
    if (!aht10.begin()) return; 
    sensorRegister(SENSORS::SAHT10);
}

void Sensors::CO2scd30Init() {
    sensorAnnounce(SENSORS::SSCD30);
    #ifdef ESP32
    if (!scd30.begin() && !scd30.begin(SCD30_I2CADDR_DEFAULT, &Wire1, SCD30_CHIP_ID)) return;
    #else
    if (!scd30.begin()) return;
    #endif
    delay(10);

    DEBUG("-->[SLIB] SCD30 Temp offset\t:",String(scd30.getTemperatureOffset()).c_str());
    DEBUG("-->[SLIB] SCD30 Altitude offset\t:", String(scd30.getAltitudeOffset()).c_str());

    if(scd30.getAltitudeOffset() != uint16_t(altoffset)){
        DEBUG("-->[SLIB] SCD30 altitude offset to\t:", String(altoffset).c_str());
        setSCD30AltitudeOffset(altoffset);
        delay(10);
    }

    if(uint16_t((scd30.getTemperatureOffset()*100)) != (uint16_t(toffset*100))) {
        setSCD30TempOffset(toffset);
        delay(10);
    }
    sensorRegister(SENSORS::SSCD30);
}

/// set SCD30 temperature compensation
void Sensors::setSCD30TempOffset(float offset) {
    if (isSensorRegistered(SENSORS::SSCD30)) {
        Serial.println("-->[SLIB] SCD30 new temp offset\t: " + String(offset));
        scd30.setTemperatureOffset(offset);
    }
}

/// set SCD30 altitude compensation
void Sensors::setSCD30AltitudeOffset(float offset) {
    if (isSensorRegistered(SENSORS::SSCD30)) {
        Serial.println("-->[SLIB] SCD30 new altitude offset\t: " + String(offset));
        scd30.setAltitudeOffset(uint16_t(offset));
    }
}

void Sensors::CO2scd4xInit() {
    sensorAnnounce(SENSORS::SSCD4X);
    float tTemperatureOffset, offsetDifference;
    uint16_t tSensorAltitude;
    uint16_t error;
    scd4x.begin(Wire);
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        DEBUG("[W][SLIB] SCD4x stopping error \t:", String(error).c_str());
        return;
    }
    scd4x.getTemperatureOffset(tTemperatureOffset);
    scd4x.getSensorAltitude(tSensorAltitude);
    DEBUG("-->[SLIB] SCD4x Temp offset\t:", String(tTemperatureOffset).c_str());
    DEBUG("-->[SLIB] SCD4x Alt offset \t:", String(tSensorAltitude).c_str());

    if (tSensorAltitude != uint16_t(altoffset)) setSCD4xAltitudeOffset(altoffset);

    offsetDifference = abs((toffset*100) - (tTemperatureOffset*100)); 
    if(offsetDifference > 0.5) { // Accounts for SCD4x conversion rounding errors in temperature offset
        Serial.println("-->[SLIB] SCD4x new offset\t: Temp offset to" + String(toffset));
        setSCD4xTempOffset(toffset);
    }
    error = scd4x.startPeriodicMeasurement();
    if (error) DEBUG("[W][SLIB] SCD4x periodic measure\t: starting error:", String(error).c_str());
    sensorRegister(SENSORS::SSCD4X);
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

void Sensors::GCJA5Init() {
    sensorAnnounce(SENSORS::SGCJA5);
    #ifdef ESP32
    if (!pmGCJA5.begin() && !pmGCJA5.begin(Wire1)) return;
    #else
    if (!pmGCJA5.begin()) return;
    #endif
    sensorRegister(SENSORS::SGCJA5);
}

void Sensors::dhtInit() {
    sensorAnnounce(SENSORS::SDHTX);
    dhtRead();
}

// Altitude compensation for CO2 sensors without Pressure atm or Altitude compensation

void Sensors::CO2correctionAlt() {
    DEBUG("-->[SLIB] CO2 altitud original\t:", String(CO2Val).c_str());
    float CO2cor = (0.016 * ((1013.25 - hpa) /10 ) * (CO2Val - 400)) + CO2Val;       // Increment of 1.6% for every hpa of difference at sea level
    CO2Val = round (CO2cor);
    DEBUG("-->[SLIB] CO2 compensated\t:", String(CO2Val).c_str());
}

float Sensors::hpaCalculation(float altitude) {
    DEBUG("-->[SLIB] Altitude Compensation for CO2 lectures ON\t:", String(altitude).c_str());
    float hpa = 1012 - 0.118 * altitude + 0.00000473 * altitude * altitude;            // Cuadratic regresion formula obtained PA (hpa) from high above the sea
    DEBUG("-->[SLIB] Atmospheric pressure calculated in hPa\t:", String(hpa).c_str());
    return hpa;
}

void Sensors::sensorAnnounce(SENSORS sensor) {
    DEBUG("-->[SLIB] attempt enable sensor\t:",getSensorName(sensor).c_str());
}

void Sensors::sensorRegister(SENSORS sensor) {
    if (isSensorRegistered(sensor)) return;
    Serial.printf("-->[SLIB] sensor registered\t: %s  \t:D\n", getSensorName(sensor).c_str());
    sensors_registered[sensors_registered_count++] = sensor;
}

void Sensors::unitRegister(UNIT unit) {
    if (isUnitRegistered(unit)) return;
    if (unit == UNIT::NUNIT) return;
    units_registered[units_registered_count++] = unit;
}

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
}

void Sensors::DEBUG(const char *text, const char *textb) {
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
    Wire.begin(32,33);   // M5CoreInk Ext port (default for all sensors)
    enableWire1();
#endif
#ifdef M5ATOM
    enableWire1();
#endif
#if not defined(M5STICKCPLUS) && not defined(M5COREINK) && not defined(M5ATOM)
    Wire.begin();
#endif
}

void Sensors::enableWire1() {
#ifdef M5STICKCPLUS
    Wire1.flush();
    Wire1.begin(0,26);   // M5CoreInk hat pines (header on top)
#endif
#ifdef M5COREINK
    Wire1.flush();
    Wire1.begin(25,26);   // M5CoreInk hat pines (header on top)
#endif
#ifdef M5ATOM
    Wire1.flush();
    Wire1.begin(26,32,100000);   // M5CoreInk Ext port (default for all sensors)
#endif
}

void Sensors::disableWire1() {
#ifdef M5STICKCPLUS
    Wire1.flush();
    Wire1.begin(21,22); // Restore AXP192 I2C pins (failed after some time)
#endif
#ifdef M5COREINK
    Wire1.flush();
    Wire1.begin(21,22);   // M5CoreInk hat pines (header on top)
#endif
}

bool Sensors::serialInit(int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx) {
    if(devmode)Serial.printf("-->[SLIB] UART init with speed\t: %lu RX:%i TX:%i\n", speed_baud, pms_rx, pms_tx);
    switch (SENSOR_COMMS) {
        case SERIALPORT:
            Serial.begin(speed_baud);
            _serial = &Serial;
            break;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(SAMD21G18A) || defined(ARDUINO_SAM_DUE)
        case SERIALPORT1:
            Serial1.begin(speed_baud);
            _serial = &Serial1;
            break;
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(ARDUINO_SAM_DUE)
        case SERIALPORT2:
            Serial2.begin(speed_baud);
            _serial = &Serial2;
            break;

        case SERIALPORT3:
            Serial3.begin(speed_baud);
            _serial = &Serial3;
            break;
#endif
#if defined(__AVR_ATmega32U4__)
        case SERIALPORT1:
            Serial1.begin(speed_baud);
            _serial = &Serial1;
            break;
#endif

#if defined(ARDUINO_ARCH_ESP32)
        //on a Sparkfun ESP32 Thing the default pins for serial1 are used for acccessing flash memory
        //you have to define different pins upfront in order to use serial1 port.
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
            DEBUG("-->[SLIB] UART COMM port \t: Serial2");
            if (pms_type == SENSORS::SSPS30)
                Serial2.begin(speed_baud);
            else
                Serial2.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
            _serial = &Serial2;
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
                DEBUG("-->[SLIB] SoftWareSerial not enabled");
                return (false);
#endif  //INCLUDE_SOFTWARE_SERIAL
            }
            break;
    }

    delay(10);
    return true;
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SENSORSHANDLER)
Sensors sensors;
#endif

// *************** GEIGER ****************
// ***************************************
#ifdef ESP8266
    static volatile unsigned long counts = 0;
    static int secondcounts[60];
    static unsigned long int secidx_prev = 0;
    static unsigned long int count_prev = 0;
    static unsigned long int second_prev = 0; 
#else

// variables shared between main code and interrupt code
hw_timer_t * timer = NULL;
#endif
volatile uint32_t updateTime = 0;       // time for next update
volatile uint16_t tic_cnt = 0;
 
// To calculate a running average over 10 sec, we keep tic counts in 250ms intervals and add all 40 tic_buf values
#define TICBUFSIZE 40                   // running average buffer size
volatile uint16_t tic_buf[TICBUFSIZE];  // buffer for 40 times 250ms tick values (to have a running average for 10 seconds)
volatile uint16_t tic_buf_cnt = 0;
 
// In order to display a history of the last 7 minutes, we keep the last 50 values of 10sec tics
#define SEC10BUFSIZE 50                 // history array for displaymode==true
volatile uint16_t sec10 = 0;            // every 10 seconds counter
volatile uint16_t sec10_buf[SEC10BUFSIZE];  // buffer to hold 10 sec history (40*10 = 400 seconds)
volatile bool sec10updated = false;     // set to true when sec10_buf is updated
 
 #ifdef ESP8266
   // interrupt routine
ICACHE_RAM_ATTR static void TicISR()
{
    counts++;
}
#else
// #########################################################################
// Interrupt routine called on each click from the geiger tube
//
void IRAM_ATTR TicISR() {
  tic_cnt++;
}
 #endif

// #########################################################################
// Interrupt timer routine called every 250 ms
//
void IRAM_ATTR onTimer() {
  tic_buf[tic_buf_cnt++] = tic_cnt;
  tic_cnt = 0;
  if (tic_buf_cnt>=TICBUFSIZE) {
    uint16_t tot = 0;
    for (int i=0; i<TICBUFSIZE; i++) tot += tic_buf[i];
    sec10_buf[sec10++] = tot;
    tic_buf_cnt = 0;    
    if (sec10>=SEC10BUFSIZE) sec10 = 0;
    sec10updated = true;
  }
}
 
 // Convert tics to mR/hr
float tics2mrem(uint16_t tics) {
  return float(tics) * TICFACTOR;
}

 void geigerInit() {
  Serial.begin(115200); // For debug
  Serial.println("Geiger counter startup");
  #ifndef ESP8266
  updateTime = millis(); // Next update time
   // attach interrupt routine to TIC interface from the geiger counter module
  pinMode(PINTIC, INPUT);
  attachInterrupt(PINTIC, TicISR, FALLING);
 
  // attach interrupt routine to internal timer, to fire every 250 ms
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 250000, true); // 250 ms
  timerAlarmEnable(timer);
  Serial.println("Geiger counter ready");
  #else
   // start counting
    memset(secondcounts, 0, sizeof(secondcounts));
    Serial.println("Starting count ...");

    pinMode(PINTIC, INPUT);
    attachInterrupt(digitalPinToInterrupt(PINTIC), TicISR, FALLING);
  #endif
}

void geigerLoop() {
 #ifdef ESP32 
 // curent mR/hr value to display - add all tics from walking average 10 seconds
  uint16_t v=0;
  for (int i=0; i<TICBUFSIZE; i++) {
     v+=tic_buf[i];
  }
     
   // convert tics to mR/hr and put in display buffer for TFT
  float mrem = tics2mrem(v); 
  char buf[12];
  dtostrf(mrem, 7, (mrem<1 ? 2: (mrem<10 ? 1 : 0)), buf); 

  Serial.print("mRem: "); Serial.println (mrem);
  Serial.print("tics: "); Serial.println(v);
#else
// update the circular buffer every second
    unsigned long int second = millis() / 1000;
    unsigned long int secidx = second % 60;
    if (secidx != secidx_prev) {
        // new second, store the counts from the last second
        unsigned long int count = counts;
        secondcounts[secidx_prev] = count - count_prev;
        count_prev = count;
        secidx_prev = secidx;
    }
    // report every LOG_PERIOD
    if ((second - second_prev) >= LOG_PERIOD) {
        second_prev = second;

        // calculate sum
        int cpm = 0;
        for (int i = 0; i < 60; i++) {
            cpm += secondcounts[i];
        }
        Serial.print("Counts: "); Serial.println(counts);
        Serial.print("cpm: "); Serial.println(cpm);
        }
#endif
}

/*// Convert tics to mR/hr
//
float tics2mrem(uint16_t tics) {
  return float(tics) * TICFACTOR;
}
*/ 

/* // convert millirem value to a log percentage on analog and bar graph
//
int mrem2perc(float mrem, int maxperc) {
  if (mrem<=0) {
    return 0;
  } */
 