#include "Sensors.hpp"

DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

/***********************************************************************************
 *  P U B L I C   M E T H O D S
 * *********************************************************************************/

/**
 * Main sensors loop.
 * All sensors are read here, please call it on main loop.
 */
void Sensors::loop() {
    static uint32_t pmLoopTimeStamp = 0;                 // timestamp for sensor loop check data
    if ((millis() - pmLoopTimeStamp > sample_time * (uint32_t)1000)) {  // sample time for each capture
        pmLoopTimeStamp = millis();
        dataReady = false;
        if(!i2conly ) {
            dataReady = pmSensorRead();
            DEBUG("-->[SLIB] UART sensors count: ",String(dataReady).c_str());
        }

        dhtRead();
        am2320Read();
        bme280Read();
        bmp280Read();
        bme680Read();
        aht10Read();
        sht31Read();
        CO2scd30Read();
        CO2scd4xRead();
        PMGCJA5Read();

        if(i2conly && dev_uart_type == SSPS30) sps30Read();

        if(!dataReady)DEBUG("-->[SLIB] Any data from sensors? check your wirings!");

        if (dataReady && (_onDataCb != nullptr)) {
            _onDataCb();  // if any sensor reached any data, dataReady is true.
        } else if (!dataReady && (_onErrorCb != nullptr))
            _onErrorCb("[W][SLIB] No data from any sensor!");

        printValues();

    }

    dhtRead();  // DHT2x sensors need check fastest
}

/**
 * All sensors init.
 * Particle meter sensor (PMS) and AM2320 sensor init.
 * 
 * @param pms_type PMS type, please see DEVICE_TYPE enum.
 * @param pms_rx PMS RX pin.
 * @param pms_tx PMS TX pin.
 * @param debug enable PMS log output.
 */
void Sensors::init(int pms_type, int pms_rx, int pms_tx) {
// override with debug INFO level (>=3)
#ifdef CORE_DEBUG_LEVEL
    if (CORE_DEBUG_LEVEL >= 3) devmode = true;
#endif
    if (devmode) Serial.println("-->[SLIB] debug is enable.");

    Serial.println("-->[SLIB] temperature offset: " + String(toffset));
    Serial.println("-->[SLIB] altitude offset: " + String(altoffset));
    Serial.println("-->[SLIB] forced only i2c sensors: " + String(i2conly));

    if (!i2conly && !sensorSerialInit(pms_type, pms_rx, pms_tx)) {
        DEBUG("-->[SLIB] not found any PM sensor via UART");
    }

#ifdef M5COREINK
    Wire.begin(25,26);  // M5CoreInk hat pines (header on top)
#endif
    Wire.begin();
    
    DEBUG("-->[SLIB] trying to load I2C sensors..");
    sps30I2CInit();
    PMGCJA5Init();
    am2320Init();
    sht31Init();
    bme280Init();
    bmp280Init();
    bme680Init();
    aht10Init();
    dhtInit();
    CO2scd30Init();
    CO2scd4xInit();
}

/// set loop time interval for each sensor sample
void Sensors::setSampleTime(int seconds) {
    sample_time = seconds;
    Serial.println("-->[SLIB] new sample time: " + String(seconds));
    if(getMainDeviceSelected().equals("SCD30")){
        scd30.setMeasurementInterval(seconds * 2);
        Serial.println("-->[SLIB] SCD30 interval time to (2x): " + String(seconds * 2));
    }
}

/// set CO2 recalibration PPM value (400 to 2000)
void Sensors::setCO2RecalibrationFactor(int ppmValue)
{
    Serial.print("DEVICE SELECTED: ");
    Serial.println(getMainDeviceSelected());
    if (getMainDeviceSelected().equals("SCD30")) {
        Serial.println("-->[SLIB] SCD30 setting calibration to: " + String(ppmValue));
        scd30.setForcedRecalibrationFactor(ppmValue);
    }
 if (getMainDeviceSelected().equals("CM1106")) {
        Serial.println("-->[SLIB] CM1106 setting calibration to: " + String(ppmValue));
        cm1106->start_calibration(ppmValue);
    }   
    if (getMainDeviceSelected().equals("MHZ19")) {
        Serial.println("-->[SLIB] MH-Z19 setting calibration to: " + String(ppmValue));
        mhz19.calibrate();
    }
    if (getMainDeviceSelected().equals("SENSEAIRS8")) {
        Serial.println("-->[SLIB] SenseAir S8 setting calibration to: " + String(ppmValue));
        if (s8->manual_calibration()) Serial.println("-->[SLIB] S8 calibration ready.");
    }
    if (getMainDeviceSelected().equals("SCD4x")) {
        Serial.println("-->[SLIB] SCD4x setting calibration to: " + String(ppmValue));
        uint16_t frcCorrection;
        uint16_t error = 0;
        char errorMessage[256];
        scd4x.stopPeriodicMeasurement();
        delay(510);
        error = scd4x.performForcedRecalibration(ppmValue, frcCorrection);
        if (error) {
            Serial.print("Error trying to execute performForcedRecalibration(): ");
            errorToString(error, errorMessage, 256);
            Serial.println(errorMessage);
            return;
        }
        delay(50);
        scd4x.startPeriodicMeasurement();
    }
}

void Sensors::setCO2AltitudeOffset(float altitude){
    this->altoffset = altitude;
    this->hpa = hpaCalculation(altitude);       //hPa hectopascal calculation based on altitude

    if (getMainDeviceSelected().equals("SCD30")) {
        setSCD30AltitudeOffset(altoffset);
    }
    if (getMainDeviceSelected().equals("SCD4x")) {
        scd4x.stopPeriodicMeasurement();
        delay(510);
        scd4x.setSensorAltitude(altoffset);
        delay(100);
        scd4x.startPeriodicMeasurement();
    }
}

void Sensors::restart() {
    _serial->flush();
    init();
    delay(100);
}

void Sensors::setOnDataCallBack(voidCbFn cb) {
    _onDataCb = cb;
}

void Sensors::setOnErrorCallBack(errorCbFn cb) {
    _onErrorCb = cb;
}

void Sensors::setDebugMode(bool enable) {
    devmode = enable;
}

bool Sensors::isDataReady() {
    return dataReady;
}

uint16_t Sensors::getPM1() {
    return pm1;
}

String Sensors::getStringPM1() {
    char output[5];
    sprintf(output, "%03d", getPM1());
    return String(output);
}

uint16_t Sensors::getPM25() {
    return pm25;
}

String Sensors::getStringPM25() {
    char output[5];
    sprintf(output, "%03d", getPM25());
    return String(output);
}

uint16_t Sensors::getPM4() {
    return pm4;
}

String Sensors::getStringPM4() {
    char output[5];
    sprintf(output, "%03d", getPM4());
    return String(output);
}

uint16_t Sensors::getPM10() {
    return pm10;
}

String Sensors::getStringPM10() {
    char output[5];
    sprintf(output, "%03d", getPM10());
    return String(output);
}

uint16_t Sensors::getCO2() {
    return CO2;
}

String Sensors::getStringCO2() {
    char output[5];
    sprintf(output, "%04d", getCO2());
    return String(output);
}

float Sensors::getCO2humi() {
    return CO2humi;
}

float Sensors::getCO2temp() {
    return CO2temp;
}

float Sensors::getHumidity() {
    return humi;
}

float Sensors::getTemperature() {
    return temp;
}

void Sensors::setTempOffset(float offset){
    toffset = offset;
    setSCD30TempOffset(toffset);
    setSCD4xTempOffset(toffset);
}

float Sensors::getGas() {
    return gas;
}

float Sensors::getAltitude() {
    return alt;
}

float Sensors::getPressure() {
    return pres;
}

bool Sensors::isUARTSensorConfigured() {
    return dev_uart_type >= 0;
}

String Sensors::getMainDeviceSelected() {
    return device_selected;
}

int Sensors::getUARTDeviceTypeSelected() {
    return dev_uart_type;
}

int Sensors::getMainSensorTypeSelected() {
    if (device_selected.isEmpty()) return SENSOR_NONE;
    else if (dev_uart_type >= 0 && dev_uart_type <= SDS011) return SENSOR_PM; // TODO: we need dev_i2c_type ??
    return SENSOR_CO2;
}

void Sensors::detectI2COnly(bool enable) {
    i2conly = enable;
}

/******************************************************************************
*  U A R T   S E N S O R   P R I V A T E   M E T H O D S
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
            DEBUG("-->[SLIB] UART PMGENERIC read > done!");
            pm25 = txtMsg[6] * 256 + (char)(txtMsg[7]);
            pm10 = txtMsg[8] * 256 + (char)(txtMsg[9]);
            if (pm25 > 1000 && pm10 > 1000) {
                onSensorError("[E][SLIB] UART PMGENERIC out of range pm25 > 1000");
            } else
                return true;
        } else {
            onSensorError("[E][SLIB] UART PMGENERIC invalid sensor header!");
        }
    }
    return false;
}

/**
 *  @brief Panasonic SNGC particulate meter sensor read.
 *  @return true if header and sensor data is right
 */
bool Sensors::pmPanasonicRead() {
    int lenght_buffer = 32;
    String txtMsg = hwSerialRead(lenght_buffer);
    if (txtMsg[0] == 02) {
        DEBUG("-->[SLIB] PANASONIC read > done!");
        pm1 = txtMsg[2] * 256 + (char)(txtMsg[1]);
        pm25 = txtMsg[6] * 256 + (char)(txtMsg[5]);
        pm10 = txtMsg[10] * 256 + (char)(txtMsg[9]);
        if (pm25 > 2000 && pm10 > 2000) {
            onSensorError("[E][SLIB] PANASONIC out of range pm25 > 2000");
        } else
            return true;
    } else {
        onSensorError("[E][SLIB] PANASONIC invalid sensor header!");
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
            DEBUG("-->[SLIB] SDS011 read > done!");
            pm25 = (txtMsg[3] * 256 + (char)(txtMsg[2])) / 10;
            pm10 = (txtMsg[5] * 256 + (char)(txtMsg[4])) / 10;
            if (pm25 > 1000 && pm10 > 1000) {
                onSensorError("[E][SLIB] SDS011 out of range pm25 > 1000");
            } else
                return true;
        } else {
            onSensorError("[E][SLIB] SDS011 invalid sensor header!");
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
        DEBUG("-->[SLIB] no data on UART port");
    }
    return txtMsg;
}

/**
 *  @brief Sensirion SPS30 particulate meter sensor read.
 *  @return true if reads succes
 */
bool Sensors::sps30Read() {
    uint8_t ret, error_cnt = 0;

    delay(35);  //Delay for sincronization
    
    if(i2conly && sample_time > 30) {
        if (!sps30.start()) return false;  // power saving validation
        delay(15000);
    }
    
    do {
        ret = sps30.GetValues(&val);
        if (ret == ERR_DATALENGTH) {
            if (error_cnt++ > 3) {
                DEBUG("[E][SLIB] SPS30 Error during reading values: ", String(ret).c_str());
                return false;
            }
            delay(500);
        } else if (ret != ERR_OK) {
            sps30ErrToMess((char *)"[W][SLIB] SPS30 Error during reading values: ", ret);
            return false;
        }
    } while (ret != ERR_OK);

    DEBUG("-->[SLIB] SPS30 read > done!");

    pm1 = round(val.MassPM1);
    pm25 = round(val.MassPM2);
    pm4 = round(val.MassPM4);
    pm10 = round(val.MassPM10);

    if(i2conly && sample_time > 30) sps30.stop();  // power saving validation

    if (pm25 > 1000 && pm10 > 1000) {
        onSensorError("[E][SLIB] SPS30 Sensirion out of range pm25 > 1000");
        return false;
    }

    dataReady = true;

    return true;
}

bool Sensors::CO2Mhz19Read() {
    CO2 = mhz19.getCO2();              // Request CO2 (as ppm)
    CO2temp = mhz19.getTemperature()-toffset;  // Request Temperature (as Celsius)
    if (CO2 > 0) {
        if(altoffset != 0) CO2correctionAlt();
        dataReady = true;
        DEBUG("-->[SLIB] MHZ14-9 read > done!");
        return true;
    }
    return false;
}

bool Sensors::CO2CM1106Read() {
    CO2 = cm1106->get_co2();;
    if (CO2 > 0) {
        dataReady = true;
        if(altoffset != 0) CO2correctionAlt();
        DEBUG("-->[SLIB] CM1106 read > done!");
        return true;
    }
    return false;
}

bool Sensors::senseAirS8Read() {
    CO2 = s8->get_co2();      // Request CO2 (as ppm)
    if (CO2 > 0) {
        if(altoffset != 0) CO2correctionAlt();
        dataReady = true;
        DEBUG("-->[SLIB] SENSEAIRS8 read > done!");
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

        case Panasonic:
            return pmPanasonicRead();
            break;

        case SSPS30:
            return sps30Read();
            break;

        case SDS011:
            return pmSDS011Read();
            break;

        case Mhz19:
            return CO2Mhz19Read();
            break;

        case CM1106:
            return CO2CM1106Read();
            break;

        case SENSEAIRS8:
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
    int status = am2320.read();
    if (status != AM232X_OK) return;
    float humi1 = am2320.getHumidity();
    float temp1 = am2320.getTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) {
        temp = temp1-toffset;
        dataReady = true;
        DEBUG("-->[SLIB] AM2320 read > done!");
    }
}

void Sensors::bme280Read() {
    float humi1 = bme280.readHumidity();
    float temp1 = bme280.readTemperature();
    if (isnan(humi1) || humi1 == 0 || isnan(temp1)) return;
    humi = humi1;
    temp = temp1-toffset;
    pres = bme280.readPressure();
    alt = bme280.readAltitude(SEALEVELPRESSURE_HPA);
    dataReady = true;
    DEBUG("-->[SLIB] BME280 read > done!");
}

void Sensors::bmp280Read() {
    float temp1 = bmp280.readTemperature();
    float press1 = bmp280.readPressure();
    if (press1 == 0) return;
    temp = temp1-toffset;
    pres = bmp280.readPressure();
    alt = bmp280.readAltitude(SEALEVELPRESSURE_HPA);
    dataReady = true;
    DEBUG("-->[SLIB] BMP280 read > done!");
}

void Sensors::bme680Read() {
    unsigned long endTime = bme680.beginReading();
    if (endTime == 0) return;
    if (!bme680.endReading()) return;

    float temp1 = bme680.temperature;

    if (temp1 != 0) {
        temp = temp1-toffset;
        humi = bme680.humidity;
        pres = bme680.pressure / 100.0;
        gas  = bme680.gas_resistance / 1000.0;
        alt  = bme680.readAltitude(SEALEVELPRESSURE_HPA);

        dataReady = true;
        DEBUG("-->[SLIB] BME680 read > done!");
    }
}

void Sensors::aht10Read() {
    float humi1 = aht10.readHumidity();
    float temp1 = aht10.readTemperature();
    if (humi1 != 255) humi = humi1;
    if (temp1 != 255) {
        temp = temp1-toffset;
        dataReady = true;
        DEBUG("-->[SLIB] AHT10 read > done!");
    }
}

void Sensors::sht31Read() {
    float humi1 = sht31.readHumidity();
    float temp1 = sht31.readTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) {
        temp = temp1-toffset;
        dataReady = true;
        DEBUG("-->[SLIB] SHT31 read > done!");
    }
}

void Sensors::CO2scd30Read() {
    uint16_t tCO2 = scd30.getCO2();  // we need temp var, without it override CO2
    if (tCO2 > 0) {
        CO2 = tCO2;
        CO2humi = scd30.getHumidity();
        CO2temp = scd30.getTemperature();
        dataReady = true;
        DEBUG("-->[SLIB] SCD30 read > done!");
    }
}

void Sensors::CO2scd4xRead()
{
    uint16_t error = 0;
    char errorMessage[256];
    uint16_t tCO2 = 0;
    float tCO2temp, tCO2humi = 0; // we need temp vars, without it override values
    if (getMainDeviceSelected() != "SCD4x") return;
    error = scd4x.readMeasurement(tCO2, tCO2temp, tCO2humi);
    if (error) {
        DEBUG("[E][SLIB] SCD4x Error reading measurement: ", String(error).c_str());
        errorToString(error, errorMessage, 256);
        DEBUG("[E][SLIB] SCD4x ", errorMessage);
        return;
    } else {
        CO2 = tCO2;
        CO2humi = tCO2humi;
        CO2temp = tCO2temp;
        dataReady = true;
        DEBUG("-->[SLIB] SCD4x read > done!");
    }
}

void Sensors::PMGCJA5Read() {
    if (!getMainDeviceSelected().equals("PANASONIC_I2C")) return;
    pm1 = pmGCJA5.getPM1_0();
    pm25 = pmGCJA5.getPM2_5();
    pm10 = pmGCJA5.getPM10();
    dataReady = true;
    DEBUG("-->[SLIB] GCJA5 read > done!");
}

bool Sensors::dhtIsReady(float *temperature, float *humidity) {
    static unsigned long measurement_timestamp = millis();

    /* Measure once every four seconds. */
    if (millis() - measurement_timestamp > 4000ul) {
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
    if (dhtIsReady(&dhttemp, &dhthumi) == true) {
        temp = dhttemp-toffset;
        humi = dhthumi;
        dataReady = true;
        DEBUG("-->[SLIB] DHTXX read > done!");
    }
}

void Sensors::onSensorError(const char *msg) {
    DEBUG(msg);
    if (_onErrorCb != nullptr ) _onErrorCb(msg);
}

void Sensors::sps30ErrToMess(char *mess, uint8_t r) {
    char buf[80];
    sps30.GetErrDescription(r, buf, 80);
    DEBUG("[E][SLIB] SPS30", buf);
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
        DEBUG("-->[SLIB] UART detecting Generic PM sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    }
    // set UART for custom sensors
    else if (pms_type == Panasonic) {
        DEBUG("-->[SLIB] UART detecting Panasonic PM sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    } else if (pms_type == SSPS30) {
        DEBUG("-->[SLIB] UART detecting SPS30 PM sensor..");
        if (!serialInit(pms_type, 115200, pms_rx, pms_tx)) return false;
    } else if (pms_type == SDS011) {
        DEBUG("-->[SLIB] UART detecting SDS011 PM sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    } else if (pms_type == Mhz19) {
        DEBUG("-->[SLIB] UART detecting MHZ19 sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    } else if (pms_type == CM1106) {
        DEBUG("-->[SLIB] UART detecting CM1106 sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    } else if (pms_type == SENSEAIRS8) {
        DEBUG("-->[SLIB] UART detecting SenseAir S8 sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    }

    // starting auto detection loop
    int try_sensor_init = 0;
    while (!pmSensorAutoDetect(pms_type) && try_sensor_init++ < 2);

    // get device selected..
    if (dev_uart_type >= 0) {
        DEBUG("-->[SLIB] UART detected: ", device_selected.c_str());
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

    if (pms_type == SSPS30) {
        if (sps30UARTInit()) {
            device_selected = "SENSIRION";
            dev_uart_type = SSPS30;
            return true;
        }
    }

    if (pms_type == SDS011) {
        if (pmSDS011Read()) {
            device_selected = "SDS011";
            dev_uart_type = SDS011;
            return true;
        }
    }

    if (pms_type == Mhz19) {
        if (CO2Mhz19Init()) {
            device_selected = "MHZ19";
            dev_uart_type = Mhz19;
            return true;
        }
    }

    if (pms_type == CM1106) {
        if (CO2CM1106Init()) {
            device_selected = "CM1106";
            dev_uart_type = CM1106;
            return true;
        }
    }

    if (pms_type == SENSEAIRS8) {
        if (senseAirS8Init()) {
            device_selected = "SENSEAIRS8";
            dev_uart_type = SENSEAIRS8;
            return true;
        }
    }

    if (pms_type <= Panasonic) {
        if (pmGenericRead()) {
            device_selected = "GENERIC";
            dev_uart_type = Auto;
            return true;
        }
        delay(1000);  // sync serial
        if (pmPanasonicRead()) {
            device_selected = "PANASONIC";
            dev_uart_type = Panasonic;
            return true;
        }
    }

    return false;
}

bool Sensors::CO2Mhz19Init() {
    DEBUG("-->[SLIB] MH-Z19 starting MH-Z14 or MH-Z19 sensor..");
    mhz19.begin(*_serial);
    mhz19.autoCalibration(false); 
    return true;
}

bool Sensors::CO2CM1106Init() {
    DEBUG("-->[SLIB] CM1106 starting CM1106 sensor..");
    cm1106 = new CM1106_UART(*_serial);

    // Check if CM1106 is available
    cm1106->get_software_version(cm1106sensor.softver);
    int len = strlen(cm1106sensor.softver);
    if (len > 0) {
        if (len >= 10 && !strncmp(cm1106sensor.softver+len-5, "SL-NS", 5)) {
            DEBUG("-->[SLIB] CM1106 version detected: CM1106SL-NS");
        } else if (!strncmp(cm1106sensor.softver, "CM", 2)) {
            DEBUG("-->[SLIB] CM1106 version detected: CM1106");
        } else {
            DEBUG("-->[SLIB] CM1106 version detected: unknown");
        }
    } else {
        DEBUG("[E][SLIB] CM1106 not detected!");
        return false;
    }     

    // Show sensor info
    cm1106->get_serial_number(cm1106sensor.sn);
    DEBUG("-->[SLIB] CM1106 Serial number:", cm1106sensor.sn);
    DEBUG("-->[SLIB] CM1106 Software version:", cm1106sensor.softver);

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
        DEBUG("-->[SLIB] CM1106 Calibration cycle: ", String(abc.cycle).c_str());
        DEBUG("-->[SLIB] CM1106 Calibration baseline: ", String(abc.base).c_str());
    }

    return true;
}

bool Sensors::senseAirS8Init() {
    DEBUG("-->[SLIB] SENSEAIR S8 starting sensor..");
    s8 = new S8_UART(*_serial);
    // Check if S8 is available
    s8->get_firmware_version(s8sensor.firm_version);
    int len = strlen(s8sensor.firm_version);
    if (len == 0) {
        DEBUG("[E][SLIB]SENSEAIR S8 not detected!");
        return false;
    }
    // Show S8 sensor info

    Serial.println("-->[SLIB] SENSEAIR S8 detected SenseAir S8 sensor :)");
    if (devmode) {
        Serial.printf("-->[SLIB] SENSEAIR S8 Software version: %s\n", s8sensor.firm_version);
        Serial.printf("-->[SLIB] SENSEAIR S8 Sensor type: 0x%08x\n", s8->get_sensor_type_ID());
        Serial.printf("-->[SLIB] SENSEAIR S8 Sensor ID:  %08x\n", s8->get_sensor_ID());
        Serial.printf("-->[SLIB] SENSEAIR S8 Memory map version: 0x%04x\n", s8->get_memory_map_version());
        Serial.printf("-->[SLIB] SENSEAIR S8 ABC period (0 = disabled): %d hours\n", s8->get_ABC_period());
    }
    DEBUG("-->[SLIB] SENSEAIR S8 Disable ABC period");
    s8->set_ABC_period(0);
    delay(1000);
    if (devmode) Serial.printf("-->[SLIB] SENSEAIR S8 ABC period (0 = disabled): %d hours\n", s8->get_ABC_period());

    DEBUG("-->[SLIB] SENSEAIR S8 ABC period set to 180 hours");
    s8->set_ABC_period(180);
    delay(1000);
    if (devmode) Serial.printf("-->[SLIB] SENSEAIR S8 ABC period (0 = disabled): %d hours\n", s8->get_ABC_period());

    s8->get_meter_status();
    s8->get_alarm_status();
    s8->get_output_status();
    s8->get_acknowledgement();

    return true;
}

bool Sensors::sps30UARTInit() {
    // Begin communication channel
    DEBUG("-->[SLIB] UART SPS30 starting sensor..");

    // set driver debug level
    sps30.EnableDebugging(devmode);
    // Begin communication channel;
    if (!sps30.begin(SENSOR_COMMS)) {
        sps30Errorloop((char *)"[E][SLIB] UART SPS30 could not initialize communication channel.", 0);
        return false;
    }

    if (!sps30tests()) return false;

    DEBUG("-->[SLIB] SPS30 Detected SPS30 via UART.");

    // start measurement
    if (sps30.start() == true) {
        DEBUG("-->[SLIB] SPS30 Measurement OK");
        Serial.println("-->[SLIB] UART detected SPS30 sensor :)");
        return true;
    } else
        sps30Errorloop((char *)"[E][SLIB] UART SPS30 Could NOT start measurement", 0);

    return false;
}

bool Sensors::sps30I2CInit() {
    if (dev_uart_type == SSPS30) return false;
    
    DEBUG("-->[SLIB] I2C SPS30 starting sensor..");

    // set driver debug level
    sps30.EnableDebugging(devmode);
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
        Serial.println("-->[SLIB] I2C detected SPS30 sensor :)");
        device_selected = "SENSIRION";
        dev_uart_type = SSPS30; // TODO: it isn't a uart, but it's a uart-like device
        if (sps30.I2C_expect() == 4)
            DEBUG("[E][SLIB] SPS30 due to I2C buffersize only PM values  \n");
        return true;
    }
    else
        sps30Errorloop((char *)"[E][SLIB] I2C SPS30 Could NOT start measurement.", 0);

    return false;
}

bool Sensors::sps30tests() {
    // check for SPS30 connection
    if (!sps30.probe()) {
        sps30Errorloop((char *)"[E][SLIB] SPS30 could not probe.", 0);
        return false;
    } else {
        sps30DeviceInfo();
    }
    // reset SPS30 connection
    if (!sps30.reset()) {
        sps30Errorloop((char *)"[E][SLIB] SPS30 could not reset.", 0);
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
            DEBUG("-->[SLIB] SPS30 Serial number : ", buf);
        else
            DEBUG("[SLIB] SPS30 could not get serial number");
    } else
        DEBUG("[SLIB] SPS30 could not get serial number");

    // try to get product name
    ret = sps30.GetProductName(buf, 32);
    if (ret == ERR_OK) {
        if (strlen(buf) > 0)
            DEBUG("-->[SLIB] SPS30 product name  : ", buf);
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
    DEBUG("-->[SLIB] SPS30 firmware level: ", buf);

    if (SENSOR_COMMS != I2C_COMMS) {
        sprintf(buf, "%d.%d", v.SHDLC_major, v.SHDLC_minor);
        DEBUG("-->[SLIB] SPS30 Hardware level: ", String(v.HW_version).c_str());
        DEBUG("-->[SLIB] SPS30 SHDLC protocol: ", buf);
    }

    sprintf(buf, "%d.%d", v.DRV_major, v.DRV_minor);
    DEBUG("-->[SLIB] SPS30 Library level : ", buf);
}

void Sensors::am2320Init() {
    DEBUG("-->[SLIB] AM2320 starting AM2320 sensor..");
    if (am2320.begin()) Serial.println("-->[SLIB] I2C detected AM2320 sensor :)");
}

void Sensors::sht31Init() {
    DEBUG("-->[SLIB] SHT31 starting SHT31 sensor..");
    sht31 = Adafruit_SHT31();
    if (sht31.begin()) Serial.println("-->[SLIB] I2C detected SHT31 sensor :)");
}

void Sensors::bme280Init() {
    DEBUG("-->[SLIB] BME280 starting BME280 sensor..");
    if (bme280.begin()) Serial.println("-->[SLIB] I2C detected BME280 sensor :)");
}

void Sensors::bmp280Init() {
    DEBUG("-->[SLIB] BMP280 starting BMP280 sensor..");
    if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) return;
    Serial.println("-->[SLIB] I2C detected BMP280 sensor :)");
    // Default settings from datasheet.
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,  // Operating Mode.
                    Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                    Adafruit_BMP280::FILTER_X16,      // Filtering.
                    Adafruit_BMP280::STANDBY_MS_500); // Standby time.
    Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
    Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();
    if(devmode) bmp_temp->printSensorDetails();
    if(devmode) bmp_pressure->printSensorDetails();
}

void Sensors::bme680Init() {
    DEBUG("-->[SLIB] BME680 starting BME680 sensor..");
    if (!bme680.begin()) return;
    Serial.println("-->[SLIB] I2C detected BME680 sensor :)");
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);  // 320*C for 150 ms
    DEBUG("-->[SLIB] BME680 set sea level pressure ", String(SEALEVELPRESSURE_HPA).c_str());
}

void Sensors::aht10Init() {
    DEBUG("-->[SLIB] AHT10 starting AHT10 sensor..");
    aht10 = AHT10(AHT10_ADDRESS_0X38);
    if (aht10.begin()) Serial.println("-->[SLIB] I2C detected AHT10 sensor :)");
}

void Sensors::CO2scd30Init() {
    DEBUG("-->[SLIB] SCD30 starting CO2 SCD30 sensor..");
    if (!scd30.begin()) return;
    Serial.println("-->[SLIB] I2C detected SCD30 sensor :)");
    delay(10);

    device_selected = "SCD30";  // TODO: sync this constants with app

    DEBUG("-->[SLIB] SCD30 current temperature offset: ",String(scd30.getTemperatureOffset()).c_str());
    DEBUG("-->[SLIB] SCD30 current altitude offset: ", String(scd30.getAltitudeCompensation()).c_str());

    if(scd30.getAltitudeCompensation() != uint16_t(altoffset)){
        DEBUG("-->[SLIB] SCD30 updated altitude offset to: ", String(altoffset).c_str());
        setSCD30AltitudeOffset(altoffset);
        delay(10);
    }

    if(uint16_t((scd30.getTemperatureOffset()*100)) != (uint16_t(toffset*100))) {
        setSCD30TempOffset(toffset);
        delay(10);
    }

    CO2scd30Read();

}

/// set SCD30 temperature compensation
void Sensors::setSCD30TempOffset(float offset) {
    if (getMainDeviceSelected().equals("SCD30")) {
        Serial.println("-->[SLIB] SCD30 new temperature offset: " + String(offset));
        scd30.setTemperatureOffset(offset);
    }
}

/// set SCD30 altitude compensation
void Sensors::setSCD30AltitudeOffset(float offset) {
    if (getMainDeviceSelected().equals("SCD30")) {
        Serial.println("-->[SLIB] SCD30 new altitude offset: " + String(offset));
        scd30.setAltitudeCompensation(uint16_t(offset));
    }
}

void Sensors::CO2scd4xInit() {
    float tTemperatureOffset, offsetDifference;
    uint16_t tSensorAltitude;
    uint16_t error;
    char errorMessage[256];

    DEBUG("-->[SLIB] SCD4x starting CO2 SCD4x sensor..");
    scd4x.begin(Wire);
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        DEBUG("[E][SLIB] SCD4x Error Stopping Periodic Measurement : ", String(error).c_str());
        errorToString(error, errorMessage, 256);
        DEBUG("[E][SLIB] SCD4x ", errorMessage);
        return;
    } else {
        Serial.println("-->[SLIB] I2C detected SCD4x sensor :)");
        delay(500);
    }

    device_selected = "SCD4x";  // TODO: sync this constants with app

    scd4x.getTemperatureOffset(tTemperatureOffset);
    scd4x.getSensorAltitude(tSensorAltitude);
    DEBUG("-->[SLIB] SCD4x current temperature offset: ", String(tTemperatureOffset).c_str());
    DEBUG("-->[SLIB] SCD4x current altitude offset: ", String(tSensorAltitude).c_str());

    if (tSensorAltitude != uint16_t(altoffset)) {
        setSCD4xAltitudeOffset(altoffset);
        delay(1);
    }

    offsetDifference = abs((toffset*100) - (tTemperatureOffset*100)); 
    if(offsetDifference > 0.5) { // Accounts for SCD4x conversion rounding errors in temperature offset
        Serial.println("-->[SLIB] SCD4x setting new temp offset: " + String(toffset));
        setSCD4xTempOffset(toffset);
        delay(1);
    }

    error = scd4x.startPeriodicMeasurement();
    if (error) {
        DEBUG("[E][SLIB] SCD4x Error Starting Periodic Measurement : ", String(error).c_str());
        errorToString(error, errorMessage, 256);
        DEBUG("[E][SLIB] SCD4x ", errorMessage);
        return;
    } else {
        Serial.println("-->[SLIB] I2C detected SCD4x sensor :)");
        delay(500);
    }

    if (error) return;

    CO2scd4xRead();
}

/// set SCD4x temperature compensation
void Sensors::setSCD4xTempOffset(float offset) {
    if (getMainDeviceSelected().equals("SCD4x")) {
        Serial.println("-->[SLIB] SCD4x new temperature offset: " + String(offset));
        scd4x.stopPeriodicMeasurement();
        delay(510);    
        scd4x.setTemperatureOffset(offset);
        scd4x.startPeriodicMeasurement();
    }
}

/// set SCD4x altitude compensation
void Sensors::setSCD4xAltitudeOffset(float offset) {
    if (getMainDeviceSelected().equals("SCD4x")) {
        Serial.println("-->[SLIB] SCD4x new altitude offset: " + String(offset));
        scd4x.stopPeriodicMeasurement();
        delay(510);    
        scd4x.setSensorAltitude(uint16_t(offset));
        scd4x.startPeriodicMeasurement();
    }
}

void Sensors::PMGCJA5Init() {
    if (dev_uart_type == Panasonic) return;
    DEBUG("-->[SLIB] GCJA5 starting PANASONIC GCJA5 sensor..");
    if (!pmGCJA5.begin()) return;
    Serial.println("-->[SLIB] I2C detected SN-GCJA5 sensor :)");
    device_selected = "PANASONIC_I2C";
    dev_uart_type = Auto;  // TODO: it isn't a uart, but it's a uart-like device
    uint8_t status = pmGCJA5.getStatusFan();
    DEBUG("-->[SLIB] GCJA5 FAN status: ", String(status).c_str());
}

void Sensors::dhtInit() {
}

// Altitude compensation for CO2 sensors without Pressure atm or Altitude compensation

void Sensors::CO2correctionAlt() {
    DEBUG("-->[SLIB] CO2 original:", String(CO2).c_str());
    float CO2cor = (0.016 * ((1013.25 - hpa) /10 ) * (CO2 - 400)) + CO2;       // Increment of 1.6% for every hpa of difference at sea level
    CO2 = round (CO2cor);
    DEBUG("-->[SLIB] CO2 compensated:", String(CO2).c_str());
}

float Sensors::hpaCalculation(float altitude) {
    DEBUG("-->[SLIB] Altitude Compensation for CO2 lectures ON:", String(altitude).c_str());
    float hpa = 1012 - 0.118 * altitude + 0.00000473 * altitude * altitude;            // Cuadratic regresion formula obtained PA (hpa) from high above the sea
    DEBUG("-->[SLIB] Atmospheric pressure calculated in hPa:", String(hpa).c_str());
    return hpa;
}

// Print some sensors values

void Sensors::printValues() {
    char output[200];
    sprintf(output, "PM1:%03d PM25:%03d PM10:%03d CO2:%04d CO2humi:%03f%% CO2temp:%03f°C H:%03f%% T:%03f°C", pm1, pm25, pm10, CO2, CO2humi, CO2temp, humi, temp);
    DEBUG("-->[SLIB]", output);
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

bool Sensors::serialInit(int pms_type, long speed_baud, int pms_rx, int pms_tx) {
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
            if (pms_rx == 0 || pms_tx == 0) {
                DEBUG("-->[SLIB] TX/RX line not defined");
                return false;
            }
            Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
            _serial = &Serial1;
            break;

        case SERIALPORT2:
            if (pms_type == SSPS30)
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
                DEBUG("-->[SLIB] swSerial init on pin: ", String(pms_rx).c_str());
                static SoftwareSerial swSerial(pms_rx, pms_tx);
                if (pms_type == SSPS30)
                    swSerial.begin(speed_baud);
                else if (pms_type == Panasonic)
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
