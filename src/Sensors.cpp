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
    static uint_fast64_t pmLoopTimeStamp = 0;                 // timestamp for sensor loop check data
    if ((millis() - pmLoopTimeStamp > sample_time * 1000)) {  // sample time for each capture
        pmLoopTimeStamp = millis();
        dataReady = false;
        dataReady = pmSensorRead();
        DEBUG("-->[SENSORS] enable data from UART sensors: ",String(dataReady).c_str());
        dhtRead();
        am2320Read();
        bme280Read();
        bme680Read();
        aht10Read();
        sht31Read();
        CO2scd30Read();

        if(!dataReady)DEBUG("-->[SENSORS] Any data from sensors? check your wirings!");

        if (dataReady && (_onDataCb != nullptr)) {
            _onDataCb();  // if any sensor reached any data, dataReady is true.
        } else if (!dataReady && (_onErrorCb != nullptr))
            _onErrorCb("-->[W][SENSORS] No data from any sensor!");

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
    if (devmode) Serial.println("-->[SENSORS] debug is enable.");

    DEBUG("-->[SENSORS] sample time set to: ", String(sample_time).c_str());

    if (!_only_i2c_sensors && !sensorSerialInit(pms_type, pms_rx, pms_tx)) {
        DEBUG("-->[PMSENSOR] not found any PM sensor via UART");
    }

#ifdef M5COREINK
    Wire.begin(25,26);  // M5CoreInk hat pines (header on top)
#endif
    Wire.begin();
    
    DEBUG("-->[SENSORS] trying to load I2C sensors..");
    sps30I2CInit();
    am2320Init();
    sht31Init();
    bme280Init();
    bme680Init();
    aht10Init();
    dhtInit();
    CO2scd30Init();
}

/// set loop time interval for each sensor sample
void Sensors::setSampleTime(int seconds) {
    sample_time = seconds;
    if(getPmDeviceSelected().equals("SCD30")){
        Serial.println("-->[SENSORS] SCD30 interval time to (2x): " + String(seconds * 2));
        scd30.setMeasurementInterval(seconds * 2);
    }
}

/// set CO2 recalibration PPM value (400 to 2000)
void Sensors::setCO2RecalibrationFactor(int ppmValue) {
    if (getPmDeviceSelected().equals("SCD30")) {
        Serial.println("-->[SENSORS] SCD30 setting calibration factor: " + String(ppmValue));
        scd30.setForcedRecalibrationFactor(400);
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

float Sensors::getGas() {
    return gas;
}

float Sensors::getAltitude() {
    return alt;
}

float Sensors::getPressure() {
    return pres;
}

bool Sensors::isPmSensorConfigured() {
    return device_type >= 0;
}

String Sensors::getPmDeviceSelected() {
    return device_selected;
}

int Sensors::getPmDeviceTypeSelected() {
    return device_type;
}

void Sensors::detectI2COnly(bool enable) {
    _only_i2c_sensors = enable;
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
            DEBUG("-->[PMS-HPMA] read > done!");
            pm25 = txtMsg[6] * 256 + byte(txtMsg[7]);
            pm10 = txtMsg[8] * 256 + byte(txtMsg[9]);
            if (pm25 > 1000 && pm10 > 1000) {
                onSensorError("-->[E][PMSENSOR] out of range pm25 > 1000");
            } else
                return true;
        } else {
            onSensorError("-->[E][PMSENSOR] invalid Generic sensor header!");
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
        DEBUG("-->[PANASONIC] read > done!");
        pm1 = txtMsg[2] * 256 + byte(txtMsg[1]);
        pm25 = txtMsg[6] * 256 + byte(txtMsg[5]);
        pm10 = txtMsg[10] * 256 + byte(txtMsg[9]);
        if (pm25 > 2000 && pm10 > 2000) {
            onSensorError("-->[E][PMSENSOR] out of range pm25 > 2000");
        } else
            return true;
    } else {
        onSensorError("-->[E][PMSENSOR] invalid Panasonic sensor header!");
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
            DEBUG("-->[SDS011] read > done!");
            pm25 = (txtMsg[3] * 256 + byte(txtMsg[2])) / 10;
            pm10 = (txtMsg[5] * 256 + byte(txtMsg[4])) / 10;
            if (pm25 > 1000 && pm10 > 1000) {
                onSensorError("-->[E][PMSENSOR] out of range pm25 > 1000");
            } else
                return true;
        } else {
            onSensorError("-->[E][PMSENSOR] invalid Generic sensor header!");
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
        onSensorError("-->[E][PMSENSOR] sensor read fail!");
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
    do {
        ret = sps30.GetValues(&val);
        if (ret == ERR_DATALENGTH) {
            if (error_cnt++ > 3) {
                DEBUG("-->[E][SPS30] Error during reading values: ", String(ret).c_str());
                return false;
            }
            delay(500);
        } else if (ret != ERR_OK) {
            sps30ErrToMess((char *)"-->[W][SPS30] Error during reading values: ", ret);
            return false;
        }
    } while (ret != ERR_OK);

    DEBUG("-->[SPS30] read > done!");

    pm1 = round(val.MassPM1);
    pm25 = round(val.MassPM2);
    pm4 = round(val.MassPM4);
    pm10 = round(val.MassPM10);

    if (pm25 > 1000 && pm10 > 1000) {
        onSensorError("-->[E][SPS30] Sensirion out of range pm25 > 1000");
        return false;
    }

    return true;
}

bool Sensors::CO2Mhz19Read() {
    CO2 = mhz19.getCO2();              // Request CO2 (as ppm)
    CO2temp = mhz19.getTemperature();  // Request Temperature (as Celsius)
    if (CO2 > 0) {
        dataReady = true;
        DEBUG("-->[MHZ14-9] read > done!");
        return true;
    }
    return false;
}

bool Sensors::CO2CM1106Read() {
    CO2 = sensor_CM1106->get_co2();;
    if (CO2 > 0) {
        dataReady = true;
        DEBUG("-->[CM1106] read > done!");
        return true;
    }
    return false;
}

/**
 * @brief read sensor data. Sensor selected.
 * @return true if data is loaded from sensor
 */
bool Sensors::pmSensorRead() {
    switch (device_type) {
        case Auto:
            return pmGenericRead();
            break;

        case Panasonic:
            return pmPanasonicRead();
            break;

        case Sensirion:
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

        default:
            return false;
            break;
    }
}

/******************************************************************************
*  I 2 C   S E N S O R   R E A D   M E T H O D S
******************************************************************************/

void Sensors::am2320Read() {
    float humi1 = am2320.readHumidity();
    float temp1 = am2320.readTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) {
        temp = temp1;
        dataReady = true;
        DEBUG("-->[AM2320] read > done!");
    }
}

void Sensors::bme280Read() {
    float humi1 = bme280.readHumidity();
    float temp1 = bme280.readTemperature();
    if (humi1 != 0) humi = humi1;
    if (temp1 != 0) {
        temp = temp1;
        dataReady = true;
        DEBUG("-->[BME280] read > done!");
    }
}

void Sensors::bme680Read() {
    unsigned long endTime = bme680.beginReading();
    if (endTime == 0) return;
    if (!bme680.endReading()) return;

    float temp1 = bme680.temperature;

    if (temp1 != 0) {
        temp = temp1;
        humi = bme680.humidity;
        pres = bme680.pressure / 100.0;
        gas  = bme680.gas_resistance / 1000.0;
        alt  = bme680.readAltitude(SEALEVELPRESSURE_HPA);

        dataReady = true;
        DEBUG("-->[BME680] read > done!");
    }
}

void Sensors::aht10Read() {
    float humi1 = aht10.readHumidity();
    float temp1 = aht10.readTemperature();
    if (humi1 != 255) humi = humi1;
    if (temp1 != 255) {
        temp = temp1;
        dataReady = true;
        DEBUG("-->[AHT10] read > done!");
    }
}

void Sensors::sht31Read() {
    float humi1 = sht31.readHumidity();
    float temp1 = sht31.readTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) {
        temp = temp1;
        dataReady = true;
        DEBUG("-->[SHT31] read > done!");
    }
}

void Sensors::CO2scd30Read() {
    uint16_t tco2 = scd30.getCO2();
    if (tco2 > 0) {
        CO2 = tco2;
        CO2humi = scd30.getHumidity();
        CO2temp = scd30.getTemperature();
        dataReady = true;
        DEBUG("-->[SCD30] read > done!");
    }
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
        temp = dhttemp;
        humi = dhthumi;
        dataReady = true;
        DEBUG("-->[DHTXX] read > done!");
    }
}

void Sensors::onSensorError(const char *msg) {
    DEBUG(msg);
    if (_onErrorCb) _onErrorCb(msg);
}

void Sensors::sps30ErrToMess(char *mess, uint8_t r) {
    char buf[80];
    sps30.GetErrDescription(r, buf, 80);
    DEBUG("-->[E][SENSIRION]", buf);
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
        DEBUG("-->[PMSENSOR][UART] detecting Generic PM sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    }
    // set UART for custom sensors
    else if (pms_type == Panasonic) {
        DEBUG("-->[PMSENSOR][UART] detecting Panasonic PM sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    } else if (pms_type == Sensirion) {
        DEBUG("-->[PMSENSOR][UART] detecting SPS30 PM sensor..");
        if (!serialInit(pms_type, 115200, pms_rx, pms_tx)) return false;
    } else if (pms_type == SDS011) {
        DEBUG("-->[PMSENSOR][UART] detecting SDS011 PM sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    } else if (pms_type == Mhz19) {
        DEBUG("-->[CO2SENSOR][UART] detecting MHZ19 sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    } else if (pms_type == CM1106) {
        DEBUG("-->[CO2SENSOR][UART] detecting CM1106 sensor..");
        if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
    }

    // starting auto detection loop
    int try_sensor_init = 0;
    while (!pmSensorAutoDetect(pms_type) && try_sensor_init++ < 2);

    // get device selected..
    if (device_type >= 0) {
        DEBUG("-->[PMSENSOR] detected: ", device_selected.c_str());
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

    if (pms_type == Sensirion) {
        if (sps30UARTInit()) {
            device_selected = "SENSIRION";
            device_type = Sensirion;
            return true;
        }
    }

    if (pms_type == SDS011) {
        if (pmSDS011Read()) {
            device_selected = "SDS011";
            device_type = SDS011;
            return true;
        }
    }

    if (pms_type == Mhz19) {
        if (CO2Mhz19Init()) {
            device_selected = "MHZ19";
            device_type = Mhz19;
            return true;
        }
    }

    if (pms_type == CM1106) {
        if (CO2CM1106Init()) {
            device_selected = "CM1106";
            device_type = CM1106;
            return true;
        }
    }

    if (pms_type <= Panasonic) {
        if (pmGenericRead()) {
            device_selected = "GENERIC";
            device_type = Auto;
            return true;
        }
        delay(1000);  // sync serial
        if (pmPanasonicRead()) {
            device_selected = "PANASONIC";
            device_type = Panasonic;
            return true;
        }
    }

    return false;
}

bool Sensors::CO2Mhz19Init() {
    DEBUG("-->[MH-Z19] starting MH-Z14 or MH-Z19 sensor..");
    mhz19.begin(*_serial);    // *Serial(Stream) refence must be passed to library begin().
    mhz19.autoCalibration(false);  // Turn auto calibration ON (OFF autoCalibration(false))
    return true;
}

bool Sensors::CO2CM1106Init() {
    DEBUG("-->[CM1106] starting CM1106 sensor..");
    sensor_CM1106 = new CM1106_UART(*_serial);

    // Check if CM1106 is available
    sensor_CM1106->get_software_version(sensor.softver);
    int len = strlen(sensor.softver);
    if (len > 0) {
        if (len >= 10 && !strncmp(sensor.softver+len-5, "SL-NS", 5)) {
            DEBUG("-->[CM1106] CM1106SL-NS version detected");
        } else if (!strncmp(sensor.softver, "CM", 2)) {
            DEBUG("-->[CM1106] CM1106 version detected");
        } else {
            DEBUG("-->[CM1106] unknown version");
        }
    } else {
        DEBUG("-->[E][CM1106] not detected!");
        return false;
    }     

    // Show sensor info
    DEBUG("-->[CM1106] Cubic CM1106 NDIR CO2 sensor <<<");  
    sensor_CM1106->get_serial_number(sensor.sn);
    DEBUG("-->[CM1106] Serial number:", sensor.sn);
    DEBUG("-->[CM1106] Software version:", sensor.softver);

    // Setup ABC parameters
    DEBUG("-->[CM1106] Setting ABC parameters...");
    sensor_CM1106->set_ABC(CM1106_ABC_OPEN, 7, 415);    // 7 days cycle, 415 ppm for base

    // Force mode continous B for CM1106SL-NS
    sensor_CM1106->set_working_status(1);

    // // Getting ABC parameters
    // if (sensor_CM1106->get_ABC(&abc)) {
    //     DEBUG("-->[CM1106] ABC parameters:");
    //     if (abc.open_close == CM1106_ABC_OPEN) {
    //         DEBUG("-->[CM1106] Auto calibration is enabled");
    //     } else if (abc.open_close == CM1106_ABC_CLOSE) {
    //         DEBUG("-->[CM1106] Auto calibration is disabled");
    //     }
    //     DEBUG("-->[CM1106] Calibration cycle: ", String(abc.cycle).c_str());
    //     DEBUG("-->[CM1106] Calibration baseline: ", String(abc.base).c_str());
    // }

    // // Start calibration
    // DEBUG("Starting calibration...");
    // sensor_CM1106->start_calibration(400);

    return true;
}

bool Sensors::sps30UARTInit() {
    // Begin communication channel
    DEBUG("-->[SPS30] starting SPS30 (UART) sensor..");

    // set driver debug level
    sps30.EnableDebugging(devmode);
    // Begin communication channel;
    if (!sps30.begin(SENSOR_COMMS)) {
        sps30Errorloop((char *)"-->[E][SPS30] could not initialize communication channel.", 0);
        return false;
    }
    // check for SPS30 connection
    if (!sps30.probe()) {
        sps30Errorloop((char *)"-->[E][SPS30] could not probe with SPS30.", 0);
        return false;
    }
    else {
        DEBUG("-->[SPS30] Detected SPS30 via UART.");
        sps30DeviceInfo();
    }
    // reset SPS30 connection
    if (!sps30.reset())
        sps30Errorloop((char *)"-->[E][SPS30] could not reset.", 0);

    // start measurement
    if (sps30.start() == true) {
        DEBUG("-->[SPS30] Measurement OK");
        return true;
    } else
        sps30Errorloop((char *)"-->[E][SPS30] Could NOT start measurement", 0);

    return false;
}

bool Sensors::sps30I2CInit() {
    if (device_type == Sensirion) return false;
    
    DEBUG("-->[SPS30] starting SPS30 (I2C) sensor..");

    // set driver debug level
    sps30.EnableDebugging(devmode);
    // Begin communication channel;
    if (sps30.begin(&Wire) == false) {
        sps30Errorloop((char *)"-->[E][SPS30] Could not set I2C communication channel.", 0);
        return false;
    }

    // check for SPS30 connection
    if (!sps30.probe()) {
        sps30Errorloop((char *)"-->[E][SPS30] Could not probe with SPS30.", 0);
        return false;
    }
    else {
        DEBUG("-->[SPS30] Detected SPS30 via I2C.");
        sps30DeviceInfo();
    }

    // reset SPS30 connection
    if (!sps30.reset()) 
        sps30Errorloop((char *)"-->[E][SPS30] could not reset.", 0);


    // start measurement
    if (sps30.start()) {
        sps30Read();
        DEBUG("-->[SPS30] Measurement OK");
        device_selected = "SENSIRION";
        device_type = Sensirion;
        if (sps30.I2C_expect() == 4)
            DEBUG("-->[E][SPS30] Due to I2C buffersize only PM values  \n");
        return true;
    }
    else
        sps30Errorloop((char *)"-->[E][SPS30] Could NOT start measurement.", 0);

    return false;
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
            DEBUG("-->[SPS30] Serial number : ", buf);
        else
            DEBUG("not available");
    } else
        DEBUG("[SPS30] could not get serial number");

    // try to get product name
    ret = sps30.GetProductName(buf, 32);
    if (ret == ERR_OK) {
        if (strlen(buf) > 0)
            DEBUG("-->[SPS30] Product name  : ", buf);
        else
            DEBUG("not available");
    } else
        DEBUG("[SPS30] could not get product name.");

    // try to get version info
    ret = sps30.GetVersion(&v);
    if (ret != ERR_OK) {
        DEBUG("[SPS30] Can not read version info");
        return;
    }
    sprintf(buf, "%d.%d", v.major, v.minor);
    DEBUG("-->[SPS30] Firmware level: ", buf);

    if (SENSOR_COMMS != I2C_COMMS) {
        sprintf(buf, "%d.%d", v.SHDLC_major, v.SHDLC_minor);
        DEBUG("-->[SPS30] Hardware level: ", String(v.HW_version).c_str());
        DEBUG("-->[SPS30] SHDLC protocol: ", buf);
    }

    sprintf(buf, "%d.%d", v.DRV_major, v.DRV_minor);
    DEBUG("-->[SPS30] Library level : ", buf);
}

void Sensors::am2320Init() {
    DEBUG("-->[AM2320] starting AM2320 sensor..");
    am2320 = Adafruit_AM2320();
    am2320.begin();  // temp/humidity sensor
}

void Sensors::sht31Init() {
    DEBUG("-->[SHT31] starting SHT31 sensor..");
    sht31 = Adafruit_SHT31();
    sht31.begin(0x44);  // temp/humidity sensor
}

void Sensors::bme280Init() {
    DEBUG("-->[BME280] starting BME280 sensor..");
    bme280.begin(0x76);  // temp/humidity sensor
}

void Sensors::bme680Init() {
    DEBUG("-->[BME280] starting BME680 sensor..");
    if (!bme680.begin()) return;
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);  // 320*C for 150 ms
    DEBUG("BME680 set sea level pressure ", String(SEALEVELPRESSURE_HPA).c_str());
}

void Sensors::aht10Init() {
    DEBUG("-->[AHT10] starting AHT10 sensor..");
    aht10 = AHT10(AHT10_ADDRESS_0X38);
    aht10.begin();  // temp/humidity sensor
}

void Sensors::CO2scd30Init() {
    DEBUG("-->[SCD30] starting CO2 SCD30 sensor..");
    scd30.begin();
    delay(10);
    CO2scd30Read();
    if (CO2 > 0) {
        DEBUG("-->[SCD30] detected!");      
        device_selected = "SCD30";  // TODO: sync this constants with app
        device_type = 6;
    }
}

void Sensors::dhtInit() {
}

/// Print some sensors values
void Sensors::printValues() {
    char output[200];
    sprintf(output, "PM1:%03d PM25:%03d PM10:%03d CO2:%04d CO2humi:%03f%% CO2temp:%03f°C H:%03f%% T:%03f°C", pm1, pm25, pm10, CO2, CO2humi, CO2temp, humi, temp);
    DEBUG("-->[SENSORS]", output);
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
                DEBUG("TX/RX line not defined");
                return false;
            }
            Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
            _serial = &Serial1;
            break;

        case SERIALPORT2:
            if (pms_type == Sensirion)
                Serial2.begin(speed_baud);
            else
                Serial2.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
            _serial = &Serial2;
            break;
#endif
        default:

            if (pms_rx == 0 || pms_tx == 0) {
                DEBUG("TX/RX line not defined");
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
                DEBUG("-->[SENSORS] swSerial init on pin: ", String(pms_rx).c_str());
                static SoftwareSerial swSerial(pms_rx, pms_tx);
                if (pms_type == Sensirion)
                    swSerial.begin(speed_baud);
                else if (pms_type == Panasonic)
                    swSerial.begin(speed_baud, SWSERIAL_8E1, pms_rx, pms_tx, false);
                else
                    swSerial.begin(speed_baud, SWSERIAL_8N1, pms_rx, pms_tx, false);
                _serial = &swSerial;
#else
                DEBUG("SoftWareSerial not enabled\n");
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
