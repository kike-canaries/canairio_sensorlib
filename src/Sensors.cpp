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
        dataReady = false;
        pmLoopTimeStamp = millis();
        if(pmSensorRead()) {           
            if(_onDataCb) _onDataCb();
            dataReady = true;            // only if the main sensor is ready
        }else{
            if(_onErrorCb)_onErrorCb("-->[W][SENSORS] PM sensor not configured!");
            dataReady = false;
        }

        am2320Read();
        bme280Read();
        aht10Read();
        sht31Read();
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
    if (CORE_DEBUG_LEVEL>=3) devmode = true;  
    #endif
    if (devmode) Serial.println("-->[SENSORS] debug is enable.");

    DEBUG("-->[SENSORS] sample time set to: ",String(sample_time).c_str());
    
    if(!sensorSerialInit(pms_type, pms_rx, pms_tx)){
        DEBUG("-->[E][PMSENSOR] init failed!");
    }

    DEBUG("-->[SENSORS] try to load temp and humidity sensor..");
    am2320Init();
    sht31Init();
    bme280Init();
    aht10Init();
    dhtInit();
}

/// set loop time interval for each sensor sample
void Sensors::setSampleTime(int seconds){
    sample_time = seconds;
}

void Sensors::restart(){
    _serial->flush();
    init();
    delay(100);
}

void Sensors::setOnDataCallBack(voidCbFn cb){
    _onDataCb = cb;
}

void Sensors::setOnErrorCallBack(errorCbFn cb){
    _onErrorCb = cb;
}

void Sensors::setDebugMode(bool enable){
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

bool Sensors::isPmSensorConfigured(){
    return device_type>=0;
}

String Sensors::getPmDeviceSelected(){
    return device_selected;
}

int Sensors::getPmDeviceTypeSelected(){
    return device_type;
}

/******************************************************************************
*   S E N S O R   P R I V A T E   M E T H O D S
******************************************************************************/

/**
 *  @brief PMS sensor generic read. Supported: Honeywell & Plantower sensors
 *  @return true if header and sensor data is right
 */
bool Sensors::pmGenericRead() {
    String txtMsg = hwSerialRead();
    if (txtMsg[0] == 66) {
        if (txtMsg[1] == 77) {
            DEBUG("-->[HPMA] read > done!");
            pm25 = txtMsg[6] * 256 + byte(txtMsg[7]);
            pm10 = txtMsg[8] * 256 + byte(txtMsg[9]);
            if (pm25 > 1000 && pm10 > 1000) {
                onPmSensorError("-->[E][PMSENSOR] out of range pm25 > 1000");
            }
            else
                return true;
        } else {
            onPmSensorError("-->[E][PMSENSOR] invalid Generic sensor header!");
        }
    }
    return false;
} 

/**
 *  @brief Panasonic SNGC particulate meter sensor read.
 *  @return true if header and sensor data is right
 */
bool Sensors::pmPanasonicRead() {
    String txtMsg = hwSerialRead();
    if (txtMsg[0] == 02) {
        DEBUG("-->[PANASONIC] read > done!");
        pm1  = txtMsg[2] * 256 + byte(txtMsg[1]);
        pm25 = txtMsg[6] * 256 + byte(txtMsg[5]);
        pm10 = txtMsg[10] * 256 + byte(txtMsg[9]);
        if (pm25 > 2000 && pm10 > 2000) {
            onPmSensorError("-->[E][PMSENSOR] out of range pm25 > 2000");
        }
        else
            return true;
    } else {
        onPmSensorError("-->[E][PMSENSOR] invalid Panasonic sensor header!");
    }
    return false;
}

/**
 * @brief PMSensor Serial read to basic string
 * 
 * @param SENSOR_RETRY attempts before failure
 * @return String buffer
 **/
String Sensors::hwSerialRead() {
    int try_sensor_read = 0;
    String txtMsg = "";
    while (txtMsg.length() < 32 && try_sensor_read++ < SENSOR_RETRY) {
        while (_serial->available() > 0) {
            char inChar = _serial->read();
            txtMsg += inChar;
        }
    }
    if (try_sensor_read > SENSOR_RETRY) {
        onPmSensorError("-->[E][PMSENSOR] sensor read fail!");
    }
    return txtMsg;
}

/**
 *  @brief Sensirion SPS30 particulate meter sensor read.
 *  @return true if reads succes
 */
bool Sensors::pmSensirionRead() {
    uint8_t ret, error_cnt = 0;
    delay(35);  //Delay for sincronization
    do {
        ret = sps30.GetValues(&val);
        if (ret == ERR_DATALENGTH) {
            if (error_cnt++ > 3) {
                DEBUG("-->[E][SPS30] Error during reading values: ", String(ret).c_str());
                return false;
            }
            delay(1000);
        } else if (ret != ERR_OK) {
            pmSensirionErrtoMess((char *)"-->[W][SPS30] Error during reading values: ", ret);
            return false;
        }
    } while (ret != ERR_OK);

    DEBUG("-->[SPS30] read > done!");

    pm1  = round(val.MassPM1);
    pm25 = round(val.MassPM2);
    pm4  = round(val.MassPM4);
    pm10 = round(val.MassPM10);

    if (pm25 > 1000 && pm10 > 1000) {
        onPmSensorError("-->[E][SPS30] Sensirion out of range pm25 > 1000");
        return false;
    }

    return true;
}

bool Sensors:: CO2Mhz19Read() {
    CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
    CO2temp = myMHZ19.getTemperature();                    // Request Temperature (as Celsius)
    if(CO2>0){
        DEBUG("-->[MHZ14-9] read > done!");
        return true;
    }
    return false;
}

bool Sensors:: CO2CM1106Read() {
  CO2 = CO2CM1106val();
  Serial.print("CO2 ppm: ");
  Serial.println(CO2);
    if(CO2>0){
        DEBUG("-->[CM1106] read > done!");
        return true;
    }
    return false;
}

int Sensors:: CO2CM1106val() {
  static byte cmd[4] = {0x11, 0x01, 0x01, 0xED}; // Commands 0x01 Read ppm, 0x10 open/close ABC, 0x03 Calibrate 
  static byte response[8] = {0};                 // response 0x16, 0x05, 0x01, DF1, DF2, DF3, DF4, CRC.  ppm=DF1*256+DF2
  co2cm1106.write(cmd, 4);
  co2cm1106.readBytes(response, 8);
  int crc = 0;
  for (int i = 0; i < 7; i++) crc += response[i];
  crc = 256 - crc%256;
  if (((int) response[0] == 0x16) && ((int)response[7] == crc)) {
    unsigned int responseHigh = (unsigned int) response[3];
    unsigned int responseLow = (unsigned int) response[4];
    return (256 * responseHigh) + responseLow;
  } else {
    while(co2cm1106.available() > 0)  char t = co2cm1106.read();  // Clear serial input buffer;
    return -1; 
  }
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
            return pmSensirionRead();
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

void Sensors::am2320Read() {
    humi1 = am2320.readHumidity();
    temp1 = am2320.readTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) {
        temp = temp1;
        DEBUG("-->[AM2320] read > done!");
    }
}

void Sensors::bme280Read() {
    humi1 = bme.readHumidity();
    temp1 = bme.readTemperature();
    if (humi1 != 0) humi = humi1;
    if (temp1 != 0) {
        temp = temp1;
        DEBUG("-->[BME280] read > done!");
    }
}

void Sensors::aht10Read() {
    humi1 = aht10.readHumidity(); 
    temp1 = aht10.readTemperature();
    if (humi1 != 255) humi = humi1;
    if (temp1 != 255) {
        temp = temp1;
        DEBUG("-->[AHT10] read > done!");
    }
}

void Sensors::sht31Read() {
    humi1 = sht31.readHumidity();
    temp1 = sht31.readTemperature();
    if (!isnan(humi1)) humi = humi1;
    if (!isnan(temp1)) {
        temp = temp1;
        DEBUG("-->[SHT31] read > done!");
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

void Sensors::setDHTparameters (int dht_sensor_pin, int dht_sensor_type) {
    DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
}

void Sensors::dhtRead() {
    if (dhtIsReady(&dhttemp, &dhthumi) == true) {
        temp=dhttemp;
        humi=dhthumi;
        DEBUG("-->[DHTXX] read > done!");
    }
}

void Sensors::onPmSensorError(const char *msg) {
    DEBUG(msg);
    if(_onErrorCb)_onErrorCb(msg);
}

void Sensors::pmSensirionErrtoMess(char *mess, uint8_t r) {
    char buf[80];
    sps30.GetErrDescription(r, buf, 80);
    DEBUG("-->[E][SENSIRION]",buf);
}

void Sensors::pmSensirionErrorloop(char *mess, uint8_t r) {
    if (r) pmSensirionErrtoMess(mess, r);
    else DEBUG(mess);
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
    // set UART for autodetection sensors (Honeywell, Plantower, Panasonic)
    if (pms_type == Auto) {
        DEBUG("-->[PMSENSOR] detecting Generic PM sensor..");
        if(!serialInit(pms_type, 9600, pms_rx, pms_tx))return false;
    }
    else if (pms_type == Panasonic) {
        DEBUG("-->[PMSENSOR] detecting Panasonic PM sensor..");
        if(!serialInit(pms_type, 9600, pms_rx, pms_tx))return false;
    }
    // set UART for autodetection Sensirion sensor
    else if (pms_type == Sensirion) {
        DEBUG("-->[PMSENSOR] detecting Sensirion PM sensor..");
        if(!serialInit(pms_type, 115200, pms_rx, pms_tx))return false;
    }
    else if (pms_type == Mhz19) {
        DEBUG("-->[CO2SENSOR] detecting Mhz19 sensor..");
        if(!serialInit(pms_type, 9600, pms_rx, pms_tx))return false;
    }
    else if (pms_type == CM1106) {
        DEBUG("-->[CO2SENSOR] detecting CM1106 sensor..");
        if(!serialInit(pms_type, 9600, pms_rx, pms_tx))return false;
    }

    // starting auto detection loop 
    int try_sensor_init = 0;
    while (!pmSensorAutoDetect(pms_type) && try_sensor_init++ < 2);

    // get device selected..
    if (device_type >= 0) {
        DEBUG("-->[PMSENSOR] detected: ",device_selected.c_str());
        return true;
    } else {
        DEBUG("-->[E][PMSENSOR] detection failed!");
        if(_onErrorCb)_onErrorCb("-->[E][PMSENSOR] detection failed!");
        return false;
    }
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
        if (pmSensirionInit()) {
            device_selected = "SENSIRION";
            device_type = Sensirion;
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

bool Sensors:: CO2Mhz19Init() {
    myMHZ19.begin(*_serial);                                // *Serial(Stream) refence must be passed to library begin(). 
    myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))
    return true;
}

bool Sensors:: CO2CM1106Init() {
    co2cm1106.begin(9600);
    return true;
}

bool Sensors::pmSensirionInit() {
    // Begin communication channel
    DEBUG("-->[SPS30] starting SPS30 sensor..");
    if(!devmode) sps30.EnableDebugging(0);
    // Begin communication channel;
    if (!sps30.begin(SENSOR_COMMS)){
        pmSensirionErrorloop((char *)"-->[E][SPS30] could not initialize communication channel.", 0);
        return false;
    }
    // check for SPS30 connection
    if (!sps30.probe())
        pmSensirionErrorloop((char *)"-->[E][SPS30] could not probe / connect with SPS30.", 0);
    else {
        DEBUG("-->[SPS30] Detected SPS30.");
        getSensirionDeviceInfo();
    }
    // reset SPS30 connection
    if (!sps30.reset())
        pmSensirionErrorloop((char *)"-->[E][SPS30] could not reset.", 0);

    // start measurement
    if (sps30.start()==true) {
        DEBUG("-->[SPS30] Measurement OK");
        return true;
    } else
        pmSensirionErrorloop((char *)"-->[E][SPS30] Could NOT start measurement", 0);

    if (SENSOR_COMMS == I2C_COMMS) {
        if (sps30.I2C_expect() == 4)
            DEBUG("-->[E][SPS30] Due to I2C buffersize only PM values  \n");
    }
    return false;
}
/**
 * @brief : read and display Sensirion device info
 */
void Sensors::getSensirionDeviceInfo() { 
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == ERR_OK) {
    if(strlen(buf) > 0) DEBUG("-->[SPS30] Serial number : ",buf);
    else DEBUG("not available");
  }
  else
    DEBUG("[SPS30] could not get serial number");

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK)  {
    if(strlen(buf) > 0) DEBUG("-->[SPS30] Product name  : ",buf);
    else DEBUG("not available");
  }
  else
    DEBUG("[SPS30] could not get product name.");

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != ERR_OK) {
    DEBUG("[SPS30] Can not read version info");
    return;
  }
  sprintf(buf,"%d.%d",v.major,v.minor);
  DEBUG("-->[SPS30] Firmware level: ", buf);

  if (SENSOR_COMMS != I2C_COMMS) {
    sprintf(buf,"%d.%d",v.SHDLC_major,v.SHDLC_minor);
    DEBUG("-->[SPS30] Hardware level: ",String(v.HW_version).c_str());
    DEBUG("-->[SPS30] SHDLC protocol: ",buf);
  }

  sprintf(buf, "%d.%d", v.DRV_major, v.DRV_minor);
  DEBUG("-->[SPS30] Library level : ",buf); 
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
    bme.begin(0x76);  // temp/humidity sensor
}

void Sensors::aht10Init() {
    DEBUG("-->[AHT10] starting AHT10 sensor..");
    aht10 = AHT10(AHT10_ADDRESS_0X38);
    aht10.begin();  // temp/humidity sensor
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
            if (pms_rx == 0 || pms_tx== 0) {
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

            else 
            {
#if defined(INCLUDE_SOFTWARE_SERIAL)
                DEBUG("-->[SENSORS] swSerial init on pin: ",String(pms_rx).c_str());
                static SoftwareSerial swSerial(pms_rx, pms_tx);
                if (pms_type == Sensirion)
                    swSerial.begin(speed_baud);
                else if (pms_type == Panasonic)
                    swSerial.begin(speed_baud,SWSERIAL_8E1,pms_rx,pms_tx,false);
                else
                    swSerial.begin(speed_baud,SWSERIAL_8N1,pms_rx,pms_tx,false);
                _serial = &swSerial;
#else
                DEBUG("SoftWareSerial not enabled\n");
                return (false);
#endif  //INCLUDE_SOFTWARE_SERIAL
            }
            break;
    }

    delay(100);
    return true;
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SENSORSHANDLER)
Sensors sensors;
#endif
