#ifndef Sensors_hpp
#define Sensors_hpp

#include <AHT10.h>
#include <Adafruit_AM2320.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <MHZ19.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h>
#include <dht_nonblocking.h>
#include <sps30.h>
#include <cm1106_uart.h>
#include <s8.h>

using namespace std;
#include <vector>

/******************************************************************************
* S E T U P  ESP32 B O A R D S  A N D  F I E L D S
* -------------------------------------------
* Please select the board on platformio.ini file or pass it via build flag
******************************************************************************/
#ifdef WEMOSOLED
#define PMS_RX 13           // config for Wemos board & TTGO18650
#define PMS_TX 15           // some old TTGO18650 have PMS_RX 18 & PMS_TX 17
#define DHT_SENSOR_PIN 23  
#elif HELTEC
#define PMS_RX 17           // config for Heltec board, ESP32Sboard & ESPDUINO-32. Use Uart2
#define PMS_TX 18           // some old ESP32Sboard have PMS_RX 27 & PMS_TX 25. Jump Uart2 tx from 16 to 18. !6 used by Oled.
#define DHT_SENSOR_PIN 23    
#elif TTGO_TQ
#define PMS_RX 13  
#define PMS_TX 18
#define DHT_SENSOR_PIN 23    
#elif M5COREINK
#define PMS_RX 13           // config for backward header in M5CoreInk
#define PMS_TX 14
#define DHT_SENSOR_PIN 25
#elif TTGO_TDISPLAY
#define PMS_RX 13  
#define PMS_TX 12
#define DHT_SENSOR_PIN 17
#else                       // ** DEFAULT **
#define PMS_RX 17           // config for D1MIN1 / TTGO T7 / Default for main ESP32 dev boards
#define PMS_TX 16
#define DHT_SENSOR_PIN 23      
#endif

// DHT sensor type
#define DHT_SENSOR_TYPE DHT_TYPE_22  

// Read UART sensor retry. 
#define SENSOR_RETRY 1000         // Max Serial characters

// Sensirion SPS30 sensor
#define SENSOR_COMMS SERIALPORT2  // UART OR I2C

//H&T definitions
#define SEALEVELPRESSURE_HPA (1013.25)


typedef void (*errorCbFn)(const char *msg);
typedef void (*voidCbFn)();

class Sensors {
   public:
    /// Supported devices. Auto is for Honeywell and Plantower sensors and similars
    enum SENSOR_TYPE { Auto, Panasonic, Sensirion, SDS011, Mhz19, CM1106, SENSEAIRS8 };
    
    /// SPS30 values. Only for Sensirion SPS30 sensor.
    struct sps_values val;

    /// Debug mode for increase verbose.
    bool devmode;

    /// Initial sample time for all sensors
    int sample_time = 5;

    // temperature offset (for final temp output)
    float toffset = 0.0;  

    /// Sensirion library
    SPS30 sps30;

    // only detect i2c sensors
    bool i2conly;

    /*****************************************
     * I2C sensors:
     ****************************************/

    // AM2320 (Humidity and temperature)
    Adafruit_AM2320 am2320;
    // BME280 (Humidity, Pressure, Altitude and Temperature)
    Adafruit_BME280 bme280;
    // BME680 (Humidity, Gas, IAQ, Pressure, Altitude and Temperature)
    Adafruit_BME680 bme680; 
    // AHT10
    AHT10 aht10;
    // SHT31
    Adafruit_SHT31 sht31;
    // DHT sensor
    float dhthumi, dhttemp;
    // Mhz19 sensor
    MHZ19 mhz19;
    // CM1106
    // it using the auto detected hardware serial;
    // SCD30 sensor
    SCD30 scd30;
    // CM1106 UART
    CM1106_UART *cm1106;
    CM1106_sensor cm1106sensor;
    CM1106_ABC abc;
    // Panasonic SN-GCJA5
    SFE_PARTICLE_SENSOR pmGCJA5;
    // SenseAir S8 CO2 sensor
    S8 *sensor_S8;
    S8_sensor s8sensor;

    void init(int pms_type = 0, int pms_rx = PMS_RX, int pms_tx = PMS_TX);
    void loop();
    bool isDataReady();
    bool isPmSensorConfigured();
    void setSampleTime(int seconds);
    void setOnDataCallBack(voidCbFn cb);
    void setOnErrorCallBack(errorCbFn cb);
    void setDebugMode(bool enable);
    void setDHTparameters(int dht_sensor_pin = DHT_SENSOR_PIN, int dht_sensor_type = DHT_SENSOR_TYPE);
    int getPmDeviceTypeSelected();
    String getPmDeviceSelected();

    uint16_t getPM1();
    uint16_t getPM25();
    uint16_t getPM4();
    uint16_t getPM10();
    uint16_t getCO2();

    float getCO2humi();
    float getCO2temp();

    float getTemperature();
    float getHumidity();
    float getPressure();
    float getAltitude();
    float getGas();

    void setTempOffset(float offset);

    String getFormatTemp();
    String getFormatPress();
    String getFormatHum();
    String getFormatGas();
    String getFormatAlt();

    String getStringPM1();
    String getStringPM25();
    String getStringPM4();
    String getStringPM10();
    String getStringCO2();
    String getStringCO2temp(); 

    void setCO2RecalibrationFactor(int ppmValue);

    void detectI2COnly(bool enable);

   private:
    /// DHT library
    uint32_t delayMS;
    /// For UART sensors (autodetected available serial)
    Stream *_serial;
    /// Callback on some sensors error.
    errorCbFn _onErrorCb = nullptr;
    /// Callback when sensor data is ready.
    voidCbFn _onDataCb = nullptr;

    String device_selected;
    int device_type = -1;
    bool dataReady;
    
    uint16_t pm1;   // PM1
    uint16_t pm25;  // PM2.5
    uint16_t pm4;   // PM4
    uint16_t pm10;  // PM10

    float humi = 0.0;   // % Relative humidity
    float temp = 0.0;   // Temperature (°C)
    float pres = 0.0;   // Pressure
    float alt = 0.0;
    float gas = 0.0;
    
    uint16_t CO2;         // CO2 in ppm
    float CO2humi = 0.0;  // temperature of the CO2 sensor
    float CO2temp = 0.0;  // temperature of the CO2 sensor


    void am2320Init();
    void am2320Read();

    void bme280Init();
    void bme280Read();

    void bme680Init();
    void bme680Read();

    void aht10Init();
    void aht10Read();

    void sht31Init();
    void sht31Read();

    void CO2scd30Init();
    void CO2scd30Read();
    void setSCD30TempOffset(float offset);

    void PMGCJA5Init();
    void PMGCJA5Read();

    void dhtInit();
    void dhtRead();
    bool dhtIsReady(float *temperature, float *humidity);

    // UART sensors methods:

    bool sensorSerialInit(int pms_type, int rx, int tx);
    bool pmSensorAutoDetect(int pms_type);
    bool pmSensorRead();
    bool pmGenericRead();
    bool pmPanasonicRead();
    
    bool pmSDS011Read();
    bool CO2Mhz19Read();
    bool CO2CM1106Read();
    int CO2CM1106val();
    bool CO2Mhz19Init();
    bool CO2CM1106Init();
    bool senseAirS8Init();
    bool senseAirS8Read();

    bool sps30I2CInit();
    bool sps30UARTInit();
    bool sps30Read();
    bool sps30tests();
    void sps30ErrToMess(char *mess, uint8_t r);
    void sps30Errorloop(char *mess, uint8_t r);
    void sps30DeviceInfo();

    void onSensorError(const char *msg);

    bool serialInit(int pms_type, long speed_baud, int pms_rx, int pms_tx);
    String hwSerialRead(unsigned int lenght_buffer);
    void restart();  // restart serial (it isn't works sometimes)
    void DEBUG(const char *text, const char *textb = "");

    void printValues();

// @todo use DEBUG_ESP_PORT ?
#ifdef WM_DEBUG_PORT
    Stream &_debugPort = WM_DEBUG_PORT;
#else
    Stream &_debugPort = Serial;  // debug output stream ref
#endif
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SENSORSHANDLER)
extern Sensors sensors;
#endif

#endif
