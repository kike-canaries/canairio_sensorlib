#ifndef Sensors_hpp
#define Sensors_hpp

#include <AHT10.h>
#include <AM232X.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <MHZ19.h>
#include <SensirionI2CScd4x.h>
#include <SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <cm1106_uart.h>
#include <dht_nonblocking.h>
#include <s8_uart.h>
#include <sps30.h>

#define CSL_VERSION "0.5.2"
#define CSL_REVISION 355

/***************************************************************
* S E T U P   E S P 3 2   B O A R D S   A N D   F I E L D S
***************************************************************/

#ifdef WEMOSOLED
#define PMS_RX 13          // config for Wemos board & TTGO18650
#define PMS_TX 15          // some old TTGO18650 have PMS_RX 18 & PMS_TX 17
#define DHT_SENSOR_PIN 23  // default DHT sensor pin
#elif HELTEC
#define PMS_RX 17  // config for Heltec board, ESP32Sboard & ESPDUINO-32. Use Uart2
#define PMS_TX 18  // some old ESP32Sboard have PMS_RX 27 & PMS_TX 25. Jump Uart2 tx from 16 to 18. !6 used by Oled.
#define DHT_SENSOR_PIN 23
#elif TTGO_TQ
#define PMS_RX 13
#define PMS_TX 18
#define DHT_SENSOR_PIN 23
#elif M5COREINK
#define PMS_RX 13  // config for backward header in M5CoreInk
#define PMS_TX 14
#define DHT_SENSOR_PIN 25
#elif TTGO_TDISPLAY
#define PMS_RX 13
#define PMS_TX 12
#define DHT_SENSOR_PIN 17
#elif ESP32PICOD4
#define PMS_RX 19
#define PMS_TX 18
#define DHT_SENSOR_PIN 12
#elif ESP32GENERIC
#define PMS_RX RX
#define PMS_TX TX
#define DHT_SENSOR_PIN 12
#elif M5STICKCPLUS
#define PMS_RX 17
#define PMS_TX 16
#define DHT_SENSOR_PIN 34
#elif M5COREINK
#define PMS_RX 13
#define PMS_TX 14
#define DHT_SENSOR_PIN 34
#elif M5ATOM
#define PMS_RX 23
#define PMS_TX 33
#define DHT_SENSOR_PIN 19
#else              // **DEFAULT** for legacy CanAirIO devices:
#define PMS_RX 17  // D1MIN1 / TTGOT7 / ESP32DEVKIT, also for main ESP32 dev boards use it
#define PMS_TX 16
#define DHT_SENSOR_PIN 23  // default DHT sensor pin
#endif

// DHT sensor type
#define DHT_SENSOR_TYPE DHT_TYPE_22

// Read UART sensor retry.
#define SENSOR_RETRY 1000  // Max Serial characters

// Sensirion SPS30 sensor
#define SENSOR_COMMS SERIALPORT2  // UART OR I2C 

#define SENSOR_UNITS           \
    X(NUNIT, "NUNIT", "NUNIT") \
    X(PM1, "ug/m3", "PM1")     \
    X(PM25, "ug/m3", "PM2.5")  \
    X(PM4, "ug/m3", "PM4")     \
    X(PM10, "ug/m3", "PM10")   \
    X(TEMP, "C", "T")          \
    X(HUM, "%", "H")           \
    X(CO2, "ppm", "CO2")       \
    X(CO2TEMP, "C", "CO2T")    \
    X(CO2HUM, "%", "CO2H")     \
    X(PRESS, "hPa", "P")       \
    X(ALT, "m", "Alt")         \
    X(GAS, "Ohm", "Gas")       \
    X(UCOUNT, "COUNT", "UCOUNT")

#define X(unit, symbol, name) unit,
typedef enum UNIT : size_t { SENSOR_UNITS } UNIT;
#undef X

#define SENSORS_TYPES       \
    X(Auto, "GENERIC", 1)   \
    X(SGCJA5, "GCJA5", 1)   \
    X(SSPS30, "SPS30", 1)   \
    X(SDS011, "SDS011", 1)  \
    X(SMHZ19, "MHZ19", 2)   \
    X(SCM1106, "CM1106", 2) \
    X(SAIRS8, "SAIRS8", 2)  \
    X(SSCD30, "SCD30", 2)   \
    X(SSCD4X, "SCD4X", 2)   \
    X(SSHT31, "SHT31", 3)   \
    X(SBME280, "BME280", 3) \
    X(SBMP280, "BMP280", 3) \
    X(SBME680, "BME680", 3) \
    X(SAHT10, "AHT10", 3)   \
    X(SAM232X, "AM232X", 3) \
    X(SDHTX, "DHTX", 3)     \
    X(SCOUNT, "SCOUNT", 3)

#define X(utype, uname, umaintype) utype,
typedef enum SENSORS : size_t { SENSORS_TYPES } SENSORS;  // backward compatibility
#undef X

// MAIN SENSOR TYPE
enum class SensorGroup { SENSOR_NONE,
                         SENSOR_PM,
                         SENSOR_CO2,
                         SENSOR_ENV };

typedef void (*errorCbFn)(const char *msg);
typedef void (*voidCbFn)();

class Sensors {
   public:
    // SPS30 values. Only for Sensirion SPS30 sensor.
    struct sps_values val;

    // Debug mode for increase verbose.
    bool devmode;

    // Initial sample time for all sensors
    int sample_time = 5;

    // temperature offset (for final temp output)
    float toffset = 0.0;

    // Altitud compensation variable
    float altoffset = 0.0;

    // Sea level pressure (hPa)
    float sealevel = 1013.25;

    // Altitud hpa calculation
    float hpa = 0.0;

    /// Sensirion library
    SPS30 sps30;

    // only detect i2c sensors
    bool i2conly;

    /*****************************************
     * I2C sensors:
     ****************************************/

    // AM2320 (Humidity and temperature)
    AM232X am2320;
    // BME280 (Humidity, Pressure, Altitude and Temperature)
    Adafruit_BME280 bme280;
    // BMP280 (Humidity, Pressure, Altitude and Temperature)
    Adafruit_BMP280 bmp280;
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
    // SCD30 sensor
    SCD30 scd30;
    // CM1106 UART
    CM1106_UART *cm1106;

    CM1106_sensor cm1106sensor;

    CM1106_ABC abc;
    // Panasonic SN-GCJA5
    SFE_PARTICLE_SENSOR pmGCJA5;
    // SenseAir S8 CO2 sensor
    S8_UART *s8;

    S8_sensor s8sensor;
    // SCD4x sensor
    SensirionI2CScd4x scd4x;

    void init(int pms_type = 0, int pms_rx = PMS_RX, int pms_tx = PMS_TX);

    void loop();

    bool readAllSensors();

    bool isDataReady();

    void setSampleTime(int seconds);

    void setOnDataCallBack(voidCbFn cb);

    void setOnErrorCallBack(errorCbFn cb);

    void setDebugMode(bool enable);

    void setDHTparameters(int dht_sensor_pin = DHT_SENSOR_PIN, int dht_sensor_type = DHT_SENSOR_TYPE);

    bool isUARTSensorConfigured();

    int getUARTDeviceTypeSelected();

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

    void setCO2AltitudeOffset(float altitude);

    void setSeaLevelPressure(float hpa);

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

    String getLibraryVersion();

    int16_t getLibraryRevision();

    bool isSensorRegistered(SENSORS sensor);

    uint8_t *getSensorsRegistered();

    uint8_t getSensorsRegisteredCount();

    String getSensorName(SENSORS sensor);

    SensorGroup getSensorGroup(SENSORS sensor);

    uint8_t getUnitsRegisteredCount();

    bool isUnitRegistered(UNIT unit);

    String getUnitName(UNIT unit);

    String getUnitSymbol(UNIT unit);

    UNIT getNextUnit();

    void resetUnitsRegister();

    void resetSensorsRegister();

    void resetNextUnit();

    void resetAllVariables();

    float getUnitValue(UNIT unit);

    void printUnitsRegistered(bool debug = false);

    void printSensorsRegistered(bool debug = false);

   private:
    /// DHT library
    uint32_t delayMS;
    /// For UART sensors (autodetected available serial)
    Stream *_serial;
    /// Callback on some sensors error.
    errorCbFn _onErrorCb = nullptr;
    /// Callback when sensor data is ready.
    voidCbFn _onDataCb = nullptr;

    int dev_uart_type = -1;

    bool dataReady;

    bool readAllComplete = false;

    uint8_t sensors_registered_count;

    uint8_t units_registered_count;

    uint8_t current_unit = 0;

    uint16_t pm1;   // PM1
    uint16_t pm25;  // PM2.5
    uint16_t pm4;   // PM4
    uint16_t pm10;  // PM10

    float humi = 0.0;  // % Relative humidity
    float temp = 0.0;  // Temperature (°C)
    float pres = 0.0;  // Pressure
    float alt = 0.0;
    float gas = 0.0;

    uint16_t CO2Val;      // CO2 in ppm
    float CO2humi = 0.0;  // humidity of CO2 sensor
    float CO2temp = 0.0;  // temperature of CO2 sensor

    void am2320Init();
    void am2320Read();

    void bme280Init();
    void bme280Read();

    void bmp280Init();
    void bmp280Read();

    void bme680Init();
    void bme680Read();

    void aht10Init();
    void aht10Read();

    void sht31Init();
    void sht31Read();

    void CO2scd30Init();
    void CO2scd30Read();
    void setSCD30TempOffset(float offset);
    void setSCD30AltitudeOffset(float offset);
    void CO2correctionAlt();
    float hpaCalculation(float altitude);

    void CO2scd4xInit();
    void CO2scd4xRead();
    void setSCD4xTempOffset(float offset);
    void setSCD4xAltitudeOffset(float offset);

    void GCJA5Init();
    void GCJA5Read();

    void dhtInit();
    void dhtRead();
    bool dhtIsReady(float *temperature, float *humidity);

    // UART sensors methods:

    bool sensorSerialInit(int pms_type, int rx, int tx);
    bool pmSensorAutoDetect(int pms_type);
    bool pmSensorRead();
    bool pmGenericRead();
    bool pmGCJA5Read();
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

    void startI2C();

    void enableWire1();

    void disableWire1();

    bool serialInit(int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx);

    String hwSerialRead(unsigned int lenght_buffer);

    void restart();  // restart serial (it isn't works sometimes)

    void DEBUG(const char *text, const char *textb = "");

    void printValues();

    void sensorRegister(SENSORS sensor);

    void sensorAnnounce(SENSORS sensor);

    void unitRegister(UNIT unit);

    uint8_t *getUnitsRegistered();

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
