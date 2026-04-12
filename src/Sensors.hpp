#ifndef Sensors_hpp
#define Sensors_hpp

#include <Arduino.h>
#include <MHZ19.h>
#include <Wire.h>
#include <cm1106_uart.h>
#include <drivers/PMS5003T.h>
#include <drivers/pm1006.h>
#include <s8_uart.h>
#include <sps30.h>

#include "ISensor.hpp"
#include "sensors/SensorAHT10.hpp"
#include "sensors/SensorAM2320.hpp"
#include "sensors/SensorBME280.hpp"
#include "sensors/SensorBME680.hpp"
#include "sensors/SensorBMP280.hpp"
#include "sensors/SensorDFRobotGas.hpp"
#include "sensors/SensorGCJA5.hpp"
#include "sensors/SensorGeiger.hpp"
#include "sensors/SensorSCD30.hpp"
#include "sensors/SensorSCD4x.hpp"
#include "sensors/SensorSEN5x.hpp"
#include "sensors/SensorSGP41.hpp"
#include "sensors/SensorSHT31.hpp"

#if defined(ARDUINO_ARCH_ESP32) &&                                               \
    (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
     defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32C3_DEV) ||       \
     defined(ARDUINO_ESP32S2_DEV) || defined(ARDUINO_ESP32S3_DEV) ||             \
     defined(ARDUINO_LOLIN_C3_MINI) || defined(ARDUINO_LOLIN_S2_MINI) ||         \
     defined(ARDUINO_LOLIN_S3_MINI) || defined(ESP32C3) || defined(ESP32S2) || defined(ESP32S3))
#define CSL_NOISE_SENSOR_SUPPORTED 1
#endif

#ifdef CSL_NOISE_SENSOR_SUPPORTED
#include "drivers/NoiseSlave.h"
#endif

#ifdef DHT11_ENABLED
#include <dht_nonblocking.h>
#endif

#define CSL_VERSION "0.7.6"
#define CSL_REVISION 385

/***************************************************************
 * D F R o b o t   G r a v i t y   g a s   s e n s o r s
 ***************************************************************/
/**
 * DFRobot Gravity gas sensors — fixed I2C addresses (group 7):
 *   CO  @ 0x78  (SEN0466)
 *   O3  @ 0x79  (SEN0472)
 *   NH3 @ 0x7A  (SEN0469)
 *   NO2 @ 0x7B  (SEN0471)
 *
 * Override any address via build_flags, e.g.:
 *   -D DFROBOT_CO_I2C_ADDR=0x74
 */
#ifndef DFROBOT_CO_I2C_ADDR
#define DFROBOT_CO_I2C_ADDR 0x78
#endif
#ifndef DFROBOT_O3_I2C_ADDR
#define DFROBOT_O3_I2C_ADDR 0x79
#endif
#ifndef DFROBOT_NH3_I2C_ADDR
#define DFROBOT_NH3_I2C_ADDR 0x7A
#endif
#ifndef DFROBOT_NO2_I2C_ADDR
#define DFROBOT_NO2_I2C_ADDR 0x7B
#endif

/***************************************************************
 * S E T U P   E S P 3 2   B O A R D S   A N D   F I E L D S
 ***************************************************************/

// provisional pins. Config those via CLI
#ifdef WEMOSOLED
#define PMS_RX 13  // config for Wemos board & TTGO18650
#define PMS_TX 15  // some old TTGO18650 have PMS_RX 18 & PMS_TX 17
#elif HELTEC
#define PMS_RX 17
#define PMS_TX 18
#elif TTGO_TQ
#define PMS_RX 13
#define PMS_TX 18
#elif M5COREINK
#define PMS_RX 13
#define PMS_TX 14
#elif TTGO_TDISPLAY
#define PMS_RX 13
#define PMS_TX 12
#elif ESP32PICOD4
#define PMS_RX 19
#define PMS_TX 18
#elif ESP32GENERIC
#define PMS_RX RX
#define PMS_TX TX
#elif M5STICKCPLUS
#define PMS_RX 36
#define PMS_TX 0
#elif M5COREINK
#define PMS_RX 13
#define PMS_TX 14
#elif M5ATOM
#define PMS_RX 23
#define PMS_TX 33
#elif M5PICOD4
#define PMS_RX 3
#define PMS_TX 1
#elif AG_OPENAIR
#define PMS_RX 0
#define PMS_TX 1
#define AIRG_SDA 7
#define AIRG_SCL 6
#elif TTGO_T7
#define PMS_RX 17
#define PMS_TX 16
#elif ESP32DEVKIT
#define PMS_RX 17
#define PMS_TX 16
#else
#define PMS_RX -1  // DEFAULTS for other boards. Please setup it via CLI
#define PMS_TX -1
#endif

// I2C pins for M5COREINK and M5STICKCPLUS
#define HAT_I2C_SDA 0
#define HAT_I2C_SCL 26
#define EXT_I2C_SDA 32
#define EXT_I2C_SCL 33

#ifdef M5AIRQ
#define GROVE_SDA 13
#define GROVE_SCL 15
#define I2C1_SDA_PIN 11
#define I2C1_SCL_PIN 12
#endif

#ifdef TTGO_T7S3
#define GROVE_SDA 13
#define GROVE_SCL 14
#define I2C1_SDA_PIN 8
#define I2C1_SCL_PIN 9
#endif

// Read UART sensor retry.
#define SENSOR_RETRY 1000  // Max Serial characters

// UART default port (ESP8266 has no Serial2, use SoftwareSerial via default branch)
#if defined(ARDUINO_ARCH_ESP32)
#define SENSOR_COMMS SERIALPORT2
#else
#define SENSOR_COMMS 3  // Custom value: use default (SoftwareSerial) path on ESP8266
#endif

#include "SensorTypes.hpp"

typedef void (*errorCbFn)(const char *msg);
typedef void (*voidCbFn)();

/**
 * @brief CanAirIO Sensors Manager main class.
 * @authors \@hpsaturn and CanAir.IO contributers
 */
class Sensors {
 public:
  /// SPS30 values. Only for Sensirion SPS30 sensor.
  struct sps_values val;

  /// Debug mode for increase verbose.
  bool devmode;

  /// Initial sample time for all sensors
  int sample_time = 10;

  /// Temperature offset (for final temp output)
  float toffset = 0.0;

  /// Altitude compensation variable
  float altoffset = 0.0;

  /// Sea level pressure (hPa)
  float sealevel = 1013.25;

  /// Altitude hpa calculation
  float hpa = 0.0;

  /// Sensirion dust SPS30 library
  SPS30 sps30;

  /// only detect i2c sensors flag
  bool i2conly;

  /*****************************************
   * Modular I2C sensors:
   ****************************************/
  SensorAM2320 *_sensorAM2320 = nullptr;
  SensorBME280 *_sensorBME280 = nullptr;
  SensorBMP280 *_sensorBMP280 = nullptr;
  SensorBME680 *_sensorBME680 = nullptr;
  SensorAHT10 *_sensorAHT10 = nullptr;
  SensorSHT31 *_sensorSHT31 = nullptr;

#ifdef DHT11_ENABLED
  /// @deprecated DHT sensor variable
  float dhthumi, dhttemp;
#endif
  /// UART sensors (not yet modularized — shared serial autodetect)
  MHZ19 mhz19;
  CM1106_UART *cm1106;
  CM1106_sensor cm1106sensor;
  CM1106_ABC abc;
  S8_UART *s8;
  S8_sensor s8sensor;
  PM1006 *pm1006;
  PMS5003T *pm5003t;

  /// Modular I2C CO2 / gas / PM sensors
  SensorSCD30 *_sensorSCD30 = nullptr;
  SensorSCD4x *_sensorSCD4x = nullptr;
  SensorSGP41 *_sensorSGP41 = nullptr;
  SensorGCJA5 *_sensorGCJA5 = nullptr;
  SensorSEN5x *_sensorSEN5x = nullptr;
  SensorDFRobotGas *_sensorDFRCO = nullptr;
  SensorDFRobotGas *_sensorDFRNH3 = nullptr;
  SensorDFRobotGas *_sensorDFRNO2 = nullptr;
  SensorDFRobotGas *_sensorDFRO3 = nullptr;
  SensorGeiger *_sensorGeiger = nullptr;

  Sensors();
  ~Sensors();

  void init(u_int pms_type = 0, int pms_rx = PMS_RX, int pms_tx = PMS_TX);

  void loop();

  bool readAllSensors();

  bool isDataReady();

  void setSampleTime(int seconds);

  void setOnDataCallBack(voidCbFn cb);

  void setOnErrorCallBack(errorCbFn cb);

  void setTemperatureUnit(TEMPUNIT tunit);

  void setDebugMode(bool enable);

  bool isUARTSensorConfigured() const;

  int getUARTDeviceTypeSelected() const;

  uint16_t getPM1() const;

  uint16_t getPM25() const;

  uint16_t getPM4() const;

  uint16_t getPM10() const;

  uint16_t getCO2() const;

  float getCO2humi() const;

  float getCO2temp() const;

  float getTemperature() const;

  float getHumidity() const;

  float getPressure() const;

  float getAltitude() const;

  float getGas() const;

  float getNH3() const;

  float getCO() const;

  float getNO2() const;

  float getO3() const;

  void enableGeigerSensor(int gpio);

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  float getNoise() const;

  float getNoiseAverage() const;

  float getNoisePeak() const;

  float getNoiseMin() const;

  float getNoiseLegalAverage() const;

  float getNoiseLegalMaximum() const;
  float getNoiseLd() const;
  float getNoiseLe() const;
  float getNoiseLn() const;
  float getNoiseLden() const;
  bool sendNoiseSensorTime(uint32_t unixTime);
  bool syncNoiseSensorTime();
  void setNoiseSensorTimeSyncInterval(uint32_t intervalMs);
#endif

  uint32_t getGeigerCPM(void) const;

  float getGeigerMicroSievertHour(void) const;

  void initTOffset(float offset);

  float getTOffset() const;

  void setTempOffset(float offset);

  float getTempOffset() const;

  void setCO2AltitudeOffset(float altitude);

  void setSeaLevelPressure(float hpa);

  void setCO2RecalibrationFactor(int ppmValue);

  void detectI2COnly(bool enable);

  String getLibraryVersion() const;

  int16_t getLibraryRevision() const;

  bool isSensorRegistered(SENSORS sensor) const;

  uint8_t *getSensorsRegistered();

  uint8_t getSensorsRegisteredCount() const;

  String getSensorName(SENSORS sensor) const;

  SensorGroup getSensorGroup(SENSORS sensor) const;

  uint8_t getUnitsRegisteredCount() const;

  bool isUnitRegistered(UNIT unit) const;

  String getUnitName(UNIT unit) const;

  String getUnitSymbol(UNIT unit) const;

  UNIT getNextUnit();

  void resetUnitsRegister();

  void resetSensorsRegister();

  void resetNextUnit();

  void resetAllVariables();

  float getUnitValue(UNIT unit);

  void printUnitsRegistered(bool debug = false);

  void printSensorsRegistered(bool debug = false);

  void startI2C();

 private:
  ISensor *_active_sensors[SCOUNT] = {nullptr};
  uint8_t _active_sensors_count = 0;
  void registerSensor(ISensor *sensor);

#ifdef DHT11_ENABLED
  /// DHT library
  uint32_t delayMS;
#endif
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
  float voci = 0.0;
  float noxi = 0.0;
  uint16_t voc = 0;
  uint16_t nox = 0;

  // temperature unit (C,K,F)
  TEMPUNIT temp_unit = TEMPUNIT::CELSIUS;

  uint16_t CO2Val;      // CO2 in ppm
  float CO2humi = 0.0;  // humidity of CO2 sensor
  float CO2temp = 0.0;  // temperature of CO2 sensor

  float nh3;  // Amonium in ppm
  float co;   // Carbon monoxide in ppm
  float no2;  // Nitrogen dioxide in ppm
  float o3;   // Ozone in ppm

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  TwoWire *noiseWire = nullptr;
  SensorData noiseSensorData{};
  bool noiseWireReady = false;
  uint8_t noiseSensorAddress = 0;
#if __cplusplus >= 201103L
  static_assert(sizeof(SensorData) == 68, "SensorData size mismatch");
#endif
#endif
  bool noiseSensorEnabled = false;
  float noiseInstant = 0.0;
  float noiseAvgValue = 0.0;
  float noisePeakValue = 0.0;
  float noiseMinValue = 0.0;
  float noiseAvgLegalValue = 0.0;
  float noiseAvgLegalMaxValue = 0.0;
  float noiseLdValue = 0.0;
  float noiseLeValue = 0.0;
  float noiseLnValue = 0.0;
  float noiseLdenValue = 0.0;
  bool noiseScanDone = false;
  uint32_t noiseLastScanMs = 0;
  uint32_t noiseScanRetryMs = 5000;
  uint32_t noiseLastTimeSyncMs = 0;
  uint32_t noiseTimeSyncIntervalMs = 86400000;
  bool noiseTimeSyncEnabled = true;

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
  float getSCD30TempOffset() const;
  void setSCD30AltitudeOffset(float offset);
  void CO2correctionAlt();
  float hpaCalculation(float altitude);

  void CO2scd4xInit();
  void CO2scd4xRead();
  void setSCD4xTempOffset(float offset);
  float getSCD4xTempOffset() const;
  void setSCD4xAltitudeOffset(float offset);

  void sen5xInit();
  void sen5xRead();
  void setsen5xTempOffset(float offset);

  void sgp41Init();
  void sgp41Read();

  void GCJA5Init();
  void GCJA5Read();

#ifdef DHT11_ENABLED
  void dhtInit();
  void dhtRead();
  bool dhtIsReady(float *temperature, float *humidity);
#endif

  void DFRobotNH3Init();
  void DFRobotNH3Read();
  void DFRobotCOInit();
  void DFRobotCORead();
  void DFRobotNO2Init();
  void DFRobotNO2Read();
  void DFRobotO3Init();
  void DFRobotO3Read();

  bool dfrHasExternalTempSensor() const;

  // UART sensors methods:

  bool sensorSerialInit(u_int pms_type, int rx, int tx);
  bool pmSensorAutoDetect(u_int pms_type);
  bool pmSensorRead();
  bool pmGenericRead();
  bool pmGCJA5Read();
  bool pmSDS011Read();
  bool pm1006Read();
  bool pm5003TRead();
  bool CO2Mhz19Read();
  bool CO2CM1106Read();
  bool CO2Mhz19Init();
  bool CO2CM1106Init();
  bool senseAirS8Init();
  bool senseAirS8Read();
  bool PM1006Init();
  bool PM5003TInit();

  bool sps30I2CInit();
  bool sps30UARTInit();
  bool sps30Read();
  bool sps30tests();
  void sps30ErrToMess(char *mess, uint8_t r);
  void sps30Errorloop(char *mess, uint8_t r);
  void sps30DeviceInfo();

  void geigerRead();

  void onSensorError(const char *msg);

  void enableWire1();

  void disableWire1();

  bool serialInit(u_int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx);

  String hwSerialRead(unsigned int length_buffer);

  void restart();  // restart serial (it isn't works sometimes)

  void DEBUG(const char *text, const char *textb = "") const;

  void printValues();

  void printHumTemp();

  void tempRegister(bool isCO2temp);

  void sensorRegister(SENSORS sensor);

  void sensorAnnounce(SENSORS sensor);

  void unitRegister(UNIT unit);

  uint8_t *getUnitsRegistered();

#ifdef CSL_NOISE_SENSOR_SUPPORTED
  bool noiseSensorAutoDetect();
  void noiseSensorService();
  void noiseSensorCollect();
  bool noiseSensorReadStatus(TwoWire &wire, uint8_t address, uint8_t &status);
  bool noiseSensorReadData(TwoWire &wire, uint8_t address, SensorData &out);
  bool noiseSensorDevicePresent(TwoWire &wire, uint8_t address);
  void noiseSensorInitWire();
#endif

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
