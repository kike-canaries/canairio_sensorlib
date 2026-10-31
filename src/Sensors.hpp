#ifndef Sensors_hpp
#define Sensors_hpp

#include <AHTxx.h>
#include <AM232X.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <DFRobot_MultiGasSensor.h>
#include <MHZ19.h>
#include <NOxGasIndexAlgorithm.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSen5x.h>
#include <SensirionI2CSgp41.h>
#include <SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h>
#include <VOCGasIndexAlgorithm.h>
#include <Wire.h>
#include <cm1106_uart.h>
#include <drivers/NoiseSlave.h>
#include <drivers/PMS5003T.h>
#include <drivers/geiger.h>
#include <drivers/pm1006.h>
#include <s8_uart.h>
#include <sps30.h>
#include <time.h>

#ifdef DHT11_ENABLED
#include <dht_nonblocking.h>
#endif

#define CSL_VERSION "0.7.7"
#define CSL_REVISION 386
#include "SensorsTypes.hpp"
#include "drivers/GeigerDriver.h"
#include "drivers/NoiseSensorDriver.h"
#include "drivers/co2/Co2Drivers.h"
#include "drivers/env/EnvDrivers.h"
#include "drivers/gas/GasDrivers.h"
#include "drivers/pm/PmDrivers.h"
#include "drivers/uart/UartDriver.h"

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

  /// Sensirion sgp41 library (Rh, T, Voc, Nox)
  SensirionI2CSgp41 sgp41;
  uint8_t conditioning_s = 10;
  /// Sensirion gas index algorithms for SGP41 raw signal processing
  VOCGasIndexAlgorithm vocAlgorithm{1.0f};
  NOxGasIndexAlgorithm noxAlgorithm;

  /// only detect i2c sensors flag
  bool i2conly;

  /*****************************************
   * I2C sensors:
   ****************************************/

  /// AM2320 object (Humidity and temperature)
  AM232X am2320;
  /// BME280 object (Humidity, Pressure, Altitude and Temperature)
  Adafruit_BME280 bme280;
  /// BMP280 object (Humidity, Pressure, Altitude and Temperature)
  Adafruit_BMP280 bmp280;
  /// BME680 object (Humidity, Gas, IAQ, Pressure, Altitude and Temperature)
  Adafruit_BME680 bme680;
  /// AHTXX sensors object
  AHTxx aht10;
  /// SHT31 object (Humidity and temperature)
  Adafruit_SHT31 sht31;

#ifdef DHT11_ENABLED
  /// @deprecated DHT sensor variable
  float dhthumi, dhttemp;
#endif
  /// Mhz19 object sensor
  MHZ19 mhz19;
  /// SCD30 object sensor
  Adafruit_SCD30 scd30;
  /// CM1106 UART main object sensor
  CM1106_UART *cm1106;
  /// CM1106 UART main variable
  CM1106_sensor cm1106sensor;
  /// CM1106 UART calibration object
  CM1106_ABC abc;
  /// Panasonic SN-GCJA5 object sensor
  SFE_PARTICLE_SENSOR pmGCJA5;
  /// SenseAir S8 CO2 object sensor
  S8_UART *s8;
  /// SenseAir S8 CO2 object sensor
  S8_sensor s8sensor;
  /// SCD4x object sensor
  SensirionI2CScd4x scd4x;
  // SEN5x sensor PM
  SensirionI2CSen5x sen5x;
  /// IKEA Vindriktn object sensor
  PM1006 *pm1006;
  /// DFRobot gravity CO object sensor addr 0x78
  DFRobot_GAS_I2C dfrCO;
  /// DFRobot gravity NH3 object sensor addr 0x7A
  DFRobot_GAS_I2C dfrNH3;
  /// DFRobot gravity NO2 object sensor add 0x7B
  DFRobot_GAS_I2C dfrNO2;
  /// DFRobot gravity O3 object sensor add 0x79
  DFRobot_GAS_I2C dfrO3;
  /// Geiger CAJOE object sensor
  GEIGER *rad;
  /// PMS5003T Plantower with T&H of Airgradient
  PMS5003T *pm5003t;

  Sensors();
  ~Sensors();

  // Drivers hold a reference to this facade; copying it would leave dangling
  // references in the copy. The manager is intended as a singleton.
  Sensors(const Sensors &) = delete;
  Sensors &operator=(const Sensors &) = delete;

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

  float getVOC() const;

  float getNOX() const;

  float getVOCI() const;

  float getNOXI() const;

  void enableGeigerSensor(int gpio);

  // Noise sensor (I2C slave node). Every level is in dB(A) unless noted.
  /** LAeq,1s: level of the last second published by the node. */
  float getNoise() const;
  /** LAFmax: maximum Fast-weighted level of the last second. Not a true peak. */
  float getNoisePeak() const;
  /** L10, over the node's sliding window (300 s by default). */
  float getNoiseLegalAverage() const;
  /** L90, over the node's sliding window. */
  float getNoiseL90() const;
  /**
   * LASmax (Slow weighting): the maximum since the previous reading, so no
   * event is lost whatever the sample time. 0 on nodes older than 3.3.1.
   */
  float getNoiseLASmax() const;
  /**
   * LCpeak in dB(C): true instantaneous peak since the previous reading, the
   * indicator for impulsive noise. 0 on nodes older than 3.3.1.
   */
  float getNoiseLCpeak() const;
  float getNoiseLd() const;
  float getNoiseLe() const;
  float getNoiseLn() const;
  float getNoiseLden() const;

  /**
   * Sends the node its clock. The node applies no time zone: unixTime must be
   * LOCAL epoch (UTC + offset), because the day/evening/night bands are read
   * straight off it. syncNoiseSensorTime() already does that conversion.
   */
  bool sendNoiseSensorTime(uint32_t unixTime);
  /**
   * Sends the clock now if due: the first time, every sync interval, when the
   * local UTC offset changes (daylight saving) and at once when the node
   * reports it lost its clock. Cheap to call from loop(): the clock is only
   * examined once a minute once the node is synced.
   *
   * The local time is UTC plus the offset of the SYSTEM time zone (TZ), so the
   * application must set a proper one, with daylight saving if it applies, for
   * example setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); tzset(). With no TZ
   * the node gets UTC, and every Ld/Le/Ln/Lden is shifted by the zone offset.
   */
  bool syncNoiseSensorTime();
  void setNoiseSensorTimeSyncInterval(uint32_t intervalMs);

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
#ifdef DHT11_ENABLED
  /// DHT library
  uint32_t delayMS;
#endif
  /// For UART sensors (autodetected available serial)
  Stream *_serial;
  /// Second UART stream (e.g. SenseAir S8 on a dedicated UART)
  Stream *_serial2;
  /// Callback on some sensors error.
  errorCbFn _onErrorCb = nullptr;
  /// Callback when sensor data is ready.
  voidCbFn _onDataCb = nullptr;

  int dev_uart_type = -1;
  /// True when the SenseAir S8 is attached to the second UART
  bool s8_uart2 = false;

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

  TwoWire *noiseWire = nullptr;
  SensorData noiseSensorData{};
  bool noiseWireReady = false;
  uint8_t noiseSensorAddress = 0;

  bool noiseSensorEnabled = false;
  float noiseInstant = 0.0;        // LAeq,1s
  float noisePeakValue = 0.0;      // LAFmax
  float noiseAvgLegalValue = 0.0;  // L10
  float noiseL90Value = 0.0;
  float noiseLASmaxValue = 0.0;
  float noiseLCpeakValue = 0.0;
  float noiseLdValue = 0.0;
  float noiseLeValue = 0.0;
  float noiseLnValue = 0.0;
  float noiseLdenValue = 0.0;
  NodeMetadata noiseMeta{};
  uint32_t noiseFwVer = 0;  // 0 = unknown / node without metadata (legacy)
  bool noiseHasMeta = false;
  uint32_t noiseLastCycles = 0;
  bool noiseHaveCycles = false;
  int32_t noiseLastSentOffset = 0;  // UTC offset of the last clock sent to the node
  uint32_t noiseLastZoneCheckMs = 0;
  static constexpr uint32_t NOISE_ZONE_CHECK_MS = 60000;
  bool noiseScanDone = false;
  uint32_t noiseLastTimeSyncMs = 0;
  uint32_t noiseLastSyncAttemptMs = 0;
  uint32_t noiseTimeSyncIntervalMs = 86400000;
  static constexpr uint32_t NOISE_SYNC_RETRY_MS = 30000;
  bool noiseTimeSyncEnabled = true;

  void CO2correctionAlt();
  float hpaCalculation(float altitude);

  void onSensorError(const char *msg);

  void enableWire1();

  void disableWire1();

  void restart();  // restart serial (it isn't works sometimes)

  void DEBUG(const char *text, const char *textb = "") const;

  void printValues();

  void printHumTemp();

  void printI2C(const char *name, const char *i2cname, int pin_sda, int pin_scl);

  void tempRegister(bool isCO2temp);

  void sensorRegister(SENSORS sensor);

  void sensorAnnounce(SENSORS sensor);

  void unitRegister(UNIT unit);

  uint8_t *getUnitsRegistered();

  // Driver layer: each sensor owns its begin()/read() implementation.
  friend class Am2320Driver;
  friend class Bme280Driver;
  friend class Bmp280Driver;
  friend class Bme680Driver;
  friend class AhtxxDriver;
  friend class Sht31Driver;
#ifdef DHT11_ENABLED
  friend class Dht11Driver;
#endif
  friend class Scd30Driver;
  friend class Scd4xDriver;
  friend class Sen5xDriver;
  friend class Sgp41Driver;
  friend class DfrobotGasDriver;
  friend class DfrobotCoDriver;
  friend class DfrobotNh3Driver;
  friend class DfrobotNo2Driver;
  friend class DfrobotO3Driver;
  friend class Sps30Driver;
  friend class Gcja5Driver;
  friend class UartDriver;
  friend class NoiseSensorDriver;
  friend class GeigerDriver;

  // Drivers (declaration order is the read/init order used by Sensors).
  UartDriver _uart;
  Sps30Driver _sps30;
  Gcja5Driver _gcja5;
  Scd30Driver _scd30;
  Scd4xDriver _scd4x;
  Sen5xDriver _sen5x;
  Sgp41Driver _sgp41;
  Am2320Driver _am2320;
  Sht31Driver _sht31;
  Bme280Driver _bme280;
  Bmp280Driver _bmp280;
  Bme680Driver _bme680;
  AhtxxDriver _ahtxx;
  DfrobotCoDriver _dfrCo;
  DfrobotNh3Driver _dfrNh3;
  DfrobotNo2Driver _dfrNo2;
  DfrobotO3Driver _dfrO3;
  NoiseSensorDriver _noise;
  GeigerDriver _geiger;
#ifdef DHT11_ENABLED
  Dht11Driver _dht11;
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
