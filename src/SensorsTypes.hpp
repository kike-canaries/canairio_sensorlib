#ifndef SensorsTypes_hpp
#define SensorsTypes_hpp

#include <stddef.h>

// Shared public types and build-time configuration for CanAirIO sensorlib.
// Kept independent from the Sensors facade so driver headers can include it
// without pulling every third-party sensor library.

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
#define PMS_TYPE 8
#define PMS_RX 20
#define PMS_TX 21
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

#ifdef M5AIRQ
#define GROVE_SDA 13
#define GROVE_SCL 15
#define I2C1_SDA_PIN 11
#define I2C1_SCL_PIN 12
#endif

// Read UART sensor retry.
#define SENSOR_RETRY 1000  // Max Serial characters

// UART default port (ESP8266 has no Serial2, use SoftwareSerial via default branch)
#if defined(ARDUINO_ARCH_ESP32)
#define SENSOR_COMMS SERIALPORT2
#else
#define SENSOR_COMMS 3  // Custom value: use default (SoftwareSerial) path on ESP8266
#endif

// Sensors units definitions (symbol/name)
#define SENSOR_UNITS                      \
  X(NUNIT, "NUNIT", "NUNIT")              \
  X(PM1, "ug/m3", "PM1")                  \
  X(PM25, "ug/m3", "PM2.5")               \
  X(PM4, "ug/m3", "PM4")                  \
  X(PM10, "ug/m3", "PM10")                \
  X(TEMP, "C", "T")                       \
  X(TEMPK, "K", "T")                      \
  X(TEMPF, "F", "T")                      \
  X(HUM, "%", "H")                        \
  X(CO2, "ppm", "CO2")                    \
  X(CO2TEMP, "C", "CO2T")                 \
  X(CO2TEMPK, "K", "CO2TK")               \
  X(CO2TEMPF, "F", "CO2TF")               \
  X(CO2HUM, "%", "CO2H")                  \
  X(PRESS, "hPa", "P")                    \
  X(ALT, "m", "Alt")                      \
  X(GAS, "Ohm", "Gas")                    \
  X(CPM, "CPM", "RAD")                    \
  X(RAD, "uSv/h", "RAD")                  \
  X(NH3, "ppm", "NH3")                    \
  X(CO, "ppm", "CO")                      \
  X(NO2, "ppm", "NO2")                    \
  X(O3, "ppm", "O3")                      \
  X(NOXI, "noxi", "NOXI")                 \
  X(VOCI, "voci", "VOCI")                 \
  X(NOX, "nox", "NOX")                    \
  X(VOC, "voc", "VOC")                    \
  X(NOISE, "dB", "Noise")                 \
  X(NOISEPEAK, "dB", "NoisePeak")         \
  X(NOISEAVGLEGAL, "dB", "NoiseAvgLegal") \
  X(NOISEL90, "dB", "NoiseL90")           \
  X(NOISELASMAX, "dB", "NoiseLASmax")     \
  X(NOISELCPEAK, "dB", "NoiseLCpeak")     \
  X(NOISELD, "dB", "Ld")                  \
  X(NOISELE, "dB", "Le")                  \
  X(NOISELN, "dB", "Ln")                  \
  X(NOISELDEN, "dB", "Lden")              \
  X(UCOUNT, "COUNT", "UCOUNT")

#define X(unit, symbol, name) unit,
typedef enum UNIT : size_t { SENSOR_UNITS } UNIT;
#undef X

// sensors types: 1:PM, 2:CO2, 3:ENV
#define SENSORS_TYPES     \
  X(Auto, "GENERIC", 1)   \
  X(SGCJA5, "GCJA5", 1)   \
  X(SSPS30, "SPS30", 1)   \
  X(SDS011, "SDS011", 1)  \
  X(SMHZ19, "MHZ19", 2)   \
  X(SCM1106, "CM1106", 2) \
  X(SAIRS8, "SAIRS8", 2)  \
  X(IKEAVK, "IKEAVK", 1)  \
  X(P5003T, "PM5003T", 1) \
  X(SSCD30, "SCD30", 2)   \
  X(SSCD4X, "SCD4X", 2)   \
  X(SSEN5X, "SEN5X", 1)   \
  X(SSHT31, "SHT31", 3)   \
  X(SBME280, "BME280", 3) \
  X(SBMP280, "BMP280", 3) \
  X(SBME680, "BME680", 3) \
  X(SAHTXX, "AHTXX", 3)   \
  X(SAM232X, "AM232X", 3) \
  X(SDHTX, "DHTX", 3)     \
  X(SDFRCO, "DFRCO", 3)   \
  X(SDFRNH3, "DFRNH3", 3) \
  X(SDFRNO2, "DFRNO2", 3) \
  X(SDFRO3, "DFRO3", 3)   \
  X(SCAJOE, "CAJOE", 3)   \
  X(SSGP41, "SGP41", 3)   \
  X(SNOISE, "NOISE", 3)   \
  X(SCOUNT, "SCOUNT", 3)

#define X(utype, uname, umaintype) utype,
typedef enum SENSORS : size_t { SENSORS_TYPES } SENSORS;  // backward compatibility
#undef X

// MAIN SENSOR GROUPS TYPE
enum class SensorGroup {
  SENSOR_NONE,
  SENSOR_PM,
  SENSOR_CO2,
  SENSOR_ENV,
  SENSOR_RAD  // CAJOE_GEIGER
};
// TEMPERATURE UNITS
enum class TEMPUNIT { CELSIUS, FAHRENHEIT, KELVIN };

typedef void (*errorCbFn)(const char *msg);
typedef void (*voidCbFn)();

#endif  // SensorsTypes_hpp
