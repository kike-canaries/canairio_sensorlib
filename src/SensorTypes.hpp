#ifndef SENSOR_TYPES_HPP
#define SENSOR_TYPES_HPP

#include <stdlib.h>

// Sensors units definitions (symbol/name)
#define SENSOR_UNITS                            \
  X(NUNIT, "NUNIT", "NUNIT")                    \
  X(PM1, "ug/m3", "PM1")                        \
  X(PM25, "ug/m3", "PM2.5")                     \
  X(PM4, "ug/m3", "PM4")                        \
  X(PM10, "ug/m3", "PM10")                      \
  X(TEMP, "C", "T")                             \
  X(TEMPK, "K", "T")                            \
  X(TEMPF, "F", "T")                            \
  X(HUM, "%", "H")                              \
  X(CO2, "ppm", "CO2")                          \
  X(CO2TEMP, "C", "CO2T")                       \
  X(CO2TEMPK, "K", "CO2TK")                     \
  X(CO2TEMPF, "F", "CO2TF")                     \
  X(CO2HUM, "%", "CO2H")                        \
  X(PRESS, "hPa", "P")                          \
  X(ALT, "m", "Alt")                            \
  X(GAS, "Ohm", "Gas")                          \
  X(CPM, "CPM", "RAD")                          \
  X(RAD, "uSv/h", "RAD")                        \
  X(NH3, "ppm", "NH3")                          \
  X(CO, "ppm", "CO")                            \
  X(NO2, "ppm", "NO2")                          \
  X(O3, "ppm", "O3")                            \
  X(NOXI, "noxi", "NOXI")                       \
  X(VOCI, "voci", "VOCI")                       \
  X(NOX, "nox", "NOX")                          \
  X(VOC, "voc", "VOC")                          \
  X(NOISE, "dB", "Noise")                       \
  X(NOISEAVG, "dB", "NoiseAvg")                 \
  X(NOISEPEAK, "dB", "NoisePeak")               \
  X(NOISEMIN, "dB", "NoiseMin")                 \
  X(NOISEAVGLEGAL, "dB", "NoiseAvgLegal")       \
  X(NOISEAVGLEGALMAX, "dB", "NoiseAvgLegalMax") \
  X(NOISELD, "dB", "Ld")                        \
  X(NOISELE, "dB", "Le")                        \
  X(NOISELN, "dB", "Ln")                        \
  X(NOISELDEN, "dB", "Lden")                    \
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
typedef enum SENSORS : size_t { SENSORS_TYPES } SENSORS;
#undef X

#define SCOUNT SENSORS::SCOUNT

// MAIN SENSOR GROUPS TYPE
enum class SensorGroup { SENSOR_NONE, SENSOR_PM, SENSOR_CO2, SENSOR_ENV, SENSOR_RAD };

// TEMPERATURE UNITS
enum class TEMPUNIT { CELSIUS, FAHRENHEIT, KELVIN };

#endif  // SENSOR_TYPES_HPP
