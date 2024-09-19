#include "PMS5003T.h"

/**
 * @brief Constructor.
 *
 * @param serial the serial port, NOTE: the serial port has to be pre-configured
 */
PMS5003T::PMS5003T(Stream &serial) { this->_serial = &serial; }

/**
 * @brief Init sensor
 *
 * @return true Success
 * @return false Failure
 */
bool PMS5003T::begin(void) {
  if (this->_isBegin) {
    return true;
  }

  if (pms.begin(this->_serial) == false) {
    #if defined(ESP32)
    log_e("PMS failed");
    #endif
    return false;
  }

  this->_isBegin = true;
  return true;
}

/**
 * @brief Read PM1.0 must call this function after @ref readData success
 *
 * @return int PM1.0 index
 */
int PMS5003T::getPm01Ae(void) { return pms.getPM0_1(); }

/**
 * @brief Read PM2.5 must call this function after @ref readData success
 *
 * @return int PM2.5 index
 */
int PMS5003T::getPm25Ae(void) { return pms.getPM2_5(); }

/**
 * @brief Read PM10.0 must call this function after @ref readData success
 *
 * @return int PM10.0 index
 */
int PMS5003T::getPm10Ae(void) { return pms.getPM10(); }

/**
 * @brief Read PM 0.3 Count must call this function after @ref readData success
 *
 * @return int PM 0.3 Count index
 */
int PMS5003T::getPm03ParticleCount(void) { return pms.getCount0_3(); }

/**
 * @brief Convert PM2.5 to US AQI
 *
 * @param pm25 PM2.5 index
 * @return int PM2.5 US AQI
 */
int PMS5003T::convertPm25ToUsAqi(int pm25) { return pms.pm25ToAQI(pm25); }

/**
 * @brief Get temperature, Must call this method after @ref readData() success
 *
 * @return float Degree Celcius
 */
float PMS5003T::getTemperature(void) {
  return pms.getTemp()/10.0f;
}

/**
 * @brief Get humidity, Must call this method after  @ref readData() success
 *
 * @return float Percent (%)
 */
float PMS5003T::getRelativeHumidity(void) {
  return pms.getHum()/10.0f;
}

/**
 * @brief Check device initialized or not
 *
 * @return true Initialized
 * @return false No-initialized
 */
bool PMS5003T::isBegin(void) {
  if (this->_isBegin == false) {
#if defined(ESP32)
    log_d("Not-initialized");
#endif
    return false;
  }
  return true;
}

void PMS5003T::end(void) {
  if (_isBegin == false) {
    return;
  }
  _isBegin = false;
  delete _serial;
#if defined(ESP32)
  log_d("De-initialize");
#endif
}

/**
 * @brief Check and read PMS sensor data. This method should be callack from
 * loop process to continoue check sensor data if it's available
 */
void PMS5003T::handle(void) { pms.handle(); }

/**
 * @brief Get sensor status
 *
 * @return true No problem
 * @return false Communication timeout or sensor has removed
 */
bool PMS5003T::isFailed(void) { return pms.isFailed(); }

