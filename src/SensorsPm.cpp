// CanAirIO sensorlib — SensorsPm.cpp
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include "Sensors.hpp"

#include <math.h>
#include <string.h>

/**
 *  @brief Sensirion SPS30 particulate meter sensor read.
 *  @return true if reads success.
 */
bool Sensors::sps30Read() {
  if (!isSensorRegistered(SENSORS::SSPS30)) return false;
  uint8_t ret, error_cnt = 0;
  delay(35);  // Delay for synchronization

  do {
    ret = sps30.GetValues(&val);
    if (ret == SPS30_ERR_DATALENGTH) {
      if (error_cnt++ > 3) {
        DEBUG("[W][SLIB] SPS30 setup message \t: error on values\t: ", String(ret).c_str());
        return false;
      }
      delay(500);
    } else if (ret != SPS30_ERR_OK) {
      sps30ErrToMess((char *)"[W][SLIB] SPS30 setup message \t: error on values\t: ", ret);
      return false;
    }
  } while (ret != SPS30_ERR_OK);

  DEBUG("-->[SLIB] SPS30 read \t\t: done!");

  pm1 = round(val.MassPM1);
  pm25 = round(val.MassPM2);
  pm4 = round(val.MassPM4);
  pm10 = round(val.MassPM10);

  unitRegister(UNIT::PM1);
  unitRegister(UNIT::PM25);
  unitRegister(UNIT::PM4);
  unitRegister(UNIT::PM10);

  if (pm25 > 1000 && pm10 > 1000) {
    onSensorError("[W][SLIB] SPS30 setup message \t: out of range pm25 > 1000");
    return false;
  }

  dataReady = true;

  return true;
}

void Sensors::GCJA5Read() {
  if (dev_uart_type == SENSORS::SGCJA5) return;
  if (!isSensorRegistered(SENSORS::SGCJA5)) return;
  if (!pmGCJA5.isConnected()) return;
  uint16_t _pm1 = pmGCJA5.getPM1_0();
  uint16_t _pm25 = pmGCJA5.getPM2_5();
  uint16_t _pm10 = pmGCJA5.getPM10();
  if (_pm1 > 1000 || _pm25 > 1000 || _pm10 > 1000) return;
  pm1 = _pm1;
  pm25 = _pm25;
  pm10 = _pm10;
  dataReady = true;
  DEBUG("-->[SLIB] GCJA5 read\t\t: done!");
  unitRegister(UNIT::PM1);
  unitRegister(UNIT::PM25);
  unitRegister(UNIT::PM10);
}

void Sensors::sps30ErrToMess(char *mess, uint8_t r) {
  char buf[80];
  sps30.GetErrDescription(r, buf, 80);
  DEBUG("[E][SLIB] SPS30 error msg\t:", buf);
}

void Sensors::sps30Errorloop(char *mess, uint8_t r) {
  if (r)
    sps30ErrToMess(mess, r);
  else
    DEBUG(mess);
}

bool Sensors::sps30UARTInit() {
  sensorAnnounce(SENSORS::SSPS30);
  // set driver debug level
  if (CORE_DEBUG_LEVEL > 0) {
    sps30.EnableDebugging(true);
  }
  // Begin communication channel (non-ESP32: use Stream* e.g. SoftwareSerial; ESP32: SENSOR_COMMS)
#if defined(ARDUINO_ARCH_ESP32)
  if (!sps30.begin(SENSOR_COMMS)) {
#else
  if (_serial == nullptr || !sps30.begin(*_serial)) {
#endif
    sps30Errorloop((char *)"[E][SLIB] UART SPS30 could not initialize communication channel.", 0);
    return false;
  }

  if (!sps30tests()) return false;

  // start measurement
  if (sps30.start() == true) {
    DEBUG("-->[SLIB] SPS30 Measurement OK");
    sensorRegister(SENSORS::SSPS30);
    return true;
  } else
    sps30Errorloop((char *)"[E][SLIB] UART SPS30 Could NOT start measurement", 0);

  return false;
}

bool Sensors::sps30I2CInit() {
  if (dev_uart_type == SENSORS::SSPS30) return false;
  sensorAnnounce(SENSORS::SSPS30);
  // set driver debug level
  // if (CORE_DEBUG_LEVEL > 0) sps30.EnableDebugging(true);
  // Begin communication channel;
  if (sps30.begin(&Wire) == false) {
    sps30Errorloop((char *)"[E][SLIB] I2C SPS30 could not set channel.", 0);
    return false;
  }

  if (!sps30tests()) return false;

  DEBUG("-->[SLIB] SPS30 Detected via\t: I2C");

  // start measurement
  if (sps30.start()) {
    DEBUG("-->[SLIB] SPS30 measurement \t: OK");
    if (sps30.I2C_expect() == 4)
      DEBUG("[W][SLIB] SPS30 setup message\t: I2C buffersize only PM values  \r\n");
    sensorRegister(SENSORS::SSPS30);
    return true;
  } else
    sps30Errorloop((char *)"[W][SLIB] I2C SPS30 message \t: Could NOT start measurement.", 0);

  return false;
}

bool Sensors::sps30tests() {
  // check for SPS30 connection
  if (!sps30.probe()) {
    sps30Errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not probe.", 0);
    return false;
  } else {
    sps30DeviceInfo();
  }
  // reset SPS30 connection
  if (!sps30.reset()) {
    sps30Errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not reset.", 0);
    return false;
  }
  return true;
}

/**
 * @brief : read and display Sensirion device info.
 */
void Sensors::sps30DeviceInfo() {
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  // try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK) {
    if (strlen(buf) > 0)
      DEBUG("-->[SLIB] SPS30 Serial number\t: ", buf);
    else
      DEBUG("[SLIB] SPS30 could not get serial number");
  } else
    DEBUG("[SLIB] SPS30 could not get serial number");

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK) {
    if (strlen(buf) > 0)
      DEBUG("-->[SLIB] SPS30 product name\t: ", buf);
    else
      DEBUG("[SLIB] SPS30 could not get product name.");
  } else
    DEBUG("[SLIB] SPS30 could not get product name.");

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK) {
    DEBUG("[SLIB] SPS30 can not read version info");
    return;
  }
  sprintf(buf, "%d.%d", v.major, v.minor);
  DEBUG("-->[SLIB] SPS30 firmware level\t: ", buf);

  if (SENSOR_COMMS != I2C_COMMS) {
    sprintf(buf, "%d.%d", v.SHDLC_major, v.SHDLC_minor);
    DEBUG("-->[SLIB] SPS30 Hardware level\t:", String(v.HW_version).c_str());
    DEBUG("-->[SLIB] SPS30 SHDLC protocol\t:", buf);
  }

  sprintf(buf, "%d.%d", v.DRV_major, v.DRV_minor);
  DEBUG("-->[SLIB] SPS30 Library level\t:", buf);
}

/// Panasonic GCJA5 sensor init
void Sensors::GCJA5Init() {
  sensorAnnounce(SENSORS::SGCJA5);
#ifndef Wire1
  if (!pmGCJA5.begin()) return;
#else
  if (pmGCJA5.begin() == false) {
    if (!pmGCJA5.begin(Wire1)) return;
  }
#endif
  sensorRegister(SENSORS::SGCJA5);
}
