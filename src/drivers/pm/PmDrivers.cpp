// Sensirion SPS30 and Panasonic SN-GCJA5 (I2C side).
#include "PmDrivers.h"

#include <math.h>
#include <string.h>

#include "../../Sensors.hpp"

bool Sps30Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SSPS30)) return false;
  uint8_t ret, error_cnt = 0;
  delay(35);  // Delay for synchronization

  do {
    ret = _sensors.sps30.GetValues(&_sensors.val);
    if (ret == SPS30_ERR_DATALENGTH) {
      if (error_cnt++ > 3) {
        _sensors.DEBUG("[W][SLIB] SPS30 setup message \t: error on values\t: ",
                       String(ret).c_str());
        return false;
      }
      delay(500);
    } else if (ret != SPS30_ERR_OK) {
      errToMess((char *)"[W][SLIB] SPS30 setup message \t: error on values\t: ", ret);
      return false;
    }
  } while (ret != SPS30_ERR_OK);

  _sensors.DEBUG("-->[SLIB] SPS30 read \t\t: done!");

  _sensors.pm1 = round(_sensors.val.MassPM1);
  _sensors.pm25 = round(_sensors.val.MassPM2);
  _sensors.pm4 = round(_sensors.val.MassPM4);
  _sensors.pm10 = round(_sensors.val.MassPM10);

  _sensors.unitRegister(UNIT::PM1);
  _sensors.unitRegister(UNIT::PM25);
  _sensors.unitRegister(UNIT::PM4);
  _sensors.unitRegister(UNIT::PM10);

  if (_sensors.pm25 > 1000 && _sensors.pm10 > 1000) {
    _sensors.onSensorError("[W][SLIB] SPS30 setup message \t: out of range pm25 > 1000");
    return false;
  }

  _sensors.dataReady = true;
  return true;
}

void Sps30Driver::errToMess(char *mess, uint8_t r) {
  char buf[80];
  _sensors.sps30.GetErrDescription(r, buf, 80);
  _sensors.DEBUG("[E][SLIB] SPS30 error msg\t:", buf);
}

void Sps30Driver::errorloop(char *mess, uint8_t r) {
  if (r)
    errToMess(mess, r);
  else
    _sensors.DEBUG(mess);
}

bool Sps30Driver::beginUart() {
  _sensors.sensorAnnounce(SENSORS::SSPS30);
  // set driver debug level
  if (CORE_DEBUG_LEVEL > 0) {
    _sensors.sps30.EnableDebugging(true);
  }
  // Begin communication channel (non-ESP32: use Stream* e.g. SoftwareSerial; ESP32: SENSOR_COMMS)
#if defined(ARDUINO_ARCH_ESP32)
  if (!_sensors.sps30.begin(SENSOR_COMMS)) {
#else
  if (_sensors._serial == nullptr || !_sensors.sps30.begin(*_sensors._serial)) {
#endif
    errorloop((char *)"[E][SLIB] UART SPS30 could not initialize communication channel.", 0);
    return false;
  }

  if (!tests()) return false;

  // start measurement
  if (_sensors.sps30.start() == true) {
    _sensors.DEBUG("-->[SLIB] SPS30 Measurement OK");
    _sensors.sensorRegister(SENSORS::SSPS30);
    return true;
  } else
    errorloop((char *)"[E][SLIB] UART SPS30 Could NOT start measurement", 0);

  return false;
}

bool Sps30Driver::begin() {
  if (_sensors.dev_uart_type == SENSORS::SSPS30) return false;
  _sensors.sensorAnnounce(SENSORS::SSPS30);
  // Begin communication channel;
  if (_sensors.sps30.begin(&Wire) == false) {
    errorloop((char *)"[E][SLIB] I2C SPS30 could not set channel.", 0);
    return false;
  }

  if (!tests()) return false;

  _sensors.DEBUG("-->[SLIB] SPS30 Detected via\t: I2C");

  // start measurement
  if (_sensors.sps30.start()) {
    _sensors.DEBUG("-->[SLIB] SPS30 measurement \t: OK");
    if (_sensors.sps30.I2C_expect() == 4)
      _sensors.DEBUG("[W][SLIB] SPS30 setup message\t: I2C buffersize only PM values  \r\n");
    _sensors.sensorRegister(SENSORS::SSPS30);
    return true;
  } else
    errorloop((char *)"[W][SLIB] I2C SPS30 message \t: Could NOT start measurement.", 0);

  return false;
}

bool Sps30Driver::tests() {
  // check for SPS30 connection
  if (!_sensors.sps30.probe()) {
    errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not probe.", 0);
    return false;
  } else {
    deviceInfo();
  }
  // reset SPS30 connection
  if (!_sensors.sps30.reset()) {
    errorloop((char *)"[W][SLIB] SPS30 setup message \t: could not reset.", 0);
    return false;
  }
  return true;
}

void Sps30Driver::deviceInfo() {
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  // try to read serial number
  ret = _sensors.sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK) {
    if (strlen(buf) > 0)
      _sensors.DEBUG("-->[SLIB] SPS30 Serial number\t: ", buf);
    else
      _sensors.DEBUG("[SLIB] SPS30 could not get serial number");
  } else
    _sensors.DEBUG("[SLIB] SPS30 could not get serial number");

  // try to get product name
  ret = _sensors.sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK) {
    if (strlen(buf) > 0)
      _sensors.DEBUG("-->[SLIB] SPS30 product name\t: ", buf);
    else
      _sensors.DEBUG("[SLIB] SPS30 could not get product name.");
  } else
    _sensors.DEBUG("[SLIB] SPS30 could not get product name.");

  // try to get version info
  ret = _sensors.sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK) {
    _sensors.DEBUG("[SLIB] SPS30 can not read version info");
    return;
  }
  sprintf(buf, "%d.%d", v.major, v.minor);
  _sensors.DEBUG("-->[SLIB] SPS30 firmware level\t: ", buf);

  if (SENSOR_COMMS != I2C_COMMS) {
    sprintf(buf, "%d.%d", v.SHDLC_major, v.SHDLC_minor);
    _sensors.DEBUG("-->[SLIB] SPS30 Hardware level\t:", String(v.HW_version).c_str());
    _sensors.DEBUG("-->[SLIB] SPS30 SHDLC protocol\t:", buf);
  }

  sprintf(buf, "%d.%d", v.DRV_major, v.DRV_minor);
  _sensors.DEBUG("-->[SLIB] SPS30 Library level\t:", buf);
}

bool Gcja5Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SGCJA5);
#ifndef Wire1
  if (!_sensors.pmGCJA5.begin()) return false;
#else
  if (_sensors.pmGCJA5.begin() == false) {
    if (!_sensors.pmGCJA5.begin(Wire1)) return false;
  }
#endif
  _sensors.sensorRegister(SENSORS::SGCJA5);
  return true;
}

bool Gcja5Driver::read() {
  if (_sensors.dev_uart_type == SENSORS::SGCJA5) return false;
  if (!_sensors.isSensorRegistered(SENSORS::SGCJA5)) return false;
  if (!_sensors.pmGCJA5.isConnected()) return false;
  uint16_t pm1 = _sensors.pmGCJA5.getPM1_0();
  uint16_t pm25 = _sensors.pmGCJA5.getPM2_5();
  uint16_t pm10 = _sensors.pmGCJA5.getPM10();
  if (pm1 > 1000 || pm25 > 1000 || pm10 > 1000) return false;
  _sensors.pm1 = pm1;
  _sensors.pm25 = pm25;
  _sensors.pm10 = pm10;
  _sensors.dataReady = true;
  _sensors.DEBUG("-->[SLIB] GCJA5 read\t\t: done!");
  _sensors.unitRegister(UNIT::PM1);
  _sensors.unitRegister(UNIT::PM25);
  _sensors.unitRegister(UNIT::PM10);
  return true;
}
