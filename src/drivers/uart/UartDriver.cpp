// CanAirIO sensorlib — UART transport and autodetected protocols.
#include "UartDriver.h"

#include "../../Sensors.hpp"

#include <math.h>
#include <string.h>

SENSORS UartDriver::type() const {
  int selected = _sensors.getUARTDeviceTypeSelected();
  return selected >= 0 ? (SENSORS)selected : SENSORS::Auto;
}

bool UartDriver::begin(u_int pms_type, int pms_rx, int pms_tx) {
  return sensorSerialInit(pms_type, pms_rx, pms_tx);
}

bool UartDriver::begin() { return enabled(); }

bool UartDriver::read() { return pmSensorRead(); }

bool UartDriver::enabled() const { return _sensors.isUARTSensorConfigured(); }

bool UartDriver::beginS8() {
  _sensors.sensorAnnounce(SENSORS::SAIRS8);
  if (senseAirS8Init()) {
    _sensors.sensorRegister(SENSORS::SAIRS8);
    return true;
  }
  return false;
}

bool UartDriver::readS8() { return senseAirS8Read(); }

void UartDriver::flush() {
  if (_sensors._serial != nullptr) _sensors._serial->flush();
}

void UartDriver::reset() {
  if (_sensors.cm1106 != nullptr) {
    delete _sensors.cm1106;
    _sensors.cm1106 = nullptr;
  }
  if (_sensors.s8 != nullptr) {
    delete _sensors.s8;
    _sensors.s8 = nullptr;
  }
  if (_sensors.pm1006 != nullptr) {
    delete _sensors.pm1006;
    _sensors.pm1006 = nullptr;
  }
  if (_sensors.pm5003t != nullptr) {
    delete _sensors.pm5003t;
    _sensors.pm5003t = nullptr;
  }
  _sensors.s8_uart2 = false;
  _sensors._serial2 = nullptr;
}

/**
 *  @brief PMS sensor generic read. Supported: Honeywell & Plantower sensors
 *  @return true if header and sensor data is right.
 */
bool UartDriver::pmGenericRead() {
  int length_buffer = 32;
  String txtMsg = hwSerialRead(length_buffer);
  if (txtMsg[0] == 66) {
    if (txtMsg[1] == 77) {
      _sensors.DEBUG("-->[SLIB] UART PMGENERIC read!\t: :D");
      _sensors.pm25 = txtMsg[6] * 256 + (char)(txtMsg[7]);
      _sensors.pm10 = txtMsg[8] * 256 + (char)(txtMsg[9]);

      _sensors.unitRegister(UNIT::PM25);
      _sensors.unitRegister(UNIT::PM10);

      if (_sensors.pm25 > 1000 && _sensors.pm10 > 1000) {
        _sensors.onSensorError("[E][SLIB] UART PMGENERIC error\t: out of range pm25 > 1000");
      } else
        return true;
    } else {
      _sensors.onSensorError("[E][SLIB] UART PMGENERIC error\t: invalid header");
    }
  }
  return false;
}

/**
 *  @brief Panasonic GCJA5 particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */
bool UartDriver::pmGCJA5Read() {
  int length_buffer = 32;
  String txtMsg = hwSerialRead(length_buffer);
  if (txtMsg[0] == 02) {
    _sensors.DEBUG("-->[SLIB] UART GCJA5 read\t: done!");
    _sensors.pm1 = txtMsg[2] * 256 + (char)(txtMsg[1]);
    _sensors.pm25 = txtMsg[6] * 256 + (char)(txtMsg[5]);
    _sensors.pm10 = txtMsg[10] * 256 + (char)(txtMsg[9]);

    _sensors.unitRegister(UNIT::PM1);
    _sensors.unitRegister(UNIT::PM25);
    _sensors.unitRegister(UNIT::PM10);

    if (_sensors.pm25 > 2000 && _sensors.pm10 > 2000) {
      _sensors.onSensorError("[W][SLIB] GCJA5 UART msg  \t: out of range pm25 > 2000");
    } else
      return true;
  } else {
    _sensors.onSensorError("[W][SLIB] GCJA5 UART msg  \t: invalid header");
  }
  return false;
}

/**
 *  @brief Nova SDS011 particulate meter sensor read.
 *  @return true if header and sensor data is right.
 */
bool UartDriver::pmSDS011Read() {
  int length_buffer = 10;
  String txtMsg = hwSerialRead(length_buffer);
  if (txtMsg[0] == 170) {
    if (txtMsg[1] == 192) {
      _sensors.DEBUG("-->[SLIB] SDS011 read \t\t: done!");
      _sensors.pm25 = (txtMsg[3] * 256 + (char)(txtMsg[2])) / 10;
      _sensors.pm10 = (txtMsg[5] * 256 + (char)(txtMsg[4])) / 10;

      _sensors.unitRegister(UNIT::PM25);
      _sensors.unitRegister(UNIT::PM10);

      if (_sensors.pm25 > 1000 && _sensors.pm10 > 1000) {
        _sensors.onSensorError("[W][SLIB] SDS011 UART msg\t: out of range pm25 > 1000");
      } else
        return true;
    } else {
      _sensors.onSensorError("[W][SLIB] SDS011 UART msg\t: invalid header");
    }
  }
  return false;
}

bool UartDriver::pm1006Read() {
  uint16_t pm2_5;
  if (_sensors.pm1006->read_pm25(&pm2_5)) {
    _sensors.pm25 = pm2_5;
    _sensors.unitRegister(UNIT::PM25);
    return true;
  }
  return false;
}

bool UartDriver::pm5003TRead() {
  if (!_sensors.isSensorRegistered(SENSORS::P5003T)) return false;
  _sensors.pm5003t->handle();
  _sensors.pm1 = _sensors.pm5003t->getPm01Ae();
  _sensors.pm25 = _sensors.pm5003t->getPm25Ae();
  _sensors.pm10 = _sensors.pm5003t->getPm10Ae();
  _sensors.temp = _sensors.pm5003t->getTemperature() - _sensors.toffset;
  _sensors.humi = _sensors.pm5003t->getRelativeHumidity();
  _sensors.unitRegister(UNIT::PM1);
  _sensors.unitRegister(UNIT::PM25);
  _sensors.unitRegister(UNIT::PM10);
  _sensors.unitRegister(UNIT::HUM);
  _sensors.unitRegister(UNIT::TEMP);
  return true;
}

/**
 * @brief PMSensor Serial read to basic string
 * @param SENSOR_RETRY attempts before failure
 * @return String buffer.
 **/
String UartDriver::hwSerialRead(unsigned int length_buffer) {
  unsigned int try_sensor_read = 0;
  String txtMsg = "";
  while (txtMsg.length() < length_buffer && try_sensor_read++ < SENSOR_RETRY) {
    while (_sensors._serial->available() > 0) {
      char inChar = _sensors._serial->read();
      txtMsg += inChar;
    }
  }
  if (try_sensor_read > SENSOR_RETRY) {
    _sensors.DEBUG("-->[SLIB] UART detection msg\t: no data");
  }
  return txtMsg;
}

bool UartDriver::CO2Mhz19Read() {
  _sensors.CO2Val = _sensors.mhz19.getCO2();                     // Request CO2 (as ppm)
  _sensors.CO2temp = _sensors.mhz19.getTemperature() - _sensors.toffset;  // Request Temperature (as Celsius)
  if (_sensors.CO2Val > 0) {
    if (_sensors.altoffset != 0) _sensors.CO2correctionAlt();
    _sensors.dataReady = true;
    _sensors.DEBUG("-->[SLIB] MHZ14-9 read  \t: done!");
    _sensors.tempRegister(true);
    _sensors.unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

bool UartDriver::CO2CM1106Read() {
  _sensors.CO2Val = _sensors.cm1106->get_co2();
  if (_sensors.CO2Val > 0) {
    _sensors.dataReady = true;
    if (_sensors.altoffset != 0) _sensors.CO2correctionAlt();
    _sensors.DEBUG("-->[SLIB] CM1106 read   \t: done!");
    _sensors.unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

bool UartDriver::senseAirS8Read() {
  _sensors.CO2Val = _sensors.s8->get_co2();  // Request CO2 (as ppm)
  if (_sensors.CO2Val > 0) {
    if (_sensors.altoffset != 0) _sensors.CO2correctionAlt();
    _sensors.dataReady = true;
    _sensors.DEBUG("-->[SLIB] SENSEAIRS8 read   \t: done!");
    _sensors.unitRegister(UNIT::CO2);
    return true;
  }
  return false;
}

/**
 * @brief read sensor data. Sensor selected.
 * @return true if data is loaded from sensor.
 */
bool UartDriver::pmSensorRead() {
  switch (_sensors.dev_uart_type) {
    case Auto:
      return pmGenericRead();
      break;

    case SGCJA5:
      return pmGCJA5Read();
      break;

    case SDS011:
      return pmSDS011Read();
      break;

    case IKEAVK:
      return pm1006Read();
      break;

    case SMHZ19:
      return CO2Mhz19Read();
      break;

    case SCM1106:
      return CO2CM1106Read();
      break;

    case SAIRS8:
      return senseAirS8Read();
      break;

    case P5003T:
      return pm5003TRead();
      break;

    default:
      return false;
      break;
  }
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
bool UartDriver::sensorSerialInit(u_int pms_type, int pms_rx, int pms_tx) {
  // set UART for autodetection sensors (Honeywell, Plantower)
  if (pms_type == SENSORS::Auto) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: Auto");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  }
  // set UART for custom sensors
  else if (pms_type == SENSORS::SGCJA5) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: GCJA5");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SSPS30) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: SSPS30");
    if (!serialInit(pms_type, 115200, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SDS011) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: SDS011");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SMHZ19) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: Mhz19");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SCM1106) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: CM1106");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::SAIRS8) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: SENSEAIRS8");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::IKEAVK) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: SENSEAIRS8");
    if (!serialInit(pms_type, PM1006::BIT_RATE, pms_rx, pms_tx)) return false;
  } else if (pms_type == SENSORS::P5003T) {
    _sensors.DEBUG("-->[SLIB] UART detecting type\t: PMS5003T");
    if (!serialInit(pms_type, 9600, pms_rx, pms_tx)) return false;
  }

  // starting auto detection loop
  int try_sensor_init = 0;
  while (!pmSensorAutoDetect(pms_type) && try_sensor_init++ < 2) {
  }

  // get device selected..
  if (_sensors.dev_uart_type >= 0) {
    _sensors.DEBUG("-->[SLIB] UART sensor detected \t:", _sensors.getSensorName((SENSORS)_sensors.dev_uart_type).c_str());
    _sensors.sensorRegister((SENSORS)_sensors.dev_uart_type);
    return true;
  }

  return false;
}

/**
 * @brief Generic PM sensor auto detection.
 *
 * In order UART config, this method looking up for
 * special header on Serial stream
 **/
bool UartDriver::pmSensorAutoDetect(u_int pms_type) {
  delay(1000);  // sync serial

  if (pms_type == SENSORS::SSPS30) {
    if (_sensors._sps30.beginUart()) {
      _sensors.dev_uart_type = SENSORS::SSPS30;
      return true;
    }
  }

  if (pms_type == SENSORS::SDS011) {
    if (pmSDS011Read()) {
      _sensors.dev_uart_type = SENSORS::SDS011;
      return true;
    }
  }

  if (pms_type == SENSORS::IKEAVK) {
    if (PM1006Init()) {
      _sensors.dev_uart_type = SENSORS::IKEAVK;
      return true;
    }
  }

  if (pms_type == SENSORS::P5003T) {
    if (PM5003TInit()) {
      _sensors.dev_uart_type = SENSORS::P5003T;
      return true;
    }
  }

  if (pms_type == SENSORS::SMHZ19) {
    if (CO2Mhz19Init()) {
      _sensors.dev_uart_type = SENSORS::SMHZ19;
      return true;
    }
  }

  if (pms_type == SENSORS::SCM1106) {
    if (CO2CM1106Init()) {
      _sensors.dev_uart_type = SENSORS::SCM1106;
      return true;
    }
  }

  if (pms_type == SENSORS::SAIRS8) {
    if (senseAirS8Init()) {
      _sensors.dev_uart_type = SENSORS::SAIRS8;
      return true;
    }
  }

  if (pms_type <= SENSORS::SGCJA5) {
    if (pmGenericRead()) {
      _sensors.dev_uart_type = SENSORS::Auto;
      return true;
    }
    delay(1000);  // sync serial
    if (pmGCJA5Read()) {
      _sensors.dev_uart_type = SENSORS::SGCJA5;
      return true;
    }
  }

  return false;
}

bool UartDriver::CO2Mhz19Init() {
  _sensors.mhz19.begin(*_sensors._serial);
  _sensors.mhz19.autoCalibration(false);
  delay(100);
  int co2 = _sensors.mhz19.getCO2();
  if (co2 == 0) return false;
  _sensors.sensorRegister(SENSORS::SMHZ19);
  return true;
}

bool UartDriver::PM1006Init() {
  _sensors.pm1006 = new PM1006(*_sensors._serial);
  _sensors.sensorRegister(SENSORS::IKEAVK);
  return pm1006Read();
}

bool UartDriver::PM5003TInit() {
  _sensors.pm5003t = new PMS5003T(*_sensors._serial);
  if (!_sensors.pm5003t->begin()) return false;
  _sensors.sensorRegister(SENSORS::P5003T);
  return true;
}

bool UartDriver::CO2CM1106Init() {
  _sensors.DEBUG("-->[SLIB] try to enable sensor\t: CM1106..");
  _sensors.cm1106 = new CM1106_UART(*_sensors._serial);

  // Check if CM1106 is available
  _sensors.cm1106->get_software_version(_sensors.cm1106sensor.softver);
  int len = strlen(_sensors.cm1106sensor.softver);
  if (len > 0) {
    if (len >= 10 && !strncmp(_sensors.cm1106sensor.softver + len - 5, "SL-NS", 5)) {
      _sensors.DEBUG("-->[SLIB] CM1106 version detected :D\t: CM1106SL-NS");
    } else if (!strncmp(_sensors.cm1106sensor.softver, "CM", 2)) {
      _sensors.DEBUG("-->[SLIB] CM1106 version detected :D\t: CM1106");
    } else {
      _sensors.DEBUG("-->[SLIB] CM1106 version detected :D\t: unknown");
    }
  } else {
    _sensors.DEBUG("[E][SLIB] CM1106 not detected!");
    return false;
  }

  // Show sensor info
  _sensors.cm1106->get_serial_number(_sensors.cm1106sensor.sn);
  _sensors.DEBUG("-->[SLIB] CM1106 Serial number\t:", _sensors.cm1106sensor.sn);
  _sensors.DEBUG("-->[SLIB] CM1106 Software version\t:", _sensors.cm1106sensor.softver);

  // Setup ABC parameters
  _sensors.DEBUG("-->[SLIB] CM1106 Setting ABC params...");
  _sensors.cm1106->set_ABC(CM1106_ABC_OPEN, 7, 415);  // 7 days cycle, 415 ppm for base

  // Force mode continous B for CM1106SL-NS
  _sensors.cm1106->set_working_status(1);

  // Getting ABC parameters
  if (_sensors.cm1106->get_ABC(&_sensors.abc)) {
    _sensors.DEBUG("-->[SLIB] CM1106 ABC parameters:");
    if (_sensors.abc.open_close == CM1106_ABC_OPEN) {
      _sensors.DEBUG("-->[SLIB] CM1106 Auto calibration is enabled");
    } else if (_sensors.abc.open_close == CM1106_ABC_CLOSE) {
      _sensors.DEBUG("-->[SLIB] CM1106 Auto calibration is disabled");
    }
    _sensors.DEBUG("-->[SLIB] CM1106 Calibration cycle\t:", String(_sensors.abc.cycle).c_str());
    _sensors.DEBUG("-->[SLIB] CM1106 Calibration baseline\t:", String(_sensors.abc.base).c_str());
  }

  return true;
}

bool UartDriver::senseAirS8Init() {
// Second UART for a dedicated sensor (e.g. SenseAir S8 on AG_OPENAIR).
// The PM sensor keeps the first UART, the S8 CO2 sensor uses the second one.
#if defined(AG_OPENAIR) && defined(SLIB_UART2_RX) && defined(SLIB_UART2_TX)
  if (sensorSerial2Init(SLIB_UART2_RX, SLIB_UART2_TX)) {
    _sensors.s8 = new S8_UART(*_sensors._serial2);
  }
#else
  _sensors.s8 = new S8_UART(*_sensors._serial);
#endif
  // Check if S8 is available
  _sensors.s8->get_firmware_version(_sensors.s8sensor.firm_version);
  int len = strlen(_sensors.s8sensor.firm_version);
  if (len == 0) {
    _sensors.DEBUG("[E][SLIB] SENSEAIR S8 not detected!");
    return false;
  }
  if (_sensors.devmode) {
    Serial.printf("-->[SLIB] S8 Software version \t: %s\r\n", _sensors.s8sensor.firm_version);
    Serial.printf("-->[SLIB] S8 Sensor type      \t: 0x%08X\r\n", _sensors.s8->get_sensor_type_ID());
    Serial.printf("-->[SLIB] S8 Sensor ID        \t: %08X\r\n", _sensors.s8->get_sensor_ID());
    Serial.printf("-->[SLIB] S8 Memory ver       \t: 0x%04x\r\n", _sensors.s8->get_memory_map_version());
    Serial.printf("-->[SLIB] S8 ABC period       \t: %d hours\r\n", _sensors.s8->get_ABC_period());
  }
  _sensors.DEBUG("-->[SLIB] S8 Disabling ABC period");
  _sensors.s8->set_ABC_period(0);
  delay(100);
  _sensors.s8->set_ABC_period(180);
  delay(100);
  _sensors.s8->get_meter_status();
  _sensors.s8->get_alarm_status();
  _sensors.s8->get_output_status();
  _sensors.s8->get_acknowledgement();

  return true;
}

/**
 * @brief Initialize the second UART for a dedicated sensor SenseAir S8
 *
 * On ESP32-C3 (AG_OPENAIR) the second UART (Serial1) is used by the PM sensor,
 * the S8 CO2 sensor is attached to the first UART in Serial0 of ESP32C3
 *
 * @param rx RX pin for the second UART.
 * @param tx TX pin for the second UART.
 * @return true if the second UART was initialized.
 */
bool UartDriver::sensorSerial2Init(int rx, int tx) {
#if defined(AG_OPENAIR)
  if (rx < 0 || tx < 0) {
    _sensors.DEBUG("-->[SLIB] UART2 TX/RX line not defined");
    return false;
  }
  printUART("UART2", S8_BAUDRATE, rx, tx);
  Serial0.begin(S8_BAUDRATE, SERIAL_8N1, rx, tx, false);
  _sensors._serial2 = &Serial0;
#endif
  return true;
}

void UartDriver::printUART(const char *name, unsigned long speed, int pin_rx, int pin_tx) {
  if (_sensors.devmode) {
    Serial.printf("-->[SLIB] %s init speed\t: %lu\r\n", name, speed);
    Serial.printf("-->[SLIB] %s init pins \t: TX:%i RX:%i\r\n", name, pin_tx, pin_rx);
  }
}

bool UartDriver::serialInit(u_int pms_type, unsigned long speed_baud, int pms_rx, int pms_tx) {
#if ARDUINO_USB_CDC_ON_BOOT  // Serial used for USB CDC
#ifndef AG_OPENAIR
  printUART("UART1", speed_baud, pms_rx, pms_tx);
  Serial0.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx);
  _sensors._serial = &Serial0;
  return true;
#endif
#endif
  switch (SENSOR_COMMS) {
    case SERIALPORT:
      printUART("UART1", speed_baud, -1, -1);
      Serial.begin(speed_baud);
      _sensors._serial = &Serial;
      break;
#if defined(ARDUINO_ARCH_ESP32)
    // on a Sparkfun ESP32 Thing the default pins for serial1 are used for acccessing flash memory
    // you have to define different pins upfront in order to use serial1 port.
    case SERIALPORT1:
      _sensors.DEBUG("-->[SLIB] UART COMM port \t: Serial1");
      if (pms_rx == 0 || pms_tx == 0) {
        _sensors.DEBUG("-->[SLIB] TX/RX line not defined");
        return false;
      }
      printUART("UART1", speed_baud, pms_rx, pms_tx);
      Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
      _sensors._serial = &Serial1;
      break;
    case SERIALPORT2:
#if SOC_UART_NUM > 2
      if (pms_type == SENSORS::SSPS30) {
        printUART("UART1", speed_baud, -1, -1);
        Serial2.begin(speed_baud);
      } else {
        printUART("UART1", speed_baud, pms_rx, pms_tx);
        Serial2.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx, false);
      }
      _sensors._serial = &Serial2;
#else
      printUART("UART1", speed_baud, pms_rx, pms_tx);
      Serial1.begin(speed_baud, SERIAL_8N1, pms_rx, pms_tx);
      _sensors._serial = &Serial1;
#endif
      break;
#endif
    default:
      if (pms_rx == 0 || pms_tx == 0) {
        _sensors.DEBUG("-->[SLIB] TX/RX line not defined");
        return false;
      }
      // In case RX and TX are both pin 8, try Serial1 anyway.
      // A way to force-enable Serial1 on some boards.
      if (pms_rx == 8 && pms_tx == 8) {
        Serial1.begin(speed_baud);
        _sensors._serial = &Serial1;
      } else {
#if defined(INCLUDE_SOFTWARE_SERIAL)
        _sensors.DEBUG("-->[SLIB] swSerial init on pin\t:", String(pms_rx).c_str());
        static SoftwareSerial swSerial(pms_rx, pms_tx);
        if (pms_type == SENSORS::SSPS30)
          swSerial.begin(speed_baud);
        else if (pms_type == SENSORS::SGCJA5)
          swSerial.begin(speed_baud, SWSERIAL_8E1, pms_rx, pms_tx, false);
        else
          swSerial.begin(speed_baud, SWSERIAL_8N1, pms_rx, pms_tx, false);
        _sensors._serial = &swSerial;
#else
        _sensors.DEBUG("-->[SLIB] UART SoftwareSerial \t: disable");
        return (false);
#endif  // INCLUDE_SOFTWARE_SERIAL
      }
      break;
  }

  delay(10);
  return true;
}
