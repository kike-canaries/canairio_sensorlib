// I2C environment sensors (temperature, humidity, pressure, gas).
#include "EnvDrivers.h"

#include <math.h>

#include "../../Sensors.hpp"

#ifdef DHT11_ENABLED
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

static bool dhtIsReady(Sensors &s, float *temperature, float *humidity) {
  static unsigned long measurement_timestamp = millis();
  if (millis() - measurement_timestamp > s.sample_time * (uint32_t)1000) {
    if (dht_sensor.measure(temperature, humidity) == true) {
      measurement_timestamp = millis();
      return true;
    }
  }
  return false;
}
#endif

bool Am2320Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SAM232X);
#ifndef Wire1
  if (!_sensors.am2320.begin()) return false;
#else
  _sensors.am2320 = AM232X(&Wire);
  if (!_sensors.am2320.begin()) {
    _sensors.am2320 = AM232X(&Wire1);
    if (!_sensors.am2320.begin()) return false;
  }
#endif
  _sensors.am2320.wakeUp();
  _sensors.sensorRegister(SENSORS::SAM232X);
  return true;
}

bool Am2320Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SAM232X)) return false;
  if (!_sensors.am2320.isConnected()) return false;
  int status = _sensors.am2320.read();
  if (status != AM232X_OK) return false;
  float humi = _sensors.am2320.getHumidity();
  float temp = _sensors.am2320.getTemperature();
  bool updated = false;
  if (!isnan(humi)) _sensors.humi = humi;
  if (!isnan(temp)) {
    _sensors.temp = temp - _sensors.toffset;
    _sensors.dataReady = true;
    _sensors.DEBUG("-->[SLIB] AM2320 read\t\t: done!");
    _sensors.tempRegister(false);
    _sensors.unitRegister(UNIT::HUM);
    updated = true;
  }
  return updated;
}

bool Bme280Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SBME280);
#ifndef Wire1
  if (!_sensors.bme280.begin() && !_sensors.bme280.begin(BME280_ADDRESS_ALTERNATE)) return false;
#else
  if (!_sensors.bme280.begin() && !_sensors.bme280.begin(BME280_ADDRESS_ALTERNATE) &&
      !_sensors.bme280.begin(BME280_ADDRESS, &Wire1) &&
      !_sensors.bme280.begin(BME280_ADDRESS_ALTERNATE, &Wire1))
    return false;
#endif
  _sensors.sensorRegister(SENSORS::SBME280);
  return true;
}

bool Bme280Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SBME280)) return false;
  float humi = _sensors.bme280.readHumidity();
  float temp = _sensors.bme280.readTemperature();
  if (isnan(humi) || humi == 0 || isnan(temp)) return false;
  _sensors.humi = humi;
  _sensors.temp = temp - _sensors.toffset;
  _sensors.pres = _sensors.bme280.readPressure() / 100.0F;  // Pa -> hPa
  _sensors.alt = _sensors.bme280.readAltitude(_sensors.sealevel);
  _sensors.dataReady = true;
  _sensors.DEBUG("-->[SLIB] BME280 read\t\t: done!");
  _sensors.tempRegister(false);
  _sensors.unitRegister(UNIT::HUM);
  _sensors.unitRegister(UNIT::ALT);
  return true;
}

bool Bmp280Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SBMP280);
#ifndef Wire1
  if (!_sensors.bmp280.begin() && !_sensors.bmp280.begin(BMP280_ADDRESS_ALT)) return false;
#else
  if (!_sensors.bmp280.begin() && !_sensors.bmp280.begin(BMP280_ADDRESS_ALT)) {
    _sensors.bmp280 = Adafruit_BMP280(&Wire1);
    if (!_sensors.bmp280.begin() && !_sensors.bmp280.begin(BMP280_ADDRESS_ALT)) return false;
  }
#endif
  _sensors.bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Operating Mode.
                              Adafruit_BMP280::SAMPLING_X2,      // Temp. oversampling
                              Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                              Adafruit_BMP280::FILTER_X16,       // Filtering.
                              Adafruit_BMP280::STANDBY_MS_500);  // Standby time.
#if CORE_DEBUG_LEVEL >= 3
  Adafruit_Sensor *bmp_temp = _sensors.bmp280.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = _sensors.bmp280.getPressureSensor();
  if (_sensors.devmode) bmp_temp->printSensorDetails();
  if (_sensors.devmode) bmp_pressure->printSensorDetails();
#endif
  _sensors.sensorRegister(SENSORS::SBMP280);
  return true;
}

bool Bmp280Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SBMP280)) return false;
  float temp = _sensors.bmp280.readTemperature();
  float pres = _sensors.bmp280.readPressure();
  float alt = _sensors.bmp280.readAltitude(_sensors.sealevel);
  if (pres == 0 || isnan(temp) || isnan(alt)) return false;
  _sensors.temp = temp - _sensors.toffset;
  _sensors.pres = pres / 100;  // convert to hPa
  _sensors.alt = alt;
  _sensors.dataReady = true;
  _sensors.DEBUG("-->[SLIB] BMP280 read\t\t: done!");
  _sensors.tempRegister(false);
  _sensors.unitRegister(UNIT::PRESS);
  _sensors.unitRegister(UNIT::ALT);
  return true;
}

bool Bme680Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SBME680);
#ifndef Wire1
  if (!_sensors.bme680.begin()) return false;
#else
  if (_sensors.bme680.begin() == false) {
    _sensors.bme680 = Adafruit_BME680(&Wire1);
    if (!_sensors.bme680.begin()) return false;
  }
#endif
  _sensors.bme680.setTemperatureOversampling(BME680_OS_8X);
  _sensors.bme680.setHumidityOversampling(BME680_OS_2X);
  _sensors.bme680.setPressureOversampling(BME680_OS_4X);
  _sensors.bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  _sensors.bme680.setGasHeater(320, 150);  // 320*C for 150 ms
  _sensors.sensorRegister(SENSORS::SBME680);
  return true;
}

bool Bme680Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SBME680)) return false;
  if (!_sensors.bme680.performReading()) return false;
  float temp = _sensors.bme680.temperature;
  _sensors.temp = temp - _sensors.toffset;
  _sensors.humi = _sensors.bme680.humidity;
  _sensors.pres = _sensors.bme680.pressure / 100.0;
  _sensors.gas = _sensors.bme680.gas_resistance / 1000.0;
  _sensors.alt = _sensors.bme680.readAltitude(_sensors.sealevel);
  _sensors.dataReady = true;
  _sensors.DEBUG("-->[SLIB] BME680 read\t\t: done!");
  _sensors.tempRegister(false);
  _sensors.unitRegister(UNIT::HUM);
  _sensors.unitRegister(UNIT::PRESS);
  _sensors.unitRegister(UNIT::GAS);
  _sensors.unitRegister(UNIT::ALT);
  return true;
}

bool AhtxxDriver::begin() {
  _sensors.sensorAnnounce(SENSORS::SAHTXX);
  // TODO: this sensor only works in Wire0
  _sensors.aht10 = AHTxx(AHTXX_ADDRESS_X38, AHT1x_SENSOR);
  // TODO: check if the last M5 library fix that:
#ifdef M5STICKCPLUS  // issue: https://github.com/enjoyneering/AHTxx/issues/11
  if (!_sensors.aht10.begin(SLIB_I2C2_SDA, SLIB_I2C2_SCL, 100000, 50000)) return false;
#else
  if (!_sensors.aht10.begin()) return false;
#endif
  _sensors.sensorRegister(SENSORS::SAHTXX);
  return true;
}

bool AhtxxDriver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SAHTXX)) return false;
  float temp = _sensors.aht10.readTemperature();
  if (temp == AHTXX_ERROR) return false;
  float humi = _sensors.aht10.readHumidity();
  if (humi != AHTXX_ERROR) _sensors.humi = humi;
  _sensors.temp = temp - _sensors.toffset;
  _sensors.dataReady = true;
  _sensors.DEBUG("-->[SLIB] AHT10 read\t\t: done!");
  _sensors.tempRegister(false);
  _sensors.unitRegister(UNIT::HUM);
  return true;
}

bool Sht31Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SSHT31);
  _sensors.sht31 = Adafruit_SHT31();
#ifndef Wire1
  if (!_sensors.sht31.begin()) return false;
#else
  if (!_sensors.sht31.begin()) {
    _sensors.sht31 = Adafruit_SHT31(&Wire1);
    if (!_sensors.sht31.begin()) return false;
  }
#endif
  _sensors.sensorRegister(SENSORS::SSHT31);
  return true;
}

bool Sht31Driver::read() {
  if (!_sensors.isSensorRegistered(SENSORS::SSHT31)) return false;
  float humi = _sensors.sht31.readHumidity();
  float temp = _sensors.sht31.readTemperature();
  bool updated = false;
  if (!isnan(humi)) _sensors.humi = humi;
  if (!isnan(temp)) {
    _sensors.temp = temp - _sensors.toffset;
    _sensors.dataReady = true;
    _sensors.DEBUG("-->[SLIB] SHT31 read\t\t: done!");
    _sensors.tempRegister(false);
    _sensors.unitRegister(UNIT::HUM);
    updated = true;
  }
  return updated;
}

#ifdef DHT11_ENABLED
bool Dht11Driver::begin() {
  _sensors.sensorAnnounce(SENSORS::SDHTX);
  return read();
}

bool Dht11Driver::read() {
  if (!dhtIsReady(_sensors, &_sensors.dhttemp, &_sensors.dhthumi)) return false;
  _sensors.temp = _sensors.dhttemp - _sensors.toffset;
  _sensors.humi = _sensors.dhthumi;
  _sensors.dataReady = true;
  _sensors.sensorRegister(SENSORS::SDHTX);
  _sensors.DEBUG("-->[SLIB] DHTXX read\t\t: done!");
  _sensors.tempRegister(false);
  _sensors.unitRegister(UNIT::HUM);
  return true;
}
#endif
