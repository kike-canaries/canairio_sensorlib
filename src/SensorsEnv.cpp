// CanAirIO sensorlib — SensorsEnv.cpp
// Split from the original monolithic Sensors.cpp. Behavior preserved.
#include "Sensors.hpp"

#include <math.h>
#include <string.h>

#ifdef DHT11_ENABLED
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
#endif

void Sensors::am2320Read() {
  if (!isSensorRegistered(SENSORS::SAM232X)) return;
  if (!am2320.isConnected()) return;
  int status = am2320.read();
  if (status != AM232X_OK) return;
  float humi1 = am2320.getHumidity();
  float temp1 = am2320.getTemperature();
  if (!isnan(humi1)) humi = humi1;
  if (!isnan(temp1)) {
    temp = temp1 - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] AM2320 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::bme280Read() {
  if (!isSensorRegistered(SENSORS::SBME280)) return;
  float humi1 = bme280.readHumidity();
  float temp1 = bme280.readTemperature();
  if (isnan(humi1) || humi1 == 0 || isnan(temp1)) return;
  humi = humi1;
  temp = temp1 - toffset;
  pres = bme280.readPressure() / 100.0F;  // Pa -> hPa
  alt = bme280.readAltitude(sealevel);
  dataReady = true;
  DEBUG("-->[SLIB] BME280 read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::HUM);
  unitRegister(UNIT::ALT);
}

void Sensors::bmp280Read() {
  if (!isSensorRegistered(SENSORS::SBMP280)) return;
  float temp1 = bmp280.readTemperature();
  float press1 = bmp280.readPressure();
  float alt1 = bmp280.readAltitude(sealevel);
  if (press1 == 0 || isnan(temp1) || isnan(alt1)) return;
  temp = temp1 - toffset;
  pres = press1 / 100;  // convert to hPa
  alt = alt1;
  dataReady = true;
  DEBUG("-->[SLIB] BMP280 read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::PRESS);
  unitRegister(UNIT::ALT);
}

void Sensors::bme680Read() {
  if (!isSensorRegistered(SENSORS::SBME680)) return;
  if (!bme680.performReading()) return;
  float temp1 = bme680.temperature;
  temp = temp1 - toffset;
  humi = bme680.humidity;
  pres = bme680.pressure / 100.0;
  gas = bme680.gas_resistance / 1000.0;
  alt = bme680.readAltitude(sealevel);
  dataReady = true;
  DEBUG("-->[SLIB] BME680 read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::HUM);
  unitRegister(UNIT::PRESS);
  unitRegister(UNIT::GAS);
  unitRegister(UNIT::ALT);
}

void Sensors::aht10Read() {
  if (!isSensorRegistered(SENSORS::SAHTXX)) return;
  float temp1 = aht10.readTemperature();
  if (temp1 != AHTXX_ERROR) {
    float humi1 = aht10.readHumidity();
    if (humi1 != AHTXX_ERROR) humi = humi1;
    temp = temp1 - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] AHT10 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

void Sensors::sht31Read() {
  if (!isSensorRegistered(SENSORS::SSHT31)) return;
  float humi1 = sht31.readHumidity();
  float temp1 = sht31.readTemperature();
  if (!isnan(humi1)) humi = humi1;
  if (!isnan(temp1)) {
    temp = temp1 - toffset;
    dataReady = true;
    DEBUG("-->[SLIB] SHT31 read\t\t: done!");
    tempRegister(false);
    unitRegister(UNIT::HUM);
  }
}

#ifdef DHT11_ENABLED
/**
 * @deprecated Please don't use this sensor anymore
 */
bool Sensors::dhtIsReady(float *temperature, float *humidity) {
  static unsigned long measurement_timestamp = millis();
  if (millis() - measurement_timestamp > sample_time * (uint32_t)1000) {
    if (dht_sensor.measure(temperature, humidity) == true) {
      measurement_timestamp = millis();
      return (true);
    }
  }
  return (false);
}

/**
 * @deprecated Please don't use this sensor anymore
 */
void Sensors::dhtInit() {
  sensorAnnounce(SENSORS::SDHTX);
  dhtRead();
}

/**
 * @deprecated Please don't use this sensor anymore
 */
void Sensors::dhtRead() {
  if (dhtIsReady(&dhttemp, &dhthumi) != true) return;
  temp = dhttemp - toffset;
  humi = dhthumi;
  dataReady = true;
  sensorRegister(SENSORS::SDHTX);
  DEBUG("-->[SLIB] DHTXX read\t\t: done!");
  tempRegister(false);
  unitRegister(UNIT::HUM);
}
#endif  // DHT11_ENABLED

void Sensors::am2320Init() {
  sensorAnnounce(SENSORS::SAM232X);
#ifndef Wire1
  if (!am2320.begin()) return;
#else
  am2320 = AM232X(&Wire);
  if (!am2320.begin()) {
    am2320 = AM232X(&Wire1);
    if (!am2320.begin()) return;
  }
#endif
  am2320.wakeUp();
  sensorRegister(SENSORS::SAM232X);
}

void Sensors::sht31Init() {
  sensorAnnounce(SENSORS::SSHT31);
  sht31 = Adafruit_SHT31();
#ifndef Wire1
  if (!sht31.begin()) return;
#else
  if (!sht31.begin()) {
    sht31 = Adafruit_SHT31(&Wire1);
    if (!sht31.begin()) return;
  }
#endif
  sensorRegister(SENSORS::SSHT31);
}

void Sensors::bme280Init() {
  sensorAnnounce(SENSORS::SBME280);
#ifndef Wire1
  if (!bme280.begin() && !bme280.begin(BME280_ADDRESS_ALTERNATE)) return;
#else
  if (!bme280.begin() && !bme280.begin(BME280_ADDRESS_ALTERNATE) &&
      !bme280.begin(BME280_ADDRESS, &Wire1) && !bme280.begin(BME280_ADDRESS_ALTERNATE, &Wire1))
    return;
#endif
  sensorRegister(SENSORS::SBME280);
}

/// Environment BMP280 sensor init
void Sensors::bmp280Init() {
  sensorAnnounce(SENSORS::SBMP280);
#ifndef Wire1
  if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) return;
#else
  if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) {
    bmp280 = Adafruit_BMP280(&Wire1);
    if (!bmp280.begin() && !bmp280.begin(BMP280_ADDRESS_ALT)) return;
  }
#endif
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,      // Operating Mode.
                     Adafruit_BMP280::SAMPLING_X2,      // Temp. oversampling
                     Adafruit_BMP280::SAMPLING_X16,     // Pressure oversampling
                     Adafruit_BMP280::FILTER_X16,       // Filtering.
                     Adafruit_BMP280::STANDBY_MS_500);  // Standby time.
#if CORE_DEBUG_LEVEL >= 3
  Adafruit_Sensor *bmp_temp = bmp280.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = bmp280.getPressureSensor();
  if (devmode) bmp_temp->printSensorDetails();
  if (devmode) bmp_pressure->printSensorDetails();
#endif
  sensorRegister(SENSORS::SBMP280);
}

/// Bosch BME680 sensor init
void Sensors::bme680Init() {
  sensorAnnounce(SENSORS::SBME680);
#ifndef Wire1
  if (!bme680.begin()) return;
#else
  if (bme680.begin() == false) {
    bme680 = Adafruit_BME680(&Wire1);
    if (!bme680.begin()) return;
  }
#endif
  bme680.setTemperatureOversampling(BME680_OS_8X);
  bme680.setHumidityOversampling(BME680_OS_2X);
  bme680.setPressureOversampling(BME680_OS_4X);
  bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme680.setGasHeater(320, 150);  // 320*C for 150 ms
  sensorRegister(SENSORS::SBME680);
}

/// AHTXX sensors init
void Sensors::aht10Init() {
  sensorAnnounce(SENSORS::SAHTXX);
  // TODO: this sensor only works in Wire0
  aht10 = AHTxx(AHTXX_ADDRESS_X38, AHT1x_SENSOR);
  // TODO: check if the last M5 library fix that:
#ifdef M5STICKCPLUS  // issue: https://github.com/enjoyneering/AHTxx/issues/11
  if (!aht10.begin(SLIB_I2C2_SDA, SLIB_I2C2_SCL, 100000, 50000)) return;
#else
  if (!aht10.begin()) return;
#endif
  sensorRegister(SENSORS::SAHTXX);
}
