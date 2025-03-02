/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2025
 * @brief CanAirIO M5AirQ test
 * @license GPL3
 *
 * Full documentation:
 * https://github.com/kike-canaries/canairio_sensorlib#canairio-air-quality-sensors-library
 *
 * Full implementation for WiFi and Bluetooth Air Quality fixed and mobile station:
 * https://github.com/kike-canaries/canairio_firmware#canairio-firmware
 *
 * CanAirIO project documentation:
 * https://canair.io/docs
 */

#include <Arduino.h>
#include "Wire.h"
#include <Sensors.hpp>

#define POWER_HOLD 3 // power sensors board enable pin

#define GROVE_SDA 13 // original grove port (defaults)
#define GROVE_SCL 14

#define I2C1_SDA_PIN 8 // Wire1 alternative pins
#define I2C1_SCL_PIN 9

void printSensorsDetected() {
  uint16_t sensors_count = sensors.getSensorsRegisteredCount();
  uint16_t units_count = sensors.getUnitsRegisteredCount();
  Serial.println("-->[MAIN] Sensors detected   \t: " + String(sensors_count));
  Serial.println("-->[MAIN] Sensors units count\t: " + String(units_count));
  Serial.print("-->[MAIN] Sensors devices names\t: ");
  int i = 0;
  while (sensors.getSensorsRegistered()[i++] != 0) {
    Serial.print(sensors.getSensorName((SENSORS)sensors.getSensorsRegistered()[i - 1]));
    Serial.print(",");
  }
  Serial.println();
}

void printSensorsValues() {
  Serial.println("-->[MAIN] Preview sensor values :");
  UNIT unit = sensors.getNextUnit();
  while (unit != UNIT::NUNIT) {
    String uName = sensors.getUnitName(unit);
    float uValue = sensors.getUnitValue(unit);
    String uSymb = sensors.getUnitSymbol(unit);
    Serial.printf("-->[MAIN] %6s:\t%02.1f\t%s\n", uName.c_str(), uValue, uSymb.c_str());
    unit = sensors.getNextUnit();
  }
}

void onSensorDataOk() {
  Serial.println("======= E X A M P L E   T E S T =========");
  printSensorsDetected();
  printSensorsValues();
  delay(2000);
}

void onSensorDataError(const char* msg) {}
/******************************************************************************
 *  M A I N
 ******************************************************************************/

void powerEnableSensors() {
  Serial.println("-->[POWR] == enable sensors ==");
  pinMode(POWER_HOLD, OUTPUT);
  digitalWrite(POWER_HOLD, HIGH);
  pinMode(GROVE_SDA, OUTPUT);
  pinMode(GROVE_SCL, OUTPUT);
  Wire.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);
  Wire1.begin(GROVE_SDA, GROVE_SCL);
}

void setup() {
  Serial.begin(115200);
  delay(2000);           // Only for debugging
  powerEnableSensors();  // M5AirQ enable sensors

  delay(100);
  Serial.println("\n== Sensor test setup ==\n");
  Serial.println("-->[SETUP] Detecting sensors..");

  sensors.setSampleTime(10);                      // config sensors sample time interval
  sensors.setOnDataCallBack(&onSensorDataOk);     // all data read callback
  sensors.setDebugMode(false);                    // [optional] debug mode
  sensors.detectI2COnly(true);                    // not force to only i2c sensors
  sensors.setTemperatureUnit(TEMPUNIT::CELSIUS);  // comment for Celsius or set Fahrenheit
  sensors.init();                                 // Auto detection (UART and i2c sensors)
  delay(1000);
}

void loop() {
  sensors.loop();  // read sensor data and showed it
  delay(100);
}
