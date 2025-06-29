/**
 * @file main.cpp
 * @date June 2018 - 2021
 * @brief Particle meter sensor tests
 * @license GPL3
 * 
 * Full documentation:
 * https://github.com/kike-canaries/canairio_sensorlib#canairio-air-quality-sensors-library
 * 
 * Full implementation for WiFi and Bluetooth Air Quality fixed and mobile station:
 * https://github.com/kike-canaries/canairio_firmware#canairio-firmware
 * 
 * CanAirIO project:
 * https://canair.io
 */

#include <Arduino.h>

#include <Sensors.hpp>

void onSensorDataOk() {
    Serial.println("-->[MAIN] NH3: " + String(sensors.getNH3()));
    Serial.println("-->[MAIN] CO: " + String(sensors.getCO()));
    Serial.println("-->[MAIN] NO2: " + String(sensors.getNO2()));
    Serial.println("-->[MAIN] O3: " + String(sensors.getO3()));
}

void onSensorDataError(const char* msg) {
    Serial.println(msg);
}

/******************************************************************************
*  M A I N
******************************************************************************/

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n== Sensor test setup ==\n");

    Serial.println("-->[SETUP] Detecting sensors..");

    sensors.setSampleTime(5);                        // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);      // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError);  // [optional] error callback
    sensors.setDebugMode(false);                     // [optional] debug mode
    sensors.detectI2COnly(true);                     // force to only i2c sensors

    sensors.init(SENSORS::SDFRCO);                         // detect CO sensor
    sensors.init(SENSORS::SDFRNH3);                        // detect NH3 sensor
    sensors.init(SENSORS::SDFRNO2);                        // detect NO2 sensor
    sensors.init(SENSORS::SDFRO3);                         // detect O3 sensor
    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}