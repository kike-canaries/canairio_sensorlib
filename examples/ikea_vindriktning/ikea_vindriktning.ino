/**
 * @file main.cpp
 * @date June 2018 - 2022
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
    Serial.println("-->[MAIN] PM2.5: " + String(sensors.getPM25()));
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
    sensors.setDebugMode(true);                      // [optional] debug mode
    sensors.init(SENSORS::IKEAVK);                   // forced for IKEAVK via UART, empty for auto detection


    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}