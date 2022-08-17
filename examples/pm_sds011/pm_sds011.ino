/**
 * @file main.cpp
 * @date 2018 - 2022
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
    Serial.print("-->[MAIN] PM1: " + String(sensors.getPM1()));
    Serial.print(" PM2.5: " + String(sensors.getPM25()));
    Serial.print(" PM10: " + String(sensors.getPM10()));

    Serial.print(" CO2: " + String(sensors.getCO2()));
    Serial.print(" CO2humi: " + String(sensors.getCO2humi()));
    Serial.print(" CO2temp: " + String(sensors.getCO2temp()));

    Serial.print(" H: " + String(sensors.getHumidity()));
    Serial.println(" T: " + String(sensors.getTemperature()));
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

    sensors.init(SENSORS::SDS011);                    // forced for SDS011 via UART, empty for auto detection


    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}