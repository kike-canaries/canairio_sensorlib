/**
 * @file main.cpp
 * @date June 2018 - 2021
 * @brief CO2 mhz19 sensor example
 * @license GPL3
 *
 * @license GPL3
 * 
 * Full documentation:
 * https://github.com/kike-canaries/canairio_sensorlib#canairio-air-quality-sensors-library
 * 
 * Full implementation for WiFi and Bluetooth Air Quality fixed and mobile station:
 * https://github.com/kike-canaries/canairio_firmware#canairio-firmware
 * 
 * CanAirIO project docs:
 * https://canair.io/docs
 */

#include <Arduino.h>

#include <Sensors.hpp>

void onSensorDataOk() {
    Serial.print(" CO2: " + String(sensors.getCO2()));
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
    sensors.setDebugMode(false);                     // [optional] debug mode

    // sensors.setCO2RecalibrationFactor(400);       // calibration method (in outdoors)

    sensors.init(SENSORS::SAIRS8);                // forced UAQ sensor. Empty for auto detection
    
    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}