/**
 * @file main.cpp
 * @authors @roberbike @iw2lsi @hpsaturn 
 * @date June 2018 - 2023
 * @brief Radiation sensor example
 * @license GPL3
 * 
 * Full documentation:
 * https://github.com/kike-canaries/canairio_sensorlib#canairio-air-quality-sensors-library
 * 
 * Full implementation for WiFi and Bluetooth Air Quality fixed and mobile station:
 * https://github.com/kike-canaries/canairio_firmware#canairio-firmware
 * 
 * Main pull requests and discussions:
 * https://github.com/kike-canaries/canairio_sensorlib/pull/144
 * https://github.com/kike-canaries/canairio_firmware/pull/226
 * 
 * CanAirIO project:
 * https://canair.io
 * 
 * CanAirIO Docs:
 * https://canair.io/docs
 * 
 */

#include <Arduino.h>
#include <Sensors.hpp>

void onSensorDataOk() {
    Serial.print(" CPM: " + String(sensors.getGeigerCPM()));
    Serial.print(" uSvh: " + String(sensors.getGeigerMicroSievertHour()));
}

void onSensorDataError(const char* msg) {
    Serial.println(msg);
}

/******************************************************************************
*  M A I N
******************************************************************************/

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n== Sensor test setup ==\n");
    Serial.println("-->[SETUP] Detecting sensors..");

    sensors.setSampleTime(5);                         // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);       // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError);   // [optional] error callback
    sensors.setDebugMode(true);                       // [optional] debug mode
    sensors.detectI2COnly(true);                      // [optional] skip UART detection
    sensors.enableGeigerSensor(27);                   // Geiger in sensor pin 27

    sensors.init();                                   // forced UAQ sensor. Empty for auto detection
    
    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}


