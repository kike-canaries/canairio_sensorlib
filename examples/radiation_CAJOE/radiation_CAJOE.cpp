/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2022
 * @brief Radiation sensor example
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
 * 
 * CanAirIO Docs:
 * https://canair.io/docs
 * 
 */

#include <Arduino.h>
#include <Sensors.hpp>

void onSensorDataOk() {
    Serial.print(" CO2: " + sensors.getStringCO2());
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
    delay(1000);

    Serial.println("\n== Sensor test setup ==\n");
    Serial.println("-->[SETUP] Detecting sensors..");

    sensors.setSampleTime(5);                         // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);       // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError);   // [optional] error callback
    sensors.setDebugMode(true);                       // [optional] debug mode
    sensors.detectI2COnly(true);                      // [optional] skip UART detection

    sensors.init();                                   // forced UAQ sensor. Empty for auto detection
    
    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}


