/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2020
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
    Serial.print ("-->[MAIN] PM1.0: "+sensors.getStringPM1());
    Serial.print (" PM2.5: " + sensors.getStringPM25());
    Serial.print (" PM10: " + sensors.getStringPM10());
    Serial.print (" PM1: " + sensors.getStringPM1());
}

void onSensorDataError(const char * msg){ 
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

    sensors.setSampleTime(5);                       // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);     // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError); // [optional] error callback
    sensors.setDebugMode(true);                     // [optional] debug mode

    sensors.init(sensors.Sensirion);

    if(sensors.isPmSensorConfigured())
        Serial.println("-->[SETUP] Sensor configured: " + sensors.getPmDeviceSelected());

    delay(500);
}

uint64_t counter = 0;

void loop() {

    counter++;

    if (counter < 5000000) sensors.loop();  // read sensor data and showed it

    if (counter == 5000000) {
        Serial.println("sleep sensirion..");
        sensors.sps30.sleep();
    }
    if (counter == 100000000) {
        Serial.println("wakeup sensirion..");
        sensors.sps30.wakeup();
        counter = 0;
    }
}