/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
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
    Serial.print ("-->[MAIN] PM1.0: "+String(sensors.getPM1()));
    Serial.print (" PM2.5: " + String(sensors.getPM25()));
    Serial.print (" PM10 : " + String(sensors.getPM10()));
    Serial.println(" PM1.0: " + String(sensors.getPM1()));
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

    sensors.init(SENSORS::SSPS30);                  // Forced Sensirion via UART, default is I2C
                                                    // For Sensirion with i2c only leave empty this parameter

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