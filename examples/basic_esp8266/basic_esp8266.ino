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
    Serial.print("-->[MAIN] PM1.0: "+sensors.getStringPM1());
    Serial.print  (" PM2.5: " + sensors.getStringPM25());
    Serial.println(" PM10: " + sensors.getStringPM10());
    Serial.println(" PM1: " + sensors.getStringPM1());
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

    // sensors.init();                              // Auto detection of PM sensors (Honeywell, Plantower, Panasonic)
    // sensors.init(sensors.Auto);                  // Auto detection of PM sensors (Honeywell, Plantower, Panasonic)
    // sensors.init(sensors.Panasonic);             // Force detection to Panasonic sensor
    // sensors.init(sensors.Sensirion);             // Force detection to Sensirion sensor
    // sensors.init(sensors.Mhz19);                 // Force detection to Mhz14 or Mhz19 CO2 sensor
    // sensors.init(sensors.SDS011);                // Force detection to SDS011 sensor
    // sensors.init(sensors.CM1006);                // Force detection to CM1106 CO2 sensor
    // sensors.init(sensors.Auto,mRX,mTX);          // Auto detection and custom RX, TX pines
    // sensors.init(sensors.Auto,PMS_RX,PMS_TX);    // Auto detection, custom RX,TX and custom DHT config

    sensors.init(sensors.Panasonic, 5, 6);          // Panasonic sensor with ESP8266 software serial pines

    if(sensors.isPmSensorConfigured())
        Serial.println("-->[SETUP] Sensor configured: " + sensors.getPmDeviceSelected());

    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}