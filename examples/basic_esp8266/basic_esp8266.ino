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
    Serial.print (" PM1.0: " + String(sensors.getPM1())); 
    Serial.print (" CO2: " + String(sensors.getCO2()));
    Serial.println(" T: " + String(sensors.getTemperature()));
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

    // Alternatives only for UART sensors (TX/RX):

    // sensors.init(SENSORS::Auto);                    // Auto detection to UART sensors (Honeywell, Plantower, Panasonic)
    // sensors.init(SENSORS::SGCJA5);                  // Force UART detection to Panasonic sensor
    // sensors.init(SENSORS::SSPS30);                  // Force UART detection to Sensirion sensor
    // sensors.init(SENSORS::SMHZ19);                  // Force UART detection to Mhz14 or Mhz19 CO2 sensor
    // sensors.init(SENSORS::SDS011);                  // Force UART detection to SDS011 sensor
    // sensors.init(SENSORS::IKEAVK);                  // Force UART detection to IKEA Vindriktning sensor
    // sensors.init(SENSORS::SCM1106);                 // Force UART detection to CM1106 CO2 sensor
    // sensors.init(SENSORS::SAIRS8);                  // Force UART detection to SenseAirS8 CO2 sensor
    // sensors.init(SENSORS::Auto,PMS_RX,PMS_TX);      // Auto detection on custom RX,TX
   

    sensors.init(SENSORS::SGCJA5, 5, 6);            // Panasonic sensor with ESP8266 software serial pines
                                                    // default is empty (I2C pines)
    
    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}