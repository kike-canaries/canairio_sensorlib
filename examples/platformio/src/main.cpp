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
 * CanAirIO project docs:
 * https://canair.io/docs
 */

#include <Arduino.h>
#include <Sensors.hpp>

void printSensorsDetected() {
    uint16_t sensors_count =  sensors.getSensorsRegisteredCount();
    uint16_t units_count   =  sensors.getUnitsRegisteredCount();
    Serial.println("-->[MAIN] Sensors detected count\t: " + String(sensors_count));
    Serial.println("-->[MAIN] Sensors units count  \t: " + String(units_count));
    Serial.print(  "-->[MAIN] Sensors devices names\t: ");
    int i = 0;
    while (sensors.getSensorsRegistered()[i++] != 0) {
        Serial.print(sensors.getSensorName((SENSORS)sensors.getSensorsRegistered()[i - 1]));
        Serial.print(",");
    }
    Serial.println();
}

void printSensorsValues() {
    Serial.println("-->[MAIN] Preview sensor values:");
    UNIT unit = sensors.getNextUnit();
    while(unit != UNIT::NUNIT) {
        String uName = sensors.getUnitName(unit);
        float uValue = sensors.getUnitValue(unit);
        String uSymb = sensors.getUnitSymbol(unit);
        Serial.println("-->[MAIN] " + uName + " \t: " + String(uValue) + " " + uSymb);
        unit = sensors.getNextUnit();
    }
}

void onSensorDataOk() {
    Serial.println("======= E X A M P L E   T E S T =========");
    printSensorsDetected();
    printSensorsValues(); 
    Serial.println("=========================================");
}

void onSensorDataError(const char * msg){ 
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
    sensors.detectI2COnly(false);                   // disable force to only i2c sensors
    sensors.init();                                 // Auto detection to UART and i2c sensors

    // Alternatives only for UART sensors (TX/RX):

    // sensors.init(sensors.Auto);                  // Auto detection to UART sensors (Honeywell, Plantower, Panasonic)
    // sensors.init(sensors.Panasonic);             // Force UART detection to Panasonic sensor
    // sensors.init(sensors.Sensirion);             // Force UART detection to Sensirion sensor
    // sensors.init(sensors.Mhz19);                 // Force UART detection to Mhz14 or Mhz19 CO2 sensor
    // sensors.init(sensors.SDS011);                // Force UART detection to SDS011 sensor
    // sensors.init(sensors.CM1106);                // Force UART detection to CM1106 CO2 sensor
    // sensors.init(sensors.SENSEAIRS8);            // Force UART detection to SenseAirS8 CO2 sensor
    // sensors.init(sensors.Auto,PMS_RX,PMS_TX);    // Auto detection on custom RX,TX
  

}

void loop() {
    sensors.loop();  // read sensor data and showed it
}