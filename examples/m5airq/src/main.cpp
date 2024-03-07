/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2024
 * @brief CanAirIO M5AirQ test
 * @license GPL3
 * 
 * Full documentation:
 * https://github.com/kike-canaries/canairio_sensorlib#canairio-air-quality-sensors-library
 * 
 * Full implementation for WiFi and Bluetooth Air Quality fixed and mobile station:
 * https://github.com/kike-canaries/canairio_firmware#canairio-firmware
 * 
 * CanAirIO project documentation:
 * https://canair.io/docs
 */

#include <Arduino.h>
#include <Sensors.hpp>

#define MAIN_HW_EN_PIN 10 // M5AirQ sensor hardware enable

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
        Serial.printf("-->[MAIN] %s:\t%02.1f\t%s\n", uName.c_str(), uValue, uSymb.c_str());
        unit = sensors.getNextUnit();
    }
}

void onSensorDataOk() {
    Serial.println("======= E X A M P L E   T E S T =========");
    printSensorsDetected();
    printSensorsValues(); 
}

void onSensorDataError(const char * msg){ 
}
/******************************************************************************
*  M A I N
******************************************************************************/

void powerEnableSensors() {
    Serial.println("-->[POWR] == enable sensors ==");
    pinMode(MAIN_HW_EN_PIN, OUTPUT);
    digitalWrite(MAIN_HW_EN_PIN, HIGH);  // step-up on
}

void setup() {
    Serial.begin(115200);
    delay(2000);           // Only for debugging
    powerEnableSensors(); // M5AirQ enable sensors
    delay(100);
    Serial.println("\n== Sensor test setup ==\n");
    Serial.println("-->[SETUP] Detecting sensors..");
    
    sensors.setSampleTime(10);                       // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);      // all data read callback
    sensors.setDebugMode(false);                     // [optional] debug mode
    sensors.detectI2COnly(false);                    // not force to only i2c sensors
    sensors.setTemperatureUnit(TEMPUNIT::CELSIUS);   // comment for Celsius or set Fahrenheit
    sensors.init();                                  // Auto detection (UART and i2c sensors)
    delay(1000);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}
