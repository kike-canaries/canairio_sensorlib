/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2022
 * @brief CanAirIO Sensorslib tests
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

#define PIN_POWER_ON 15  // T-Display S3 Power pin

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

void setup() {
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    delay(1000);                                     // Wait on T-Display S3 power on 
    Serial.begin(115200);
    delay(5000);                                     // Wait on T-Display S3 power on 
    Serial.println("\n== Sensor test setup ==\n");
    Serial.println("-->[SETUP] Detecting sensors..");

    Wire.begin(43,44);                               // enable i2c for T-Display S3 board
    
    sensors.setSampleTime(5);                        // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);      // all data read callback
    sensors.setDebugMode(true);                     // [optional] debug mode
    sensors.detectI2COnly(true);                     // disable force to only i2c sensors
    sensors.init();                                  // Auto detection to UART and i2c sensors
    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}