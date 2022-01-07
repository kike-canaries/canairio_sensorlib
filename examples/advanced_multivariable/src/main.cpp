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

uint16_t mainValue = 0;
uint16_t minorValue = 0;
String uSymbol = "";
String uName = "";
UNIT nextUnit = UNIT::NUNIT;

/**
 * Example or alternative of the selection of the main sensor unit
 */
void getMainValue() {
    // If the main sensor (CO2 or PM2.5) was not detected and temperature is registered
    if (sensors.getMainDeviceSelected().isEmpty() && sensors.isUnitRegistered(UNIT::TEMP)) {  
        mainValue = (uint32_t) sensors.getTemperature();
        uName = sensors.getUnitName(UNIT::TEMP);
        uSymbol = sensors.getUnitSymbol(UNIT::TEMP);
    // The main sensor is a particle meter device
    } else if (sensors.getMainSensorTypeSelected() == Sensors::SENSOR_PM) {
        mainValue = sensors.getPM25();
        uName = sensors.getUnitName(UNIT::PM25);
        uSymbol = sensors.getUnitSymbol(UNIT::PM25);
    // The main sensor is a CO2 device
    } else if (sensors.getMainSensorTypeSelected() == Sensors::SENSOR_CO2) {
        mainValue = sensors.getCO2();
        uName = sensors.getUnitName(UNIT::CO2);
        uSymbol = sensors.getUnitSymbol(UNIT::CO2);
    }
}

/**
 * Example or alternative of the selection of the minor sensor unit
 */

void getMinorValue(UNIT mainUnit) {
    minorValue = (uint32_t)sensors.getUnitValue(mainUnit);
    uName = sensors.getUnitName(mainUnit);
    uSymbol = sensors.getUnitSymbol(mainUnit);
}


void onSensorDataOk() {
    
    Serial.println("");

    getMainValue(); // choose the main sensor possible
    Serial.println ("-->[MAIN] Main sensor unit     \t: "+uName+": "+String(mainValue)+" "+uSymbol);

    getMinorValue(nextUnit); // Load values ot the minor sensor. (see the loop)
    Serial.println ("-->[MAIN] Secondary sensor unit\t: "+uName+": "+String(minorValue)+" "+uSymbol);

    Serial.println("");
    
    nextUnit = (UNIT)sensors.getNextUnit(); // Change the minor sensor

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

    // enable possible sensors on pin 27
    pinMode(27, OUTPUT);
    digitalWrite(27, HIGH);

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
 
    Serial.println("-->[SETUP] Main sensor configured: " + sensors.getMainDeviceSelected());

    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}