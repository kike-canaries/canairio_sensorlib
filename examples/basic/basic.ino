/**
 * @file basic.ino
 * @author Antonio Vanegas @hpsaturn
 * @date CanAirIO sensorslib 2018 - 2020
 * @brief Arduino particle meter sensor example
 * @license GPL3
 */

#include <Sensors.hpp>

void onSensorDataOk() {
    Serial.print("-->[MAIN] PM1.0: "+sensors.getStringPM1());
    Serial.print  (" PM2.5: " + sensors.getStringPM25());
    Serial.println(" PM10: " + sensors.getStringPM10());
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
    sensors.setDebugMode(false);                    // [optional] debug mode
    sensors.init(sensors.Sensirion);                // force sensor detection for Sensirion SPS30

    // You also can try init() with:
    // sensors.init()  => auto detection PM sensors (Honeywell, Plantower or Panasonic)
    // sensors.init(sensors.Sensirion) => only auto detection for Sensirion
    // sensors.init(sensors.Honeywell,21,22) => force RX,TX pines

    if(sensors.isPmSensorConfigured())
        Serial.println("-->[SETUP] Sensor configured: " + sensors.getPmDeviceSelected());

    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}

