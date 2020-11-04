[![PlatformIO](https://github.com/kike-canaries/canairio_sensorlib/workflows/PlatformIO/badge.svg)](https://github.com/kike-canaries/canairio_sensorlib/actions/)

# PlatformIO sample

Example for **CanAirIO sensors library**, it is a particle meter (**PM**) sensor manager for multiple (PM) sensors: **Honeywell, Plantower, Panasonic, Sensirion**, etc, also it handling others like **AM2320** sensor.

You can review a **full implementation** on [CanAirIO project firmware](https://github.com/kike-canaries/canairio_firmware/blob/master/src/main.cpp), but a little brief is the next:

## Implementation

```Java

#include <Arduino.h>
#include <Sensors.hpp>

void onSensorDataOk() {
    Serial.print  (" PM1.0: " + sensors.getStringPM1());
    Serial.print  (" PM2.5: " + sensors.getStringPM25());
    Serial.println(" PM10 : " + sensors.getStringPM10());
}

void onSensorDataError(const char * msg){
    Serial.println(msg);
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n== Sensor test setup ==\n");
    Serial.println("-->[SETUP] Detecting sensors..");

    sensors.setSampleTime(5);                       // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);     // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError); // [optional] error callback
    sensors.setDebugMode(false);                    // [optional] debug mode
    sensors.init();                                 // start all sensors and 
                                                    // force to try autodetection, 
                                                    // you can try to select one:
    // sensors.init(sensors.Sensirion);

    if(sensors.isPmSensorConfigured())
        Serial.println("-->[SETUP] Sensor configured: " + sensors.getPmDeviceSelected());

    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}
```

### Custom RX/TX pines

```javascript
void setup() {
    sensors.init(0,RX,TX); // generic sensor(default), custom RX, custom TX pines.
}
```

## Credits

Thanks to all collaborators and [CanAirIO](https://canair.io) community for testing and reports.
