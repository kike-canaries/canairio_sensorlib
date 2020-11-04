
[![PlatformIO](https://github.com/kike-canaries/canairio_sensorlib/workflows/PlatformIO/badge.svg)](https://github.com/kike-canaries/canairio_sensorlib/actions/) [![Build Status](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master)](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master)

# CanAirIO Air Quality Sensors Library

Particle meter (PM) sensor manager for multiple (PM) sensors: Honeywell, Plantower, Panasonic, Sensirion, etc, also it handling others like AM2320 sensor.

## Features

- [x] Auto detection for Generic sensors (Honeywell, Panasonic and Plantower sensors)
- [x] Implemented `Sensirion` autodection flow (for original library)
- [x] Disable/enable logs (debug mode flag)
- [x] Debugging compatible with ESP32 log level
- [ ] BME680 support (from TTGO-T7 CanAirIO version)

## Usage

### Quick implementation

```Java
sensors.setOnDataCallBack(&onSensorDataOk);   // all data read callback
sensors.init();                               // start all sensors and
                                              // try to detect PM sensor: 
                                              // Panasonic, Honeywell or Plantower.
                                              // for Sensirion please do:
                                              // init(sensors.Sensirion)
```

### Full implementation

You can review a full implementation on [CanAirIO project firmware](https://github.com/kike-canaries/canairio_firmware/blob/master/src/main.cpp), but a little brief is the next:

```Java
/// sensors data callback
void onSensorDataOk() {
    Serial.print  (" PM1.0: " + sensors.getStringPM1());  // some fields sample
    Serial.print  (" PM2.5: " + sensors.getStringPM25());
    Serial.println(" PM10: "  + sensors.getStringPM10());
}

/// sensors error callback
void onSensorDataError(const char * msg){
    Serial.println(msg);
}

void setup() {

    sensors.setSampleTime(5);                       // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);     // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError); // [optional] error callback
    sensors.setDebugMode(true);                     // [optional] debug mode
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
