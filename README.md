
[![PlatformIO](https://github.com/kike-canaries/canairio_sensorlib/workflows/PlatformIO/badge.svg)](https://github.com/kike-canaries/canairio_sensorlib/actions/) [![Build Status](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master)](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master) ![ViewCount](https://views.whatilearened.today/views/github/kike-canaries/canairio_sensorlib.svg) 

# CanAirIO Air Quality Sensors Library

Particle meter (PM) sensor manager for multiple (PM) sensors: Honeywell, Plantower, Panasonic, Sensirion, etc, also it handling others like AM2320 sensor.

## Features

- [x] Auto detection for Generic sensors (Honeywell, Panasonic and Plantower sensors)
- [x] Implemented `Sensirion` autodection flow (for original library)
- [x] Disable/enable logs (debug mode flag)
- [x] Debugging compatible with ESP32 log level
- [x] Added bme280, aht10, sht31, am2320 i2c sensors
- [x] Exposed public sub-libraries objects, sps30, aht10, etc.
- [ ] Added old DHT sensors 
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

    // Also you can access to special public objects, for example:
    // sensors.sps30.sleep()
    // sensors.bme.readPressure();


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

### Output

On your serial monitor you should have something like that:

```bash
-->[SETUP] Detecting sensors..
-->[SETUP] Sensor configured: SENSIRION
-->[MAIN] PM1.0: 002 PM2.5: 004 PM10: 006
-->[MAIN] PM1.0: 002 PM2.5: 002 PM10: 002
-->[MAIN] PM1.0: 002 PM2.5: 002 PM10: 002
```

## Demo

[![CanAirIO auto configuration demo](https://img.youtube.com/vi/hmukAmG5Eec/0.jpg)](https://www.youtube.com/watch?v=hmukAmG5Eec)

CanAirIO sensorlib auto configuration demo on [Youtube](https://www.youtube.com/watch?v=hmukAmG5Eec)

---

## Examples

### PlatformIO

#### Compiling and Installing

Please install first [PlatformIO](http://platformio.org/) open source ecosystem for IoT development compatible with **Arduino** IDE and its command line tools (Windows, MacOs and Linux), then connect your compatible board to the USB and run the next command:

```python
pio run --target upload
```

### Arduino

#### Prerequisites

You first need to run the examples, install **arduino-cli** or the **Arduino IDE** with the following libraries:

Adafruit Unified Sensor  
Adafruit AM2320 sensor library  
[Sensirion library sps30](https://github.com/paulvha/sps30)  
`CanAirIO Air Quality Sensors Library` (this library).

Also you need to add the **alternative links** for supporting the ESP32 boards:

```bash
arduino-cli config init
```

in the `.arduino15/arduino-cli.yaml` file add:

```yml
board_manager:
  additional_urls:
    - https://arduino.esp8266.com/stable/package_esp8266com_index.json
    - https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
``` 

#### Compiling and Installing

From `arduino-cli` you can run the basic example in a ESP32 board following these steps:

```javascript
arduino-cli core update-index
arduino-cli core install esp32:esp32:lolin32
arduino-cli compile --fqbn esp32:esp32:lolin32 basic
arduino-cli upload --fqbn esp32:esp32:lolin32:UploadSpeed=115200 -p /dev/ttyUSB0 basic
```

where `basic` is the basic example on examples directory.

---

## Credits

Thanks to all collaborators and [CanAirIO](https://canair.io) community for testing and reports.
