
[![PlatformIO](https://github.com/kike-canaries/canairio_sensorlib/workflows/PlatformIO/badge.svg)](https://github.com/kike-canaries/canairio_sensorlib/actions/) [![Build Status](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master)](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master) ![ViewCount](https://views.whatilearened.today/views/github/kike-canaries/canairio_sensorlib.svg) 

# CanAirIO Air Quality Sensors Library

Particle meter (PM) sensor manager for multiple (PM) sensors: Honeywell, Plantower, Panasonic, Sensirion, etc, also it handling others like AM2320 sensor.

## Features

- [x] Auto detection for Generic sensors (Honeywell, Panasonic and Plantower sensors)
- [x] Implemented `Sensirion` autodection flow (for original library)
- [x] Disable/enable logs (debug mode flag)
- [x] Added bme280, aht10, sht31, am2320 i2c sensors
- [x] Exposed public sub-libraries objects, sps30, aht10, etc.
- [x] Added old DHT sensors 
- [x] Added CO2 sensors: MHZ19, SCD30, CM1106
- [x] Added SDS011 particle metter
- [x] BME680 support (from TTGO-T7 CanAirIO version)
- [ ] IAQ indicator from BME680 Bosch library

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
                                                    // All i2c sensors are autodetected.

    // Also you can access to special library objects, for example some calls:

    // sensors.sps30.sleep()
    // sensors.bme.readPressure();
    // sensors.mhz19.getRange();
    // sensors.scd30.getTemperatureOffset();
    // sensors.aht10.readRawData();


    if(sensors.isPmSensorConfigured())
        Serial.println("-->[SETUP] Sensor configured: " + sensors.getPmDeviceSelected());

    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
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

### PlatformIO (recommended)

#### Compiling and Installing

We recommended PlatformIO because is more easy than Arduino IDE. For this, please install first [PlatformIO](http://platformio.org/) and its command line tools (Windows, MacOs and Linux), **pio** command, then connect your compatible board to the USB and run the next command:

```python
pio run --target upload
```

### Arduino

#### Prerequisites

For run the examples, you first need to  install **arduino-cli** or the **Arduino IDE** with the libraries referenced in **lib_deps** on the file [platformio.ini](https://github.com/kike-canaries/canairio_sensorlib/blob/fix_ondataready_cb/platformio.ini), becuase **Arduino don't install it automatically** like PlatformIO. Then put CanAirIO sensor library in your library directory, you can download it from [releases](https://github.com/kike-canaries/canairio_sensorlib/releases) section.

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

## Wiring

The current version of library supports 3 kinds of wiring connection, UART, i2c and TwoWire, in the main boards the library using the defaults pins of each board, but in some special cases the pins are:

### UART

```cpp
#ifdef WEMOSOLED
#define PMS_RX 13  // config for Wemos board & TTGO18650
#define PMS_TX 15  // some old TTGO18650 have PMS_RX 18 & PMS_TX 17
#elif HELTEC
#define PMS_RX 17  // config for Heltec board, ESP32Sboard & ESPDUINO-32
#define PMS_TX 18  // some old ESP32Sboard have PMS_RX 27 & PMS_TX 25
#elif TTGO_TQ
#define PMS_RX 13  // config for TTGO_TQ board
#define PMS_TX 18
#else
#define PMS_RX 17  // config for D1MIN1 board (Default for main ESP32 dev boards)
#define PMS_TX 16
#endif
```
Also you can define the UART pins in the init() method, please see below.  

#### Custom UART RX/TX:

You can pass the custom pins if it isn't autodected:

```cpp
sensors.init(sensors.Auto,RX,TX); // custom RX, custom TX pines.
```



### i2c (recommended)

We are using the default pins for each board, some times it's pins are 21,22, please check your board schematic.

### TwoWire

For now we are using it only for DHT sensors in PIN 23. For more info please review the next lines [here](https://github.com/kike-canaries/canairio_sensorlib/blob/01ae9e976aa2985bb66856c0d6fae2a03100f552/src/Sensors.hpp#L16-L45).


---

## Credits

Thanks to all collaborators and [CanAirIO](https://canair.io) community for testing and reports.
