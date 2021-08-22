
[![PlatformIO](https://github.com/kike-canaries/canairio_sensorlib/workflows/PlatformIO/badge.svg)](https://github.com/kike-canaries/canairio_sensorlib/actions/) [![Build Status](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master)](https://travis-ci.com/kike-canaries/canairio_sensorlib.svg?branch=master) ![ViewCount](https://views.whatilearened.today/views/github/kike-canaries/canairio_sensorlib.svg) 

# Air Quality Sensors Library

Generic sensor manager, abstratctions and bindings of multiple sensors libraries: Honeywell, Plantower, Panasonic, Sensirion, etc and CO2 sensors. Also it handling others environment sensors. This library is for general purpose but also is the sensors library base of [CanAirIO project](https://canair.io/docs).

# Supported sensors

### PM sensors

| Sensor model  | UART  | I2C  | Detection mode | Status |  
|:----------------------- |:-----:|:-----:|:-------:|:----------:|
| Honeywell HPMA115S0 | Yes | --- | Auto | DEPRECATED |
| Panasonic SN-GCJA5L | Yes | Yes | Auto | STABLE |
| Plantower models    | Yes | --- | Auto | STABLE |
| Nova SDS011         | Yes | --- | Auto | STABLE |
| Sensirion SPS30     | Yes | Yes | Select / Auto | STABLE |

NOTE: Panasonic via UART in ESP8266 maybe needs select in detection

### CO2 sensors

| Sensor model  | UART  | i2c  | Detection mode | Status |  
|:----------------------- |:-----:|:-----:|:-------:|:----------:|
| Sensirion SCD30    | --- | Yes | Auto | STABLE |
| MHZ19      | Yes | --- | Select | STABLE |
| CM1106    | Yes | --- | Select | STABLE |
| SenseAir S8 | Yes | --- | Select | TESTING |


### Environmental sensors

| Sensor model  | Protocol  | Detection mode | Status |  
|:----------------------- |:-----:|:-------:|:----------:|
| AM2320      | i2c |  Auto | STABLE |
| SHT31       | i2c |  Auto | STABLE |
| AHT10       | i2c |  Auto | STABLE |
| BME280      | i2c |  Auto | STABLE |
| BME680      | i2c |  Auto | STABLE |
| DHTxx       | TwoWire |  Auto | DEPRECATED |

NOTE: DHT22 is supported but is not recommended

# Features

- Unified variables for all sensors 
- Auto UART port selection (Hw, Sw, UART1, UART2, etc)
- Multiple i2c reads and one UART sensor read support
- Preselected main stream UART pins from popular boards
- Auto config UART port for Plantower, Honeywell and Panasonic sensors
- Unified calibration trigger for all CO2 sensors
- Public access to main objects of each library (full methods access)
- Basic debug mode support toggle in execution

Full list of all sub libraries supported [here]()


# Quick implementation

```Java
sensors.setOnDataCallBack(&onSensorDataOk);   // all data read callback
sensors.init();                               // start all sensors and
                                              // try to detect UART sensors like:
                                              // Panasonic, Honeywell or Plantower.
                                              // For special UART sensors try select it:
                                              // init(sensors.Sensirion)
                                              // init(sensors.Mhz19)
                                              // init(sensors.CM1106)
                                              // init(sensors.SENSEAIRS8)
                                              // For I2C sensors, with empty parameter is enough.
```

# Full implementation

You can review a full implementation on [CanAirIO project firmware](https://github.com/kike-canaries/canairio_firmware/blob/master/src/main.cpp), but a little brief is the next:

```Java
/// sensors data callback
void onSensorDataOk() {
    Serial.print  (" PM1.0: " + sensors.getStringPM1());  // some fields sample
    Serial.print  (" PM2.5: " + sensors.getStringPM25());
    Serial.println(" PM10: "  + sensors.getStringPM10());
    Serial.println(" CO2:  "  + sensors.getStringCO2());
}

/// sensors error callback
void onSensorDataError(const char * msg){
    Serial.println(msg);
}

void setup() {

    sensors.setSampleTime(5);                       // Config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);     // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError); // [optional] error callback
    sensors.setSampleTime(15);                      // [optional] sensors sample time (default 5s)
    sensors.setDebugMode(false);                    // [optional] debug mode enable/disable
    sensors.detectI2COnly(true);                    // [optional] force to only i2c sensors
    sensors.init();                                 // start all sensors with auto detection mode.
                                                    // Also you can try to select one:
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

## Output

On your serial monitor you should have something like that:

```bash
-->[SETUP] Detecting sensors..
-->[SETUP] Sensor configured: SENSIRION
-->[MAIN] PM1.0: 002 PM2.5: 004 PM10: 006
-->[MAIN] PM1.0: 002 PM2.5: 002 PM10: 002
-->[MAIN] PM1.0: 002 PM2.5: 002 PM10: 002
```

# Demo

[![CanAirIO auto configuration demo](https://img.youtube.com/vi/hmukAmG5Eec/0.jpg)](https://www.youtube.com/watch?v=hmukAmG5Eec)

CanAirIO sensorlib auto configuration demo on [Youtube](https://www.youtube.com/watch?v=hmukAmG5Eec)


# Wiring

The current version of library supports 3 kinds of wiring connection, UART, i2c and TwoWire, in the main boards the library using the defaults pins of each board, but in some special cases the pins are:

### UART

The library has [pre-defined some UART pin configs](https://github.com/kike-canaries/canairio_sensorlib/blob/master/src/Sensors.hpp#L19-L52), these are selected on compiling time. Maybe you don't need change anything with your board.  

Also you can define the UART pins in the init() method, please see below.  

#### Custom UART RX/TX:

You can pass the custom pins if it isn't autodected:

```cpp
sensors.init(sensors.Auto,RX,TX); // custom RX, custom TX pines.
```

### I2C (recommended)

We are using the default pins for each board, some times it's pins are 21,22, please check your board schematic.

### TwoWire (deprecated soon)

For now we are using it only for DHT sensors in PIN 23. For more info please review the next lines [here](https://github.com/kike-canaries/canairio_sensorlib/blob/master/src/Sensors.hpp#L19-L52).


# Examples

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

# Supporting the project

If you want to contribute to the code or documentation, consider posting a bug report, feature request or a pull request.

When creating a pull request, we recommend that you do the following:

- Clone the repository
- Create a new branch for your fix or feature. For example, git checkout -b fix/my-fix or git checkout -b feat/my-feature.
- Run to any clang formatter if it is a code, for example using the `vscode` formatter. We are using Google style. More info [here](https://clang.llvm.org/docs/ClangFormatStyleOptions.html)
- Document the PR description or code will be great
- Target your pull request to be merged with `devel` branch

Also you can make a donation, be a patreon or buy a device:  

<a href="https://raw.githubusercontent.com/kike-canaries/canairio_firmware/master/images/ethereum_donation_address.png" target="_blank"><img src="https://raw.githubusercontent.com/kike-canaries/canairio_firmware/master/images/ethereum_donation_address.png" align="right" width="220" margin-left="10px" ></a>

- Via **Ethereum**: `0x1779cD3b85b6D8Cf1A5886B2CF5C53a0E072C108`
- Via **Liberapay**: [CanAirIO in LiberaPay](https://liberapay.com/CanAirIO)
- **Buy a device**: [CanAirIO Bike in Tindie](https://www.tindie.com/products/hpsaturn/canairio-bike/)
- [Inviting us **a coffee**](https://www.buymeacoffee.com/hpsaturn) 


# TODO

- [x] Auto detection for UART sensors (Honeywell, Panasonic and Plantower)
- [x] Added SPS30 library with auto UART detection
- [x] Disable/enable logs (debug mode flag)
- [x] Added bme280, aht10, sht31, am2320 i2c sensors
- [x] Exposed public sub-libraries objects, sps30, aht10, etc.
- [x] Added old DHT sensors 
- [x] Added CO2 sensors: MHZ19, SCD30, CM1106 via UART
- [x] Added SDS011 particle metter
- [x] BME680 support (from TTGO-T7 CanAirIO version)
- [x] Added Sensirion SPS30 and Panasonic SN-GCJA5 via i2c
- [x] Enable/Disable UART detection for force only i2c
- [ ] IAQ indicator from BME680 Bosch library




# Credits

Thanks to all collaborators and [CanAirIO](https://canair.io) community for testing and reports.

---
