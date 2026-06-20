/*!
 * @file  i2c_change_auto_detect.ino
 * @brief Auto-detect DFRobot multigas sensors and configure them:
 *        - Detect sensor type (NH3, CO, NO2, O3)
 *        - Change I2C address to group 7
 *        - Enable temperature compensation
 *        - Set to passive mode
 * @n I2C Address groups (group 7):
 * @n A0 A1 => 0x78, 0x79, 0x7A, 0x7B
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @version     V2.0
 */
#include <Arduino.h>

#include "DFRobot_MultiGasSensor.h"

#define MAX_SENSORS 4
#define GROUP_7_BASE_ADDR 0x78

typedef struct {
  uint8_t addr;
  DFRobot_GAS_I2C* sensor;
  String gasType;
  bool initialized;
} SensorInfo;

SensorInfo sensors[MAX_SENSORS];
int sensorCount = 0;

bool initSensorAtAddress(uint8_t address) {
  DFRobot_GAS_I2C temp(&Wire, address);

  if (!temp.begin()) {
    return false;
  }

  String gasType = temp.queryGasType();

  if (gasType == "" || gasType == "None") {
    return false;
  }

  DFRobot_GAS_I2C* newSensor = new DFRobot_GAS_I2C(&Wire, address);
  newSensor->begin();

  sensors[sensorCount].addr = address;
  sensors[sensorCount].sensor = newSensor;
  sensors[sensorCount].gasType = gasType;
  sensors[sensorCount].initialized = false;

  Serial.print("Detected sensor at 0x");
  Serial.print(address, HEX);
  Serial.print(": ");
  Serial.println(gasType);

  sensorCount++;
  return true;
}

void configureSensor(SensorInfo& info) {
  if (!info.sensor || info.initialized) return;

  int attempts = 0;
  const int MAX_ATTEMPTS = 5;

  Serial.print("\nConfiguring ");
  Serial.print(info.gasType);
  Serial.print(" sensor at 0x");
  Serial.println(info.addr, HEX);

  while (!info.sensor->changeI2cAddrGroup(7) && attempts < MAX_ATTEMPTS) {
    Serial.println("  - I2C address change attempt failed, retrying...");
    delay(500);
    attempts++;
  }

  if (attempts >= MAX_ATTEMPTS) {
    Serial.println("  - ERROR: Failed to change I2C address!");
    return;
  }

  Serial.println("  - I2C address group changed to 7");
  delay(1000);

  // Enable temperature compensation
  info.sensor->setTempCompensation(info.sensor->ON);
  Serial.println("  - Temperature compensation enabled");
  delay(500);

  // Set to passive mode (request data on demand)
  info.sensor->changeAcquireMode(info.sensor->PASSIVITY);
  Serial.println("  - Passive mode enabled");
  delay(500);

  info.initialized = true;
  Serial.print("  - Configuration complete for ");
  Serial.print(info.gasType);
  Serial.println(" sensor");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("DFRobot MultiGas Sensor Auto-Configure");
  Serial.println("========================================");
  Serial.println("\nScanning for sensors...\n");

  // Scan default group 6 addresses (0x74-0x77) for sensors
  uint8_t defaultAddresses[] = {0x74, 0x75, 0x76, 0x77};

  for (uint8_t addr : defaultAddresses) {
    if (initSensorAtAddress(addr)) {
      if (sensorCount >= MAX_SENSORS) break;
    }
  }

  if (sensorCount == 0) {
    Serial.println("ERROR: No sensors detected!");
    Serial.println("Please check I2C connections and sensor power.");
    while (1) {
      delay(1000);
    }
  }

  Serial.print("\nFound ");
  Serial.print(sensorCount);
  Serial.println(" sensor(s)");

  // Configure all detected sensors
  Serial.println("\n========================================");
  Serial.println("Configuring sensors...");
  Serial.println("========================================");

  for (int i = 0; i < sensorCount; i++) {
    configureSensor(sensors[i]);
  }

  Serial.println("\n========================================");
  Serial.println("Configuration complete!");
  Serial.println("========================================\n");
}

void loop() {
  for (int i = 0; i < sensorCount; i++) {
    if (sensors[i].initialized) {
      Serial.print(sensors[i].gasType);
      Serial.print(" (0x");
      Serial.print(sensors[i].addr, HEX);
      Serial.print("): ");
      Serial.print(sensors[i].sensor->readGasConcentrationPPM());

      if (sensors[i].gasType == "O2") {
        Serial.println(" %vol");
      } else {
        Serial.println(" PPM");
      }
    }
  }

  Serial.println();
  delay(2000);
}
