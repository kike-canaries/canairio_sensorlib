/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2020
 * @brief Particle meter sensor tests
 * @license GPL3
 * 
 * Full documentation:
 * https://github.com/kike-canaries/canairio_sensorlib#canairio-air-quality-sensors-library
 * 
 * Full implementation for WiFi and Bluetooth Air Quality fixed and mobile station:
 * https://github.com/kike-canaries/canairio_firmware#canairio-firmware
 * 
 * CanAirIO project docs:
 * https://canair.io/docs
 */

#include <Arduino.h>
#include <Sensors.hpp>
#include "seeed_line_chart.h" 

TFT_eSPI tft;
TFT_eSprite spr = TFT_eSprite(&tft);  // Sprite

#define MAX_SIZE 30 // maximum size of data
doubles data;       // Initilising a doubles type to store data
int brightness;
int header_height;
int btn_trigger;

UNIT current_unit = UNIT::PM25; 


int guiShowGraphHeader(String title){
  auto header = text(0, 0)
                    .value(title.c_str())
                    .align(center)
                    .valign(vcenter)
                    .width(spr.width())
                    .thickness(2);
  header.height(header.font_height(&spr) * 2);
  header.draw(&spr);  // Header height is the twice the height of the font
  return header.height();
}

void guiShowSensor(UNIT unit) {

  String uName = sensors.getUnitName(unit);
  float uValue = sensors.getUnitValue(unit);
  String uSymb = sensors.getUnitSymbol(unit);

  String title = uName + " " +String(uValue) + " (" + uSymb + ")";

  if (data.size() > MAX_SIZE) data.pop();  // keep the old line chart front
  data.push(uValue);

  // Update graph
  spr.fillSprite(TFT_WHITE);
  header_height = guiShowGraphHeader(title);

  // Settings for the line graph
  auto content = line_chart(20, header_height);    //(x,y) where the line graph begins
  content
      .height(spr.height() - header_height * 1.5)  // actual height of the line chart
      .width(spr.width() - content.x() * 2)        // actual width of the line chart
      .based_on(0.0)                               // Starting point of y-axis, must be a float
      .show_circle(false)                          // drawing a cirle at each point, default is on.
      .value(data)                                 // passing through the data to line graph
      .max_size(MAX_SIZE)
      .color(TFT_RED)        // Setting the color for the line
      .backgroud(TFT_WHITE)  // Setting the color for the backgroud
      .draw(&spr);

  spr.pushSprite(0, 0);
}

void printSensorsDetected() {
  uint16_t sensors_count = sensors.getSensorsRegisteredCount();
  uint16_t units_count = sensors.getUnitsRegisteredCount();
  Serial.print("-->[MAIN] Sensors devices\t: ");
  int i = 0;
  while (sensors.getSensorsRegistered()[i++] != 0) {
    String sensor = sensors.getSensorName((SENSORS)sensors.getSensorsRegistered()[i - 1]);
    Serial.print(sensor);
    Serial.print(",");
  }
  Serial.println(); 
}

bool selectNextUnit() {
  current_unit = sensors.getNextUnit();
  if (current_unit == UNIT::NUNIT) sensors.getNextUnit();
  return current_unit != UNIT::NUNIT;
}

void buttonsLoop() {
  if (digitalRead(WIO_5S_LEFT) == LOW && btn_trigger++ > 6) {
    while(!selectNextUnit())sensors.loop();
    btn_trigger = 0;
  } else if (digitalRead(WIO_5S_RIGHT) == LOW && btn_trigger++ > 6) {
    while(!selectNextUnit())sensors.loop();
    btn_trigger = 0;
  }
}

void onSensorDataOk() {
    printSensorsDetected();
    guiShowSensor(current_unit);
    Serial.println("=========================================");
}

void onSensorDataError(const char * msg){ 
}

/******************************************************************************
*  M A I N
******************************************************************************/

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n== Sensor test setup ==\n");

  Serial.println("-->[SETUP] Detecting sensors..");

  sensors.setSampleTime(1);                        // config sensors sample time interval
  sensors.setOnDataCallBack(&onSensorDataOk);      // all data read callback
  sensors.setOnErrorCallBack(&onSensorDataError);  // [optional] error callback
  sensors.setDebugMode(true);                      // [optional] debug mode
  sensors.detectI2COnly(true);                     // disable force to only i2c sensors
  sensors.init();                                  // Auto detection to UART and i2c sensors

  pinMode(A0, INPUT);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_WHITE);
  tft.setRotation(3);
  spr.createSprite(TFT_HEIGHT, TFT_WIDTH);
  spr.setRotation(3);

  pinMode(WIO_5S_UP, INPUT_PULLUP);
  pinMode(WIO_5S_DOWN, INPUT_PULLUP);
  pinMode(WIO_5S_LEFT, INPUT_PULLUP);
  pinMode(WIO_5S_RIGHT, INPUT_PULLUP);
  pinMode(WIO_5S_PRESS, INPUT_PULLUP);

  spr.fillSprite(TFT_WHITE);
  guiShowGraphHeader("Waiting for sensors..");
  spr.pushSprite(0, 0);
  while (!selectNextUnit()) sensors.loop();
}

void loop() {
  sensors.loop();  // read sensor data and showed it 
  buttonsLoop();
  delay(20);
}