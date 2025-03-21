/**
 * @file main.cpp
 * @author Antonio Vanegas @hpsaturn
 * @date June 2018 - 2025
 * @brief CanAirIO Sensorslib Tester App
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
#include <U8g2lib.h>

#include <ESP32WifiCLI.hpp>
#include <Sensors.hpp>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define MAIN_HW_EN_PIN 3  // Only for setup with booster board with enable pin

U8G2 *u8g2;
int dw, dh;

int LED_PIN = 17;  // change it via CLI using this example :D
                   // for instance if you are using a LilyGO T7 v1.5 board,
                   // set LED like this:
                   //   setled 19
                   // and the reboot with the command reboot.
                   // Also the LED could be ON when the WiFi is ready.

const char logo[] =
".s5SSSs.                      .s5SSSs.                s.  .s5SSSs.   \n"   
"      SS. .s5SSSs.  .s    s.        SS. s.  .s5SSSs.  SS.       SS.  \n"
"sS    `:;       SS.       SS. sS    S%S SS.       SS. S%S sS    S%S  \n"   
"SS        sS    S%S sSs.  S%S SS    S%S S%S sS    S%S S%S SS    S%S  \n"   
"SS        SSSs. S%S SS `S.S%S SSSs. S%S S%S SS .sS;:' S%S SS    S%S  \n"   
"SS        SS    S%S SS  `sS%S SS    S%S S%S SS    ;,  S%S SS    S%S  \n"   
"SS        SS    `:; SS    `:; SS    `:; `:; SS    `:; `:; SS    `:;  \n"   
"SS    ;,. SS    ;,. SS    ;,. SS    ;,. ;,. SS    ;,. ;,. SS    ;,.  \n"   
"`:;;;;;:' :;    ;:' :;    ;:' :;    ;:' ;:' `:    ;:' ;:' `:;;;;;:'  \n"   
"                                                                     \n" 
".s5SSSSs.                                                            \n"   
"SSS    .s5SSSs.  .s5SSSs.  .s5SSSSs. .s5SSSs.  .s5SSSs.             \n" 
"S%S          SS.       SS.    SSS          SS.       SS.            \n" 
"S%S    sS    `:; sS    `:;    S%S    sS    `:; sS    S%S            \n" 
"S%S    SSSs.     `:;;;;.      S%S    SSSs.     SS .sS;:'            \n" 
"S%S    SS              ;;.    S%S    SS        SS    ;,             \n" 
"`:;    SS              `:;    `:;    SS        SS    `:;            \n" 
";,.    SS    ;,. .,;   ;,.    ;,.    SS    ;,. SS    ;,.            \n" 
";:'    `:;;;;;:' `:;;;;;:'    ;:'    `:;;;;;:' `:    ;:'            \n" 
"";
    
/*********************************************************************
 * Optional callback.
 ********************************************************************/
class mESP32WifiCLICallbacks : public ESP32WifiCLICallbacks {
  void onWifiStatus(bool isConnected) {
    if (isConnected) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }

  void onHelpShow() {}

  void onNewWifi(String ssid, String passw) {}
};

/*********************************************************************
 * User defined commands sectioh.
 ********************************************************************/
void gotToSuspend(int type, int seconds) {
  delay(8);  // waiting for writing msg on serial
  // esp_deep_sleep(1000000LL * DEEP_SLEEP_DURATION);
  esp_sleep_enable_timer_wakeup(1000000LL * seconds);
  if (type == 0)
    esp_deep_sleep_start();
  else
    esp_light_sleep_start();
}

void sleep(char *args, Stream *response) {
  Pair<String, String> operands = wcli.parseCommand(args);
  int seconds = operands.second().toInt();
  if (operands.first().equals("deep")) {
    Serial.println("\ndeep suspending..");
    gotToSuspend(0, seconds);
  } else if (operands.first().equals("light")) {
    Serial.println("\nlight suspending..");
    gotToSuspend(1, seconds);
  } else {
    Serial.println("sleep: invalid option");
  }
}

void printSensorsDetected() {
  u8g2->clearBuffer();
  u8g2->setFont(u8g2_font_6x10_tf);
  uint16_t sensors_count = sensors.getSensorsRegisteredCount();
  uint16_t units_count = sensors.getUnitsRegisteredCount();
  Serial.println("-->[MAIN] Sensors detected count\t: " + String(sensors_count));
  String output = "SC:" + String(sensors_count) + " UC:" + String(units_count);
  u8g2->drawStr(0, 0, output.c_str());
  Serial.print("-->[MAIN] Sensors devices names\t: ");
  u8g2->drawStr(0, 10, "Sensors:");
  int i = 0;
  while (sensors.getSensorsRegistered()[i++] != 0) {
    Serial.print(sensors.getSensorName((SENSORS)sensors.getSensorsRegistered()[i - 1]));
    u8g2->drawStr(0, 10 + i * 10, sensors.getSensorName((SENSORS)sensors.getSensorsRegistered()[i - 1]).c_str());
    Serial.print(",");
    u8g2->drawStr(0, 10 + i * 10, ",");
  }
  Serial.println();
  u8g2->sendBuffer();
}

void printSensorsValues() {
  Serial.println("-->[MAIN] Preview sensor values:");
  UNIT unit = sensors.getNextUnit();
  while (unit != UNIT::NUNIT) {
    String uName = sensors.getUnitName(unit);
    float uValue = sensors.getUnitValue(unit);
    String uSymb = sensors.getUnitSymbol(unit);
    Serial.printf("-->[MAIN] %6s:\t%4.1f\t%s\n", uName.c_str(), uValue, uSymb.c_str());
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_6x10_tf);
    u8g2->drawStr(0, 0, uName.c_str());
    u8g2->drawStr(0, 10, String(uValue).c_str());
    u8g2->drawStr(0, 20, uSymb.c_str());
    unit = sensors.getNextUnit();
    delay(100); // only for show the values on the OLED
    u8g2->sendBuffer();
  }
}

void onSensorDataOk() {
  Serial.println("======= E X A M P L E   T E S T =========");
  printSensorsDetected();
  printSensorsValues();
}

void onSensorDataError(const char *msg) {}

void powerEnableSensors() {
  // init all sensors (step-up to 5V with enable pin)
  Serial.println("-->[POWR] == enable sensors ==");
  pinMode(MAIN_HW_EN_PIN, OUTPUT);
  digitalWrite(MAIN_HW_EN_PIN, HIGH);  // step-up on
}

void sensorsLoop(char *args, Stream *response) {
  sensors.loop();
}

void sensorsInit(char* args, Stream* response) {
  // powerEnableSensors(); // Only for special setup hardware with enable
  delay(100);
  Serial.println("\n== Sensor test setup ==\n");
  Serial.println("-->[SETUP] Detecting sensors..");

  sensors.setSampleTime(10);                     // config sensors sample time interval
  sensors.setOnDataCallBack(&onSensorDataOk);    // all data read callback
  sensors.setDebugMode(false);                   // [optional] debug mode
  sensors.detectI2COnly(false);                  // not force to only i2c sensors
  sensors.setTemperatureUnit(TEMPUNIT::KELVIN);  // comment for Celsius or set Fahrenheit
  // sensors.init(SENSORS::Auto, 13, 12);          // Auto detection (Custom UART sensor pins
  sensors.init();  // Auto detection (UART and i2c sensors)
  printSensorsDetected();
}

void initOled() {
  Wire.begin(13, 14);
  u8g2 = new U8G2_SSD1306_64X48_ER_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
  // u8g2->setBusClock(4000000);
  u8g2->begin();
  u8g2->setFont(u8g2_font_6x10_tf);
  u8g2->setContrast(128);
  u8g2->setFontRefHeightExtendedText();
  u8g2->setDrawColor(1);
  u8g2->setFontPosTop();
  u8g2->setFontDirection(0);
  u8g2->setFontMode(0);
  dw = u8g2->getDisplayWidth();
  dh = u8g2->getDisplayHeight();
}

void info(char *args, Stream *response) { 
  wcli.status(response);
  sensors.readAllSensors();
}

void reboot(char *args, Stream *response) {
  wcli.shell->clear();
  ESP.restart();
}

void wcli_clear(char *args, Stream *response){
  wcli.shell->clear();
}

void showLoader(U8G2 *u8g2, const char *title, const char *version, const char *revision, const char *msg) {
  u8g2->clearBuffer();
  u8g2->setFont(u8g2_font_6x10_tf);
  u8g2->drawStr(0, 0, title);
  u8g2->drawStr(0, 10, version);
  u8g2->drawStr(0, 20, revision);
  u8g2->drawStr(0, 30, msg);
  u8g2->sendBuffer();
}

void setup() {
  Serial.begin(115200);
  Serial.flush();                                  // Only for showing the message on serial
  Serial.setDebugOutput(true);                     // Optional, debug output redirect to serial
  delay(1000);                                     // Only for this demo. (wait for serial monitor)
  wcli.setCallback(new mESP32WifiCLICallbacks());  // Optional, set the callback
  wcli.setSilentMode(true);                        // less debug output
  // wcli.clearSettings();                          // Clear all networks and settings

  // Enter your custom commands:
  wcli.add("sleep", &sleep, "\t\t<mode> <time> ESP32 sleep mode (deep/light)\r\n");
  wcli.add("info", &info, "\t\tsystem status info");
  wcli.add("init", &sensorsInit, "\t\tperfom sensor detection init");
  wcli.add("read", &sensorsLoop, "\t\tperfom sensor read");
  wcli.add("clear", &wcli_clear, "\t\tclear shell");
  wcli.add("reboot", &reboot, "\tperform a ESP32 reboot");

  wcli.shell->attachLogo(logo);
  wcli.shell->clear();
  wcli.begin();  // Alternatively, you can init with begin(115200,appname)

  initOled();
  String version = "v" + String(CSL_VERSION);
  String revision = "r" + String(CSL_REVISION);
  showLoader(u8g2, "CanAirIO", version.c_str(), revision.c_str(), "type 'help'");
}

void loop() {
  wcli.loop();
  delay(100);
}