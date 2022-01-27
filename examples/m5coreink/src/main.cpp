#include <Arduino.h>
#include <M5CoreInk.h>
#include <Sensors.hpp>
#include <StreamString.h>

#define DEEP_SLEEP_MODE       1     // eInk and esp32 hibernate
#define DEEP_SLEEP_TIME      30     // seconds (600s = 10min)
#define SAMPLES_COUNT         5     // samples before suspend
#define LOOP_DELAY            2     // seconds
#define BEEP_ENABLE           1     // eneble high level alarm
#define ALARM_BEEP_VALUE   2500     // ppm level to trigger alarm
#define DISABLE_LED                 // improve battery

#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include "bitmaps/Bitmaps200x200.h"  // 1.54" b/w

GxEPD2_154_M09 medp = GxEPD2_154_M09(/*CS=D8*/ 9, /*DC=D3*/ 15, /*RST=D4*/ 0, /*BUSY=D2*/ 4);
GxEPD2_BW<GxEPD2_154_M09, GxEPD2_154_M09::HEIGHT> display(medp);  // GDEH0154D67

uint16_t co2value = 0;
uint16_t pm25value = 0;
float co2temp, co2humi, pressure;
uint16_t count;
bool drawReady;
bool isCharging;

void displayHomeCallback(const void*) {
    uint16_t x = 15;
    uint16_t y = display.height() / 2 - 30;
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print("0000");
}

void displayHome() {
    display.setRotation(0);
    display.setFont(&FreeMonoBold18pt7b);
    display.setTextSize(2);
    display.setTextColor(GxEPD_BLACK);
    display.setFullWindow();
    display.drawPaged(displayHomeCallback, 0);
}

void displayCO2ValuesCallback(const void*) {
    uint16_t x = 15;
    uint16_t y = display.height() / 2 - 30;
    display.fillScreen(GxEPD_WHITE);
    display.setTextSize(2);
    display.setCursor(x, y);
    display.setFont(&FreeMonoBold18pt7b);
    display.printf("%04i",co2value);

    display.setFont(&FreeMonoBold9pt7b);
    x = display.width() / 2 - 14;
    y = display.height() / 2 - 8;
    display.setTextSize(0);
    display.setCursor(x, y);
    display.print("PPM");

    display.setFont(&FreeMonoBold12pt7b);
    display.setTextSize(1);

    x = 11;

    y = display.height() / 2 + 25;
    display.setCursor(x, y);
    display.printf("Pm25: %04d",pm25value);

    y = display.height() / 2 + 45;
    display.setCursor(x, y);
    display.printf("Co2T: %.2fC",co2temp);

    y = display.height() / 2 + 65;
    display.setCursor(x, y);
    display.printf("Co2H: %.2f%%",co2humi);

    y = display.height() / 2 + 85;
    display.setCursor(x, y);
    display.printf("Pres: %04.1f", pressure);


    delay(100);

    drawReady = true;
    Serial.println("done");

}

/**
 * Display CO2 values in partialMode update.
 *  
 * it is a partial refresh mode can be used to full screen,
 * effective if display panel hasFastPartialUpdate
 */
void displayCO2ValuesPartialMode() {
    Serial.println("-->[eINK] displayCO2ValuesPartialMode");
    Serial.print("-->[eINK] drawing..");
    drawReady = false;
    display.setPartialWindow(0, 0, display.width(), display.height());
    display.setRotation(0);
    display.setFont(&FreeMonoBold24pt7b);
    display.setTextSize(1);
    display.setTextColor(GxEPD_BLACK);
    display.drawPaged(displayCO2ValuesCallback, 0);
}

bool sensorsLoop() {
    if (sensors.readAllSensors()) {
        co2value = sensors.getCO2();
        co2temp = sensors.getCO2temp();
        co2humi = sensors.getCO2humi();
        pm25value = sensors.getPM25();
        pressure = sensors.getPressure();
        return true;
    }
    return false;
}

void sensorsInit() {
    Serial.println("-->[SETUP] Detecting sensors..");
    Wire.begin(32, 33);           // I2C external port (bottom connector)
    Wire1.begin(25, 26);          // I2C via Hat (top connector)
    sensors.setSampleTime(1);     // config sensors sample time interval
    sensors.setDebugMode(true);   // [optional] debug mode
    sensors.detectI2COnly(true);  // disable force to only i2c sensors
    sensors.init();               // Auto detection to UART and i2c sensors
}

void beep() {
    M5.Speaker.tone(2000, 50);
    delay(30);
    M5.Speaker.mute();
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("-->[SETUP] setup init");

#ifdef DISABLE_LED    // turnoff it for improve battery life
    pinMode(LED_EXT_PIN, OUTPUT);
    digitalWrite(LED_EXT_PIN, HIGH);   
#endif
    
    M5.begin(false, false, true);
    sensorsInit();
    display.init(115200,false);

    M5.update();
    if (M5.BtnMID.isPressed()) {
        // sensorsConfig();
        displayHome();
        while(!sensorsLoop());
        displayCO2ValuesPartialMode();
    }

    delay(100);
    Serial.println("-->[SETUP] setup done");
}

void loop() {
    if (sensorsLoop()) {
        count++;
        if (count >= SAMPLES_COUNT) {
            displayCO2ValuesPartialMode();
            if(BEEP_ENABLE == 1 && co2value > ALARM_BEEP_VALUE ) beep();
            count = 0;
        }
    }

    if (drawReady) {
        if (DEEP_SLEEP_MODE == 1) {
            Serial.println("-->[LOOP] Deep sleep..");
            display.display(isCharging);
            display.powerOff();
            M5.shutdown(DEEP_SLEEP_TIME);
            Serial.println("-->[LOOP] USB is connected..");
            isCharging = true;              // it only is reached when the USB is connected
            Serial.println("-->[LOOP] Deep sleep done.");
        }
        drawReady = false;
    }

    delay(LOOP_DELAY * 1000);
}