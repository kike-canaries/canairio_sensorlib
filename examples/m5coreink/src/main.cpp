/***********************************************************************************
 * CanAirIO M5CoreInk
 * @author @hpsaturn
 * 
 * This project using CanAirIO Sensors Library. You can find the library here:
 * https://github.com/kike-canaries/canairio_sensorlib
 * 
 * == W A R N I N G ==
 *
 * The source code, documentation and the **last version** of this sample is here:
 * https://github.com/hpsaturn/co2_m5coreink
 * 
 * ==================
 * Tested with:
 * 
 * - One SCD30 (C02 sensor)
 * - One GCJA5 (Particulate Matter sensor)
 * - ENVII M5 Hat
 * 
 * But you can use it with any other i2c sensors, for example SPS30 or SCD41
 * UART sensors right nos is untested. 
 * 
 ***********************************************************************************/


#include <Arduino.h>
#include <M5CoreInk.h>
#include <Sensors.hpp>
#include <StreamString.h>
#include <rom/rtc.h>

#define DEEP_SLEEP_MODE       1     // eInk and esp32 hibernate
#define DEEP_SLEEP_TIME      20     // Please change it to 600s (10m) or more 
#define SAMPLES_COUNT         8     // samples before suspend (for PM2.5 ~9, 18sec, or more)
#define LOOP_DELAY            2     // seconds
#define BEEP_ENABLE           1     // eneble high level alarm
#define PM25_ALARM_BEEP      50     // PM2.5 level to trigger alarm
#define CO2_ALARM_BEEP     2000     // CO2 ppm level to trigger alarm
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

UNIT mainUnit = UNIT::NUNIT;
UNIT minorUnit = UNIT::NUNIT;
UNIT tempUnit = UNIT::NUNIT;
UNIT humiUnit = UNIT::NUNIT;
UNIT otherUnit = UNIT::NUNIT; 

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
    display.printf("%04i", (uint16_t)sensors.getUnitValue(mainUnit));

    display.setFont(&FreeMonoBold9pt7b);
    String mainUnitSymbol = sensors.getUnitName(mainUnit)+" / "+sensors.getUnitSymbol(mainUnit);
    uint16_t lenght = mainUnitSymbol.length();
    x = (display.width() / 2) - ((lenght*11)/2);
    y = display.height() / 2 - 8;
    display.setTextSize(0);
    display.setCursor(x, y);
    display.print(mainUnitSymbol);

    display.setFont(&FreeMonoBold12pt7b);
    display.setTextSize(1);

    x = 11;

    y = display.height() / 2 + 25;
    display.setCursor(x, y);
    String minorName = sensors.getUnitName(minorUnit);
    display.printf("%5s: %04d", minorName.c_str(), (uint16_t)sensors.getUnitValue(minorUnit));

    y = display.height() / 2 + 45;
    display.setCursor(x, y);
    display.printf(" Temp: %0.1fC",sensors.getUnitValue(tempUnit));

    y = display.height() / 2 + 65;
    display.setCursor(x, y);
    display.printf(" Humi: %0.1f%%",sensors.getUnitValue(humiUnit));

    y = display.height() / 2 + 85;
    display.setCursor(x, y);
    String oUnit = sensors.getUnitName(otherUnit);
    display.printf("%5s: %04.1f", oUnit.c_str(),sensors.getUnitValue(otherUnit));

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
void displayValuesPartialMode() {
    Serial.println("-->[eINK] displayValuesPartialMode");
    Serial.print("-->[eINK] drawing..");
    drawReady = false;
    display.setPartialWindow(0, 0, display.width(), display.height());
    display.setRotation(0);
    display.setFont(&FreeMonoBold24pt7b);
    display.setTextSize(1);
    display.setTextColor(GxEPD_BLACK);
    display.drawPaged(displayCO2ValuesCallback, 0);
}

void resetVariables() {  
    mainUnit = UNIT::NUNIT;
    minorUnit = UNIT::NUNIT;
    tempUnit = UNIT::NUNIT;
    humiUnit = UNIT::NUNIT;
    otherUnit = UNIT::NUNIT;
    sensors.resetUnitsRegister();
    sensors.resetSensorsRegister();
    sensors.resetAllVariables();
}

void getSensorsUnits() {
    Serial.println("-->[MAIN] Preview sensor values:");
    UNIT unit = sensors.getNextUnit();
    while(unit != UNIT::NUNIT) {
        if ((unit == UNIT::CO2 || unit == UNIT::PM25) && mainUnit == UNIT::NUNIT) {
            mainUnit = unit;
        } else if (unit == UNIT::CO2 && mainUnit == UNIT::PM25) {
            minorUnit = mainUnit;  // CO2 in indoors has more priority
            mainUnit = unit;       // and is shown in main unit field
        } else if (unit == UNIT::PM10 && minorUnit == UNIT::NUNIT) {
            minorUnit = unit;
        }
        if (unit == UNIT::TEMP || unit == UNIT::CO2TEMP) {
            tempUnit = unit;
            if (mainUnit == UNIT::NUNIT) mainUnit = unit;
        }
        if (unit == UNIT::HUM || unit == UNIT::CO2HUM) {
            humiUnit = unit;
        } 
        if (unit == UNIT::PRESS || mainUnit == UNIT::GAS || mainUnit == UNIT::ALT) {
            otherUnit = unit;
        }
        if (minorUnit == UNIT::NUNIT && unit == UNIT::ALT) minorUnit = unit;
        if (otherUnit == UNIT::NUNIT && unit == UNIT::CO2TEMP) otherUnit = unit;

        String uName = sensors.getUnitName(unit);
        float uValue = sensors.getUnitValue(unit);
        String uSymb = sensors.getUnitSymbol(unit);
        Serial.println("-->[MAIN] " + uName + " \t: " + String(uValue) + " " + uSymb);
        unit = sensors.getNextUnit();
    }
}

bool sensorsLoop() {
    if (sensors.readAllSensors()) {
        getSensorsUnits();
        return true;
    }
    return false;
}

void sensorsInit() {
    Serial.println("-->[SETUP] Detecting sensors..");
    Wire.begin(32, 33);           // I2C external port (bottom connector)
    Wire1.begin(25, 26);          // I2C via Hat (top connector)
    sensors.setSampleTime(1);     // config sensors sample time interval
    sensors.setDebugMode(false);  // [optional] debug mode
    sensors.detectI2COnly(true);  // force to only i2c sensors
    sensors.init();               // Auto detection to UART and i2c sensors
}

void beep() {
    M5.Speaker.tone(2000, 50);
    delay(30);
    M5.Speaker.mute();
}

void simulateDeepSleep() {
    Serial.println("-->[MAIN] Simulate deep sleep");
    sensors.init();
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
    int reset_reason = rtc_get_reset_reason(0);
    // Serial.println("-->[MAIN] Reset reason  \t: " + String(reset_reason));
    M5.update();
    // if (M5.BtnMID.isPressed() || reset_reason == 1) {
    if (M5.BtnMID.isPressed()) {
        displayHome();
        sensorsLoop();
        displayValuesPartialMode();
        beep();
    }

    delay(100);
    Serial.println("-->[SETUP] setup done");
}

void loop() {
    if (sensorsLoop()) {
        count++;
        if (count >= SAMPLES_COUNT) {
            displayValuesPartialMode();
            uint16_t alarmValue = 0;
            uint16_t mainValue = sensors.getUnitValue(mainUnit);
            if (mainUnit == UNIT::PM25) alarmValue = PM25_ALARM_BEEP;
            else alarmValue = CO2_ALARM_BEEP;
            if(BEEP_ENABLE == 1 && mainValue > alarmValue ) beep();
            count = 0;
        }
    }else{
        resetVariables();
        displayValuesPartialMode();
    }

    if (drawReady) {
        resetVariables(); // only for demostration connection and reconnection sensors
        if (DEEP_SLEEP_MODE == 1) {
            Serial.println("-->[LOOP] Deep sleep..");
            display.display(isCharging);
            display.powerOff();
            M5.shutdown(DEEP_SLEEP_TIME);
            Serial.println("-->[LOOP] USB is connected..");
            isCharging = true;              // it only is reached when the USB is connected
            simulateDeepSleep();
            Serial.println("-->[LOOP] Deep sleep done.");
        }
        drawReady = false;
    }

    delay(LOOP_DELAY * 1000);
}
