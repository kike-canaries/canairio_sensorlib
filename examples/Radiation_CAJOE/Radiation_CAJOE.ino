/**
 * @file main.cpp
 * @date June 2018 - 2021
 * @brief Radiation sensor example
 * @license GPL3
 *
 * @license GPL3
 * 
 * Full documentation:
 * https://github.com/kike-canaries/canairio_sensorlib#canairio-air-quality-sensors-library
 * 
 * Full implementation for WiFi and Bluetooth Air Quality fixed and mobile station:
 * https://github.com/kike-canaries/canairio_firmware#canairio-firmware
 * 
 * CanAirIO project:
 * https://canair.io
 */

#include <Arduino.h>

#include <Sensors.hpp>

void onSensorDataOk() {
    Serial.print(" CO2: " + sensors.getStringCO2());
    Serial.print(" CO2humi: " + String(sensors.getCO2humi()));
    Serial.print(" CO2temp: " + String(sensors.getCO2temp()));

    Serial.print(" H: " + String(sensors.getHumidity()));
    Serial.println(" T: " + String(sensors.getTemperature()));
}

void onSensorDataError(const char* msg) {
    Serial.println(msg);
}

/******************************************************************************
*  M A I N
******************************************************************************/

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n== Sensor test setup ==\n");

    Serial.println("-->[SETUP] Detecting sensors..");

    sensors.setSampleTime(5);                        // config sensors sample time interval
    sensors.setOnDataCallBack(&onSensorDataOk);      // all data read callback
    sensors.setOnErrorCallBack(&onSensorDataError);  // [optional] error callback
    sensors.setDebugMode(true);                      // [optional] debug mode

    sensors.init();                     // forced UAQ sensor. Empty for auto detection
    
    delay(500);
}

void loop() {
    sensors.loop();  // read sensor data and showed it
}
// *************** GEIGER ****************
// ***************************************


 
// variables shared between main code and interrupt code
hw_timer_t * timer = NULL;
volatile uint32_t updateTime = 0;       // time for next update
volatile uint16_t tic_cnt = 0;
 
// To calculate a running average over 10 sec, we keep tic counts in 250ms intervals and add all 40 tic_buf values
#define TICBUFSIZE 40                   // running average buffer size
volatile uint16_t tic_buf[TICBUFSIZE];  // buffer for 40 times 250ms tick values (to have a running average for 10 seconds)
volatile uint16_t tic_buf_cnt = 0;
 
// In order to display a history of the last 7 minutes, we keep the last 50 values of 10sec tics
#define SEC10BUFSIZE 50                 // history array for displaymode==true
volatile uint16_t sec10 = 0;            // every 10 seconds counter
volatile uint16_t sec10_buf[SEC10BUFSIZE];  // buffer to hold 10 sec history (40*10 = 400 seconds)
volatile bool sec10updated = false;     // set to true when sec10_buf is updated
 
 
// #########################################################################
// Interrupt routine called on each click from the geiger tube
//
void IRAM_ATTR TicISR() {
  tic_cnt++;
}
 
// #########################################################################
// Interrupt timer routine called every 250 ms
//
void IRAM_ATTR onTimer() {
  tic_buf[tic_buf_cnt++] = tic_cnt;
  tic_cnt = 0;
  if (tic_buf_cnt>=TICBUFSIZE) {
    uint16_t tot = 0;
    for (int i=0; i<TICBUFSIZE; i++) tot += tic_buf[i];
    sec10_buf[sec10++] = tot;
    tic_buf_cnt = 0;    
    if (sec10>=SEC10BUFSIZE) sec10 = 0;
    sec10updated = true;
  }
}
 
 // Convert tics to mR/hr
float tics2mrem(uint16_t tics) {
  return float(tics) * TICFACTOR;
}

 void geigerInit() {
  
  Serial.begin(115200); // For debug
  Serial.println("Geiger counter startup");
  updateTime = millis(); // Next update time
 // attach interrupt routine to TIC interface from the geiger counter module
  pinMode(PINTIC, INPUT);
  attachInterrupt(PINTIC, TicISR, FALLING);
 
  // attach interrupt routine to internal timer, to fire every 250 ms
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 250000, true); // 250 ms
  timerAlarmEnable(timer);
  Serial.println("Geiger counter ready");
}

void geigerLoop() {
  
 // curent mR/hr value to display - add all tics from walking average 10 seconds
  uint16_t v=0;
  for (int i=0; i<TICBUFSIZE; i++) {
     v+=tic_buf[i];
  }
     
   // convert tics to mR/hr and put in display buffer for TFT
  float mrem = tics2mrem(v); 
  char buf[12];
  dtostrf(mrem, 7, (mrem<1 ? 2: (mrem<10 ? 1 : 0)), buf); 

  Serial.print("mRem: "); Serial.println (mrem);
  Serial.print("tics: "); Serial.println(v);

}

/*// Convert tics to mR/hr
//
float tics2mrem(uint16_t tics) {
  return float(tics) * TICFACTOR;
}
*/ 

/* // convert millirem value to a log percentage on analog and bar graph
//
int mrem2perc(float mrem, int maxperc) {
  if (mrem<=0) {
    return 0;
  } */
 