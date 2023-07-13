#include "geiger.h"

#ifdef ESP32
hw_timer_t* geiger_timer = NULL;
portMUX_TYPE* geiger_timerMux = NULL;
uint16_t tics_cnt = 0U; // tics in 1000ms
uint32_t tics_tot = 0U; // total tics since boot
MovingSum<uint16_t, uint32_t>* cajoe_fms;

// #########################################################################
// Interrupt routine called on each click from the geiger tube
// WARNING! the ISR is actually called on both the rising and the falling edge even if configure for FALLING or RISING

void IRAM_ATTR GeigerTicISR() {
  portENTER_CRITICAL_ISR(geiger_timerMux);
  tics_cnt++;  // tics in 1000ms
  tics_tot++;  // total tics since boot
  portEXIT_CRITICAL_ISR(geiger_timerMux);
}

// #########################################################################
// Interrupt timer routine called every 1000 ms
void IRAM_ATTR onGeigerTimer() {
  portENTER_CRITICAL_ISR(geiger_timerMux);
  cajoe_fms->add(tics_cnt);
  tics_cnt = 0;
  portEXIT_CRITICAL_ISR(geiger_timerMux);
}
#endif

// #########################################################################
// Initialize Geiger counter
bool GEIGER::init(int gpio, bool debug) {
#ifdef ESP32
  if (gpio < 0) {
    if (debug) Serial.println("[E][SLIB] undefined Geiger pin");
    return false;
  }
  devmode = debug;
  tics_cnt = 0U;  // tics in 1000ms
  tics_tot = 0U;  // total tics since boot

  geiger_timer = NULL;
  geiger_timerMux = new portMUX_TYPE(portMUX_INITIALIZER_UNLOCKED);

  // moving sum for CAJOE Geiger Counter, configured for 60 samples (1 sample every 1s * 60 samples = 60s)
  cajoe_fms = new MovingSum<uint16_t, uint32_t>(GEIGER_BUFSIZE);

  Serial.printf("-->[SLIB] Geiger startup on pin\t: %i\r\n",gpio);

  // attach interrupt routine to the GPI connected to the Geiger counter module
  pinMode(gpio, INPUT);
  attachInterrupt(digitalPinToInterrupt(gpio), GeigerTicISR, FALLING);

  // attach interrupt routine to internal timer, to fire every 1000 ms
  geiger_timer = timerBegin(GEIGER_TIMER, 80, true);
  timerAttachInterrupt(geiger_timer, &onGeigerTimer, true);
  timerAlarmWrite(geiger_timer, 1000000, true);  // 1000 ms
  timerAlarmEnable(geiger_timer);

  Serial.println("-->[SLIB] Geiger counter ready");

  return true;
#else
  return false; // TODO: We need an alternative to atmelsam
#endif
}

// #########################################################################
// Geiger counts evaluation
// CAJOE kit comes with a Chinese J305 geiger tube
// Conversion factor used for conversion from CPM to uSv/h is 0.008120370 (J305 tube)
bool GEIGER::read() {
#ifdef ESP32
  if (geiger_timer == NULL) return false;
  bool ready;
  uint32_t tics_len;

  portENTER_CRITICAL(geiger_timerMux);
  tics_cpm = cajoe_fms->getCurrentSum();
  tics_len = cajoe_fms->getCurrentFilterLength();
  portEXIT_CRITICAL(geiger_timerMux);

  // check whether the moving sum is full
  ready = (tics_len == cajoe_fms->getFilterLength());

  // convert CPM (tics in last minute) to uSv/h and put in display buffer for TFT
  // moving sum buffer size is 60 (1 sample every 1000 ms * 60 samples): the complete sum cover exactly last 60s
  if (ready) {
    uSvh = getUSvh();
  } else {
    uSvh = 0.0;
  }
  
  if (!devmode) return true;

  Serial.print("-->[SLIB] tTOT: ");
  Serial.println(tics_tot);
  Serial.print("-->[SLIB] tLEN: ");
  Serial.print(tics_len);
  Serial.println(ready ? " (ready)" : " (not ready)");
  Serial.print("-->[SLIB] tCPM: ");
  Serial.println(tics_cpm);
  Serial.print("-->[SLIB] uSvh: ");
  Serial.println(uSvh);

  return true;
#else
  return false;
#endif
}

/**
 * Converts CPM to uSv/h units (J305 tube)
*/
float GEIGER::getUSvh() {
  return float(tics_cpm) * J305_CONV_FACTOR;
}
/**
 * Returns CPM
*/
uint32_t GEIGER::getTics() {
  return tics_cpm;
}

void GEIGER::clear() {
  tics_cpm = 0;
  uSvh = 0.0;
#ifdef ESP32
  if (cajoe_fms != NULL) cajoe_fms->clear();
#endif
}
