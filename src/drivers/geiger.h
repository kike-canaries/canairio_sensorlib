#include <Arduino.h>

#include "MovingSum.h"

/**************************************************************
 *                          GEIGER
 * ************************************************************/

#define GEIGER_TIMER 1     // timer0 is already used (at least on TTGO-TDisplay) somewhere ???
#define GEIGER_BUFSIZE 60  // moving sum buffer size (1 sample every 1s * 60 samples = 60s)
#define J305_CONV_FACTOR \
  0.008120370  // conversion factor used for conversion from CPM to uSv/h units (J305 tube)

class GEIGER {
 private:
  bool devmode = false;

 public:
  uint32_t tics_cpm = 0U;  // tics in last 60s
  float uSvh = 0.0f;
  /**
   * @brief Constructor
   * @param gpio attached pin
   * @param debug debug mode enable/disable
   */
  explicit GEIGER(int gpio = -1, bool debug = false);
  bool read();
  void clear();
  uint32_t getTics();
  float getUSvh();
};