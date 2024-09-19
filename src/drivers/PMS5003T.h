#ifndef _PMS5003T_H_
#define _PMS5003T_H_

#include "PMS.h"
#include "PMS5003TBase.h"
#include "Stream.h"
#include <HardwareSerial.h>

/**
 * @brief The class define how to handle PMS5003T sensor bas on @ref PMS class
 */
class PMS5003T: public PMS5003TBase  {
public:
  explicit PMS5003T(Stream &serial);
  bool begin(void);
  void end(void);
  void handle(void);
  bool isFailed(void);
  int getPm01Ae(void);
  int getPm25Ae(void);
  int getPm10Ae(void);
  int getPm03ParticleCount(void);
  int convertPm25ToUsAqi(int pm25);
  float getTemperature(void);
  float getRelativeHumidity(void);

private:
  bool _isBegin = false;
  bool _isSleep = false;
  Stream *_serial;
  PMSBase pms;
  bool isBegin(void);
};

#endif /** _PMS5003T_H_ */
