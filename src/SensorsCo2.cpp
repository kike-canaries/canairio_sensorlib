// CanAirIO sensorlib — generic CO2 compensation helpers.
#include <math.h>
#include <string.h>

#include "Sensors.hpp"

// Altitude compensation for CO2 sensors without Pressure atm or Altitude compensation
void Sensors::CO2correctionAlt() {
  DEBUG("-->[SLIB] CO2 altitude original\t:", String(CO2Val).c_str());
  float CO2cor = (0.016 * ((1013.25 - hpa) / 10) * (CO2Val - 400)) +
                 CO2Val;  // Increment of 1.6% for every hpa of difference at sea level
  CO2Val = round(CO2cor);
  DEBUG("-->[SLIB] CO2 compensated\t:", String(CO2Val).c_str());
}

/// hPa hectopascal calculation based on the altitude. See CO2AltitudeOffset setter
float Sensors::hpaCalculation(float altitude) {
  DEBUG("-->[SLIB] CO2 altitude offset\t:", String(altitude).c_str());
  float hpa =
      1012 - 0.118 * altitude +
      0.00000473 * altitude *
          altitude;  // Quadratic regression formula obtained PA (hpa) from high above the sea
  DEBUG("-->[SLIB] CO2 pressure (hPa)\t:", String(hpa).c_str());
  return hpa;
}
