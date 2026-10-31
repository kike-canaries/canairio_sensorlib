#include "SensorDriver.h"

#include "../Sensors.hpp"

bool SensorDriver::enabled() const { return _sensors.isSensorRegistered(type()); }
