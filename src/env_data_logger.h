#ifndef ENV_DATA_LOGGER_H
#define ENV_DATA_LOGGER_H

#include <Arduino.h>

void logEnvData(const String &gpsTime, unsigned long elapsedSec, float bmpTemp, float bmpPressure, float ahtTemp, float ahtHumidity);

#endif // ENV_DATA_LOGGER_H
