#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

void gpsInit();
void gpsProcess();

extern TinyGPSPlus gps;

#endif // GPS_MODULE_H
