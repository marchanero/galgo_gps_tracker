#ifndef BMP280_MODULE_H
#define BMP280_MODULE_H

#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

void initBmpAht();
void readBmpAhtData();

#endif // BMP280_MODULE_H
