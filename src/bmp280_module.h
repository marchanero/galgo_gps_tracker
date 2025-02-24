#ifndef BMP280_MODULE_H
#define BMP280_MODULE_H

#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

// Variables externas para los datos ambientales
extern float bmp_temperature;
extern float bmp_pressure;
extern float aht_temperature;
extern float aht_humidity;

void initBmpAht();
void readBmpAhtData();

#endif // BMP280_MODULE_H
