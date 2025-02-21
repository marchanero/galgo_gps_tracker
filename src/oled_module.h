#ifndef OLED_MODULE_H
#define OLED_MODULE_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // Ancho, en píxeles
#define SCREEN_HEIGHT 32 // Alto, en píxeles
#define OLED_RESET    -1 // Pin de reset (no usado)
#define OLED_ADDRESS  0x3C // Dirección I2C típica

extern Adafruit_SSD1306 display;

void initOled();
void displayStatus(const String &msg);

#endif // OLED_MODULE_H
