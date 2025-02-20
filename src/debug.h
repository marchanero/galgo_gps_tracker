#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

void debugSPI();
void debugPins(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs);

#endif
