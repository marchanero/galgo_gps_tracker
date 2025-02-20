#include "debug.h"
#include "SPI.h"

extern SPIClass hspi;

void debugSPI() {
    Serial.println("Debug: Iniciando prueba de SPI...");
    uint8_t testData = 0x55;  // Dato de prueba
    uint8_t received = hspi.transfer(testData);
    Serial.print("SPI transfer enviado: 0x");
    Serial.print(testData, HEX);
    Serial.print(", recibido: 0x");
    Serial.println(received, HEX);
}

void debugPins(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs) {
    Serial.println("Debug: Estados de pines:");
    Serial.print("SCK: "); Serial.println(digitalRead(sck));
    Serial.print("MISO: "); Serial.println(digitalRead(miso));
    Serial.print("MOSI: "); Serial.println(digitalRead(mosi));
    Serial.print("CS: "); Serial.println(digitalRead(cs));
}
