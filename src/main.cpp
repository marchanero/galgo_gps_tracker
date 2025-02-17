#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 26  // Pin SDA alternativo 25 zumbador
#define SCL_PIN 22  // Pin SCL alternativo

TwoWire I2C = TwoWire(1); // Usar el segundo canal I2C

void setup() {
  Serial.begin(115200);
  while (!Serial); // Esperar a conexión del monitor serie si es necesario
  Serial.println("\nIniciando el escáner I2C...");
  I2C.begin(SDA_PIN, SCL_PIN); // Iniciar I2C con pines definidos
  delay(1000); // Esperar un segundo antes de comenzar el escaneo
}

void loop() {
  uint8_t count = 0;
  Serial.println("\nBuscando dispositivos I2C...");
  for (uint8_t address = 1; address < 127; address++) {
    I2C.beginTransmission(address);
    uint8_t error = I2C.endTransmission();
    if (error == 0) {
      Serial.print("Dispositivo encontrado en la dirección 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      count++;
    } else if (error == 4) {
      Serial.print("Error desconocido en la dirección 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (count == 0)
    Serial.println("No se encontraron dispositivos I2C.");
  else
    Serial.print("Total de dispositivos encontrados: "), Serial.println(count);

  delay(5000); // Espera 5 segundos antes de reescanear
}
