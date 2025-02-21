#include <Arduino.h>
#include "sd_module.h"
#include "gps_module.h"
#include "bmp280_module.h"

void setup() {
  Serial.begin(115200);
  // ...existing code...

  // Monta la tarjeta SD al inicio
  if (!initSDModule()) {
    Serial.println("❌ Error al montar la SD, verifique conexión.");
    while(true) delay(100);
  }

  // Inicializa y calibra el GPS
  gpsInit();

  // Inicializa sensores BMP280 y AHT20
  initBmpAht();

  Serial.println("✅ Sistema inicializado correctamente.");
}

void loop() {
  // Procesar datos del GPS
  gpsProcess();
  
  // Leer y mostrar datos de BMP280 y AHT20
  readBmpAhtData();

  // ...existing periodic tasks...
  delay(1000);
}
