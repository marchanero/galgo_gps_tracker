#include <Arduino.h>

bool flagBipActivado = false; // Flag para activar/desactivar el patrón de bip

void bipPattern() {
  // Patrón: tres bips cortos con pausas intermedias y una pausa larga
  for (int i = 0; i < 3; i++) {
    analogWrite(25, 1);   // bip
    delay(100);
    analogWrite(25, 0);   // pausa
    delay(100);
  }
  delay(500); // pausa larga entre patrones
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(25, OUTPUT); // Configuración del pin 25 como salida
}

void loop() {
  if (flagBipActivado) {
    // Ejecuta el patrón de bip si está activado
    bipPattern();
  } else {
    // Otras operaciones o un retardo para no saturar el procesador
    delay(1000);
  }
}
