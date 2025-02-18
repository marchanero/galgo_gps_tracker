#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

// ...se eliminaron estructuras y funciones del sensor BNO055...

Adafruit_BMP280 bmp;    // Objeto para BMP280
Adafruit_AHTX0 aht;     // Objeto para AHT20

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Iniciando sensores BMP280 y AHT20...");
    Wire.begin(26, 22);

    // Inicializar BMP280 (direccion 0x76)
    if (!bmp.begin(0x77)) {
        Serial.println("No se encontró BMP280. Verifique el cableado.");
        while (true);
    }
    
    // Inicializar AHT20
    if (!aht.begin()) {
        Serial.println("No se encontró AHT20. Verifique el cableado.");
        while (true);
    }
    
    Serial.println("Sensores inicializados correctamente.");
}

void loop() {
    // Lectura del BMP280
    float bmp_temperature = bmp.readTemperature();
    float bmp_pressure = bmp.readPressure() / 100.0F; // Presión en hPa

    // Lectura del AHT20: se requieren dos eventos
    sensors_event_t aht_h, aht_t;
    aht.getEvent(&aht_h, &aht_t);
    float aht_temperature = aht_t.temperature;
    float aht_humidity = aht_h.relative_humidity;
    
    // Imprimir resultados en consola
    Serial.println("========== Datos del Sensor ==========");
    Serial.print("BMP280 - Temp: ");
    Serial.print(bmp_temperature);
    Serial.print(" °C, Presión: ");
    Serial.print(bmp_pressure);
    Serial.println(" hPa");
    
    Serial.print("AHT20 - Temp: ");
    Serial.print(aht_temperature);
    Serial.print(" °C, Humedad: ");
    Serial.print(aht_humidity);
    Serial.println(" %");
    Serial.println("======================================");
    
    delay(2000);
}
