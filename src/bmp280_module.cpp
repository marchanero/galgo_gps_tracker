#include "bmp280_module.h"

// Declaración de objetos para BMP280 y AHT20
Adafruit_BMP280 bmp;    // Objeto para BMP280
Adafruit_AHTX0 aht;     // Objeto para AHT20

void initBmpAht() {
    Serial.println("Inicializando sensores BMP280 y AHT20...");
    if (!bmp.begin(0x77)) {
        Serial.println("No se encontró BMP280. Verifique el cableado.");
        while (true) delay(10);
    }
    if (!aht.begin()) {
        Serial.println("No se encontró AHT20. Verifique el cableado.");
        while (true) delay(10);
    }
    Serial.println("Sensores BMP280 y AHT20 inicializados correctamente.");
}

void readBmpAhtData() {
    float bmp_temperature = bmp.readTemperature();
    float bmp_pressure = bmp.readPressure() / 100.0F; // Presión en hPa
    sensors_event_t aht_h, aht_t;
    aht.getEvent(&aht_h, &aht_t);
    float aht_temperature = aht_t.temperature;
    float aht_humidity = aht_h.relative_humidity;
    
    Serial.println("========== Datos BMP280 & AHT20 ==========");
    Serial.println();
    Serial.print("BMP280 -> Temp: ");
    Serial.print(bmp_temperature, 2);
    Serial.print(" °C | Pressure: ");
    Serial.print(bmp_pressure, 2);
    Serial.println(" hPa");
    
    Serial.print("AHT20  -> Temp: ");
    Serial.print(aht_temperature, 2);
    Serial.print(" °C | Humidity: ");
    Serial.print(aht_humidity, 2);
    Serial.println(" %");
    
    Serial.println();
    Serial.println("===========================================");
}
