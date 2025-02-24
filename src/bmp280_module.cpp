#include "bmp280_module.h"
#include "oled_module.h"

// Declaración de objetos para BMP280 y AHT20
Adafruit_BMP280 bmp;    // Objeto para BMP280
Adafruit_AHTX0 aht;     // Objeto para AHT20

// Definición de variables globales
float bmp_temperature = 0.0f;
float bmp_pressure = 0.0f;
float aht_temperature = 0.0f;
float aht_humidity = 0.0f;

void initBmpAht() {
    Serial.println("Inicializando sensores BMP280 y AHT20...");
    
    // Inicializar BMP280
    showInitProgress("Iniciando BMP280...", 80);
    if (!bmp.begin(0x77)) {
        Serial.println("No se encontró BMP280. Verifique el cableado.");
        showInitProgress("Error BMP280!", 80);
        delay(1000);
        while (true) delay(10);
    }
    showInitProgress("BMP280 OK!", 85);
    delay(300);
    
    // Inicializar AHT20
    showInitProgress("Iniciando AHT20...", 85);
    if (!aht.begin()) {
        Serial.println("No se encontró AHT20. Verifique el cableado.");
        showInitProgress("Error AHT20!", 85);
        delay(1000);
        while (true) delay(10);
    }
    showInitProgress("AHT20 OK!", 90);
    delay(300);
    
    Serial.println("Sensores BMP280 y AHT20 inicializados correctamente.");
    showInitProgress("Sensores OK!", 90);
    delay(500);
}

void readBmpAhtData() {
    // Leer datos del BMP280
    bmp_temperature = bmp.readTemperature();
    bmp_pressure = bmp.readPressure() / 100.0F; // Presión en hPa
    
    // Leer datos del AHT20
    sensors_event_t aht_h, aht_t;
    aht.getEvent(&aht_h, &aht_t);
    aht_temperature = aht_t.temperature;
    aht_humidity = aht_h.relative_humidity;
    
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
