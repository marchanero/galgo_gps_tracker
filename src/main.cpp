#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCK  18
#define VSPI_CS   5

SPIClass vspi(VSPI);

// Función para montar la tarjeta SD con reintentos
bool mountSD(uint8_t csPin, SPIClass &spi, uint32_t clock, uint8_t maxRetries = 3) {
    for (uint8_t i = 1; i <= maxRetries; i++) {
        Serial.printf("Intento %d para montar la SD...\n", i);
        if (SD.begin(csPin, spi, clock)) {
            return true;
        }
        delay(1000);  // Espera 1 segundo entre intentos
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n🔧 Iniciando ESP32 y VSPI con SD Card...");

    pinMode(VSPI_CS, OUTPUT);
    digitalWrite(VSPI_CS, HIGH);  // Aseguramos que CS está en HIGH antes de inicializar
    
    // 🚀 DEBUG: Verificar niveles de pines
    Serial.println("Debug: Estados de pines antes de SPI.begin():");
    Serial.printf("SCK: %d, MISO: %d, MOSI: %d, CS: %d\n", digitalRead(VSPI_SCK), digitalRead(VSPI_MISO), digitalRead(VSPI_MOSI), digitalRead(VSPI_CS));

    delay(1000);  // Esperamos a que la SD se estabilice

    vspi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, VSPI_CS);  // Inicializamos VSPI

    Serial.println("Debug: Iniciando prueba de SPI...");
    digitalWrite(VSPI_CS, LOW);
    uint8_t response = vspi.transfer(0x55);  // Comprobar comunicación SPI
    digitalWrite(VSPI_CS, HIGH);
    Serial.printf("SPI transfer enviado: 0x55, recibido: 0x%X\n", response);
    
    // Utilizamos la función de reintentos para montar la SD a baja velocidad (500 kHz)
    if (!mountSD(VSPI_CS, vspi, 1000000)) {
        Serial.println("⚠️ No se pudo montar la tarjeta SD");
        return;
    }

    Serial.println("✅ Tarjeta SD montada correctamente");

    // 🚀 DEBUG: Listar archivos en la SD
    File root = SD.open("/");
    Serial.println("📁 Archivos en la SD:");
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        Serial.print(" - ");
        Serial.println(entry.name());
        entry.close();
    }
}

void loop() {}
