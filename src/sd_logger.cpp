#include "sd_logger.h"
#include "spi_pins.h"  // Inclusión de la configuración SPI
#include <SPI.h>
#include <SD.h>

// Nota: El pin CS se asigna al pin 15 para HSPI
#define SD_CS_PIN 15 // Pin de selección para HSPI (actualizado a 15)

bool initSD() {
    // Inicializar SPI con los pines definidos
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS_PIN);
    
    delay(2000); // Delay aumentado para mayor estabilización de la tarjeta

    const int maxRetries = 5;  // Incrementamos el número de reintentos
    for (int i = 0; i < maxRetries; i++) {
        if (SD.begin(SD_CS_PIN)) {
            Serial.println("✅ Tarjeta SD inicializada");
            return true;
        }
        Serial.print("❌ Error al inicializar la tarjeta SD, intento ");
        Serial.print(i + 1);
        Serial.println("...");
        delay(500);
    }
    Serial.println("❌ No se pudo montar el sistema de archivos SD");
    return false;
}

void logData(const char* data) {
    File logFile = SD.open("gps_log.txt", FILE_WRITE);
    if (logFile) {
        logFile.println(data);
        logFile.close();
    } else {
        Serial.println("❌ No se pudo abrir el archivo de log");
    }
}

void sdDiagnostic() {
    uint8_t cardType = SD.cardType();
    Serial.print("Tipo de tarjeta SD: ");
    switch (cardType) {
        case CARD_NONE:
            Serial.println("Ninguna tarjeta detectada");
            return;
        case CARD_MMC:
            Serial.println("MMC");
            break;
        case CARD_SD:
            Serial.println("SDSC");
            break;
        case CARD_SDHC:
            Serial.println("SDHC");
            break;
        default:
            Serial.println("Desconocido");
            break;
    }
    
    // Listar archivos en la raíz
    File root = SD.open("/");
    if (!root) {
        Serial.println("❌ No se pudo abrir la raíz de archivos");
        return;
    }
    Serial.println("Archivos en la raíz:");
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        Serial.print("  ");
        Serial.println(entry.name());
        entry.close();
    }
    root.close();
}
