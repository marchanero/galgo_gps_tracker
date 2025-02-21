#include "sd_module.h"

// Definición de pines y creación de instancia VSPI
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCK  18
#define VSPI_CS   5

SPIClass vspi(VSPI);

// Función auxiliar para montar la SD con reintentos
static bool mountSD(uint8_t csPin, SPIClass &spi, uint32_t clock, uint8_t maxRetries = 3) {
    for (uint8_t i = 1; i <= maxRetries; i++) {
        Serial.printf("Intento %d para montar la SD...\n", i);
        if (SD.begin(csPin, spi, clock)) {
            return true;
        }
        delay(1000);
    }
    return false;
}

bool initSDModule() {
    pinMode(VSPI_CS, OUTPUT);
    digitalWrite(VSPI_CS, HIGH);  // Asegura que CS esté en HIGH

    // Inicializa VSPI
    vspi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, VSPI_CS);

    // Intenta montar la SD a 1 MHz (ajusta según sea necesario)
    if (!mountSD(VSPI_CS, vspi, 1000000)) {
         Serial.println("⚠️ No se pudo montar la tarjeta SD");
         return false;
    }

    Serial.println("✅ Tarjeta SD montada correctamente");
    return true;
}

bool writeFileSD(const char* path, const char* data) {
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.printf("No se pudo abrir el archivo %s para escribir.\n", path);
        return false;
    }
    file.print(data);
    file.close();
    return true;
}

String readFileSD(const char* path) {
    File file = SD.open(path);
    if (!file) {
        Serial.printf("No se pudo abrir el archivo %s para leer.\n", path);
        return "";
    }
    String content;
    while (file.available()) {
        content += char(file.read());
    }
    file.close();
    return content;
}

// Crea o reinicia (vacía) un fichero en la SD.
bool createFileSD(const char* path) {
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.printf("No se pudo crear el archivo %s.\n", path);
        return false;
    }
    file.flush();  // Forzar escritura del buffer
    file.close();
    Serial.printf("Archivo %s creado/reiniciado correctamente.\n", path);
    return true;
}

// Añade (guarda) datos al final del fichero especificado.
bool appendDataSD(const char* path, const char* data) {
    File file = SD.open(path, FILE_APPEND);
    if (!file) {
        Serial.printf("No se pudo abrir el archivo %s para añadir datos.\n", path);
        return false;
    }
    file.print(data);
    file.flush();  // Forzar escritura del buffer
    file.close();
    Serial.printf("Datos añadidos en %s correctamente.\n", path);
    return true;
}
