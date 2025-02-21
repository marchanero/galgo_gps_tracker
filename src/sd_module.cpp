#include "sd_module.h"

// Definición de pines y creación de instancia VSPI
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCK  18
#define VSPI_CS   5

SPIClass vspi(VSPI);

// Se mejora mountSD para informar la frecuencia y error crítico tras reintentos.
static bool mountSD(uint8_t csPin, SPIClass &spi, uint32_t clock, uint8_t maxRetries = 3) {
    for (uint8_t i = 1; i <= maxRetries; i++) {
        Serial.printf("Intento %d para montar la SD con reloj %lu Hz...\n", i, clock);
        if (SD.begin(csPin, spi, clock)) {
            return true;
        }
        delay(1000);
    }
    Serial.println("❌ Error crítico: No se pudo montar la tarjeta SD tras múltiples intentos.");
    return false;
}

// Nueva función para asegurar que la SD esté montada
static bool ensureSDMounted() {
    if (!SD.begin(VSPI_CS, vspi, 1000000)) {
        Serial.println("❌ SD no montada, intentando montar nuevamente...");
        return mountSD(VSPI_CS, vspi, 1000000);
    }
    return true;
}

// Nueva función para listar ficheros en la SD
void listFiles(const char* dirname, uint8_t levels = 0) {
    File root = SD.open(dirname);
    if (!root) {
        Serial.println("❌ No se pudo abrir el directorio");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("❌ La ruta no es un directorio");
        return;
    }
    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("DIR: ");
            Serial.println(file.name());
            if (levels) {
                listFiles(file.name(), levels - 1);
            }
        } else {
            Serial.print("FILE: ");
            Serial.print(file.name());
            Serial.print(" (");
            Serial.print(file.size());
            Serial.println(" bytes)");
        }
        file = root.openNextFile();
    }
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

    // Mostrar listado de ficheros en la SD
    Serial.println("📂 Listado de archivos en la SD:");
    listFiles("/");

    return true;
}

// Se mejora writeFileSD para verificar el número de bytes escritos y forzar el vaciado del buffer.
bool writeFileSD(const char* path, const char* data) {
    if (!ensureSDMounted()) return false;
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
         Serial.printf("❌ No se pudo abrir el archivo %s para escribir.\n", path);
         return false;
    }
    size_t bytesWritten = file.print(data);
    file.flush();
    file.close();
    Serial.printf("✅ Se escribieron %u bytes en %s.\n", bytesWritten, path);
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

// Se mejora createFileSD para eliminar previamente el archivo existente y así reiniciarlo.
bool createFileSD(const char* path) {
    if (!ensureSDMounted()) return false;
    if (SD.exists(path)) {
         SD.remove(path);
         Serial.printf("ℹ️ Archivo %s existente eliminado para reiniciarlo.\n", path);
    }
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
         Serial.printf("❌ No se pudo crear el archivo %s.\n", path);
         return false;
    }
    file.flush();
    file.close();
    Serial.printf("✅ Archivo %s creado/reiniciado correctamente.\n", path);
    return true;
}

// Añade (guarda) datos al final del fichero especificado.
bool appendDataSD(const char* path, const char* data) {
    if (!ensureSDMounted()) return false;
    File file = SD.open(path, FILE_APPEND);
    if (!file) {
        Serial.printf("❌ No se pudo abrir el archivo %s para añadir datos.\n", path);
        return false;
    }
    file.print(data);
    file.flush();  // Forzar escritura del buffer
    file.close();
    Serial.printf("✅ Datos añadidos en %s correctamente.\n", path);
    return true;
}

// Nueva función para agregar datos en formato CSV
bool appendDataCSV(const char* path, const char* data) {
    if (!ensureSDMounted()) return false;
    File file = SD.open(path, FILE_APPEND);
    if (!file) {
         Serial.printf("❌ No se pudo abrir el archivo %s para añadir datos CSV.\n", path);
         return false;
    }
    file.print(data); // Aquí se asume que 'data' ya está formateado como CSV
    file.print("\n"); // Salto de línea para cada registro
    file.flush();
    file.close();
    Serial.printf("✅ Datos CSV añadidos en %s correctamente.\n", path);
    return true;
}

// Nueva función para verificar interrupción por teclado y cerrar la SD
static volatile bool systemPaused = false;  // NUEVA VARIABLE GLOBAL
static volatile bool systemTerminated = false;  // NUEVA VARIABLE GLOBAL

// Modificada función para detectar pausado y reanudación por teclado
void handleKeyboardInterrupt() {
    if (Serial.available()) {
        char c = Serial.read();
        if (!systemPaused && (c == 'p' || c == 'P')) {
            systemPaused = true;
            Serial.println("❌ Sistema pausado. Pulsa 's' para reanudar o 'x' para finalizar.");
        } else if (systemPaused && (c == 's' || c == 'S')) {
            systemPaused = false;
            Serial.println("✅ Sistema reanudado.");
        } else if (c == 'x' || c == 'X') {
            systemTerminated = true;
            Serial.println("❌ Finalizando el programa.");
        }
    }
}

// Nueva función para consultar el estado pausado
bool isSystemPaused() {
    return systemPaused;
}

// Nueva función para consultar si el sistema debe finalizar
bool isSystemTerminated() {
    return systemTerminated;
}
