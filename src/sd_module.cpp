#include "sd_module.h"
#include <string.h>

// Variables globales
static WriteBuffer buffers[MAX_BUFFERS];
static bool sdInitialized = false;
static volatile bool systemPaused = false;
static volatile bool systemTerminated = false;

// Implementación de funciones de buffer
bool appendToBuffer(WriteBuffer* buffer, const char* data) {
    size_t dataLen = strlen(data);
    
    // Verificar si hay espacio en el buffer
    if (buffer->bufferPos + dataLen >= BUFFER_SIZE) {
        if (!flushBuffer(buffer)) {
            return false;
        }
    }
    
    // Copiar datos al buffer
    strncpy(buffer->buffer + buffer->bufferPos, data, BUFFER_SIZE - buffer->bufferPos - 1);
    buffer->bufferPos += dataLen;
    buffer->buffer[buffer->bufferPos] = '\0';
    
    // Verificar si es momento de hacer flush por tiempo
    uint32_t now = millis();
    if (now - buffer->lastFlush >= FLUSH_INTERVAL) {
        return flushBuffer(buffer);
    }
    
    return true;
}

bool flushBuffer(WriteBuffer* buffer) {
    if (buffer->bufferPos == 0) return true;
    
    File file = SD.open(buffer->filePath, FILE_APPEND);
    if (!file) {
        Serial.printf("Error: No se pudo abrir %s para flush\n", buffer->filePath);
        return false;
    }
    
    size_t written = file.write((uint8_t*)buffer->buffer, buffer->bufferPos);
    file.close();
    
    if (written != buffer->bufferPos) {
        Serial.println("Error: Flush incompleto");
        return false;
    }
    
    buffer->bufferPos = 0;
    buffer->lastFlush = millis();
    return true;
}

void initBuffer(WriteBuffer* buffer, const char* path) {
    strncpy(buffer->filePath, path, sizeof(buffer->filePath) - 1);
    buffer->filePath[sizeof(buffer->filePath) - 1] = '\0';
    buffer->bufferPos = 0;
    buffer->lastFlush = millis();
    buffer->used = true;
}

WriteBuffer* findBuffer(const char* path) {
    for (int i = 0; i < MAX_BUFFERS; i++) {
        if (buffers[i].used && strcmp(buffers[i].filePath, path) == 0) {
            return &buffers[i];
        }
    }
    return NULL;
}

WriteBuffer* getFreeBuffer() {
    for (int i = 0; i < MAX_BUFFERS; i++) {
        if (!buffers[i].used) {
            return &buffers[i];
        }
    }
    return NULL;
}

void flushAllBuffers() {
    for (int i = 0; i < MAX_BUFFERS; i++) {
        if (buffers[i].used) {
            flushBuffer(&buffers[i]);
        }
    }
}

// Implementación de funciones principales
bool initSDModule() {
    // Configurar SPI para SD
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    
    // Intentar inicializar la SD varias veces
    for (int i = 0; i < 3; i++) {
        if (SD.begin(SD_CS)) {
            uint8_t cardType = SD.cardType();
            if (cardType == CARD_NONE) {
                Serial.println("No hay tarjeta SD insertada");
                return false;
            }
            
            // Mostrar información de la SD
            Serial.print("Tipo de tarjeta SD: ");
            switch (cardType) {
                case CARD_MMC:  Serial.println("MMC"); break;
                case CARD_SD:   Serial.println("SDSC"); break;
                case CARD_SDHC: Serial.println("SDHC/SDXC"); break;
                default:        Serial.println("DESCONOCIDO"); break;
            }
            
            uint64_t cardSize = SD.cardSize() / (1024 * 1024);
            Serial.printf("Tamaño de la SD: %lluMB\n", cardSize);
            
            // Verificar espacio disponible
            if (!checkSpace()) {
                Serial.println("Error: Espacio insuficiente en la SD");
                return false;
            }
            
            // Inicializar buffers
            memset(buffers, 0, sizeof(buffers));
            sdInitialized = true;
            Serial.println("SD inicializada correctamente");
            return true;
        }
        Serial.println("Fallo al montar SD, reintentando...");
        delay(1000);
    }
    
    Serial.println("Error: No se pudo inicializar la SD después de 3 intentos");
    return false;
}

bool writeFileSD(const char* path, const char* data) {
    if (!sdInitialized) return false;
    
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.printf("Error: No se pudo abrir %s para escribir\n", path);
        return false;
    }
    
    size_t written = file.print(data);
    file.close();
    
    return written == strlen(data);
}

String readFileSD(const char* path) {
    if (!sdInitialized) return "";
    
    File file = SD.open(path);
    if (!file) {
        Serial.printf("Error: No se pudo abrir %s para leer\n", path);
        return "";
    }
    
    String content;
    while (file.available()) {
        content += (char)file.read();
    }
    file.close();
    
    return content;
}

bool createFileSD(const char* path) {
    if (!sdInitialized) return false;
    
    if (SD.exists(path)) {
        SD.remove(path);
    }
    
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.printf("Error: No se pudo crear %s\n", path);
        return false;
    }
    
    file.close();
    return true;
}

bool appendDataSD(const char* path, const char* data) {
    if (!sdInitialized) return false;
    
    WriteBuffer* buffer = findBuffer(path);
    if (!buffer) {
        buffer = getFreeBuffer();
        if (!buffer) {
            // Si no hay buffers libres, usar el primero
            buffer = &buffers[0];
            flushBuffer(buffer);
        }
        initBuffer(buffer, path);
    }
    
    return appendToBuffer(buffer, data);
}

bool appendDataCSV(const char* path, const char* data) {
    if (!sdInitialized) return false;
    
    // Verificar si necesitamos rotar el archivo
    if (shouldRotate(path)) {
        rotateFile(path);
    }
    
    // Añadir salto de línea para CSV
    char csvData[strlen(data) + 2];
    strcpy(csvData, data);
    strcat(csvData, "\n");
    
    return appendDataSD(path, csvData);
}

// Implementación de funciones de rotación y espacio
bool checkSpace() {
    size_t freeSpace = getFreeSpace();
    return freeSpace >= MIN_FREE_SPACE;
}

size_t getFreeSpace() {
    return SD.totalBytes() - SD.usedBytes();
}

bool shouldRotate(const char* path) {
    File file = SD.open(path);
    if (!file) return false;
    
    size_t size = file.size();
    file.close();
    
    return size >= MAX_FILE_SIZE;
}

String getRotatedFileName(const char* path, int rotation) {
    String basePath = String(path);
    int dotPos = basePath.lastIndexOf('.');
    if (dotPos < 0) return String(path) + "." + String(rotation);
    
    return basePath.substring(0, dotPos) + "." + String(rotation) + 
           basePath.substring(dotPos);
}

void removeOldestRotation(const char* path) {
    String oldestFile = getRotatedFileName(path, MAX_ROTATIONS);
    if (SD.exists(oldestFile.c_str())) {
        SD.remove(oldestFile.c_str());
    }
}

bool rotateFile(const char* path) {
    // Eliminar la rotación más antigua si existe
    removeOldestRotation(path);
    
    // Mover cada archivo una posición
    for (int i = MAX_ROTATIONS - 1; i >= 1; i--) {
        String oldName = getRotatedFileName(path, i);
        String newName = getRotatedFileName(path, i + 1);
        if (SD.exists(oldName.c_str())) {
            SD.rename(oldName.c_str(), newName.c_str());
        }
    }
    
    // Mover el archivo actual a .1
    String newName = getRotatedFileName(path, 1);
    return SD.rename(path, newName.c_str());
}

void cleanOldFiles(const char* directory, uint32_t maxAge) {
    File root = SD.open(directory);
    if (!root || !root.isDirectory()) return;
    
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            // Obtener la última modificación del archivo
            time_t lastWrite = file.getLastWrite();
            if (millis() - lastWrite > maxAge) {
                SD.remove(file.path());
            }
        }
        file = root.openNextFile();
    }
}

// Implementación de funciones de control del sistema
void handleKeyboardInterrupt() {
    if (Serial.available()) {
        char c = Serial.read();
        if (!systemPaused && (c == 'p' || c == 'P')) {
            systemPaused = true;
            Serial.println("Sistema pausado. Pulsa 's' para reanudar o 'x' para finalizar.");
            flushAllBuffers();
        } else if (systemPaused && (c == 's' || c == 'S')) {
            systemPaused = false;
            Serial.println("Sistema reanudado.");
        } else if (c == 'x' || c == 'X') {
            systemTerminated = true;
            Serial.println("Finalizando el programa.");
            flushAllBuffers();
        }
    }
}

bool isSystemPaused() {
    return systemPaused;
}

bool isSystemTerminated() {
    return systemTerminated;
}
