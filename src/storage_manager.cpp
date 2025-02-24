#include "storage_manager.h"

// Inicialización de variables estáticas
char StorageManager::compressionBuffer[COMPRESSION_BUFFER_SIZE];
char StorageManager::cacheBuffer[CACHE_BUFFER_SIZE];
size_t StorageManager::cacheSize = 0;
uint32_t StorageManager::lastFlushTime = 0;

bool StorageManager::initialize() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Error: No se pudo montar SPIFFS");
        return false;
    }
    initializeBuffers();
    return true;
}

void StorageManager::initializeBuffers() {
    memset(compressionBuffer, 0, COMPRESSION_BUFFER_SIZE);
    memset(cacheBuffer, 0, CACHE_BUFFER_SIZE);
    cacheSize = 0;
    lastFlushTime = millis();
}

bool StorageManager::writeData(const char* path, const char* data) {
    // Primero intentamos escribir en la caché
    if (!cacheData(path, data)) {
        // Si la caché está llena, hacer flush y reintentar
        if (!flushCache() || !cacheData(path, data)) {
            // Si aún falla, escribir directamente a SD
            return compressAndWrite(path, data);
        }
    }
    
    // Verificar si es momento de hacer flush
    if (shouldFlush()) {
        flushCache();
    }
    
    return true;
}

bool StorageManager::cacheData(const char* path, const char* data) {
    size_t dataLen = strlen(data);
    if (cacheSize + dataLen + 2 >= CACHE_BUFFER_SIZE) {
        return false;
    }
    
    // Formato: path\ndata\n
    strcat(cacheBuffer + cacheSize, path);
    strcat(cacheBuffer + cacheSize, "\n");
    strcat(cacheBuffer + cacheSize, data);
    strcat(cacheBuffer + cacheSize, "\n");
    
    cacheSize += strlen(path) + strlen(data) + 2;
    return true;
}

bool StorageManager::flushCache() {
    if (cacheSize == 0) return true;
    
    char* current = cacheBuffer;
    char* next;
    while ((next = strchr(current, '\n')) != NULL) {
        *next = '\0';
        const char* path = current;
        current = next + 1;
        
        next = strchr(current, '\n');
        if (!next) break;
        *next = '\0';
        const char* data = current;
        
        // Comprimir y escribir a SD
        if (!compressAndWrite(path, data)) {
            return false;
        }
        
        current = next + 1;
    }
    
    clearCache();
    lastFlushTime = millis();
    return true;
}

size_t StorageManager::compressData(const char* input, size_t inputSize, char* output, size_t outputSize) {
    return rleCompress(input, inputSize, output, outputSize);
}

size_t StorageManager::rleCompress(const char* input, size_t inputSize, char* output, size_t outputSize) {
    if (!input || !output || outputSize < 3) return 0;
    
    size_t outIndex = 0;
    size_t inIndex = 0;
    
    while (inIndex < inputSize && outIndex + 2 < outputSize) {
        char current = input[inIndex];
        uint8_t count = 1;
        
        while (inIndex + 1 < inputSize && 
               input[inIndex + 1] == current && 
               count < 255) {
            count++;
            inIndex++;
        }
        
        if (count > 3 || current == 0) {
            // Usar RLE si hay más de 3 repeticiones o es un carácter nulo
            output[outIndex++] = 0;  // Marcador RLE
            output[outIndex++] = count;
            output[outIndex++] = current;
        } else {
            // Copiar directamente
            while (count-- && outIndex < outputSize) {
                output[outIndex++] = current;
            }
        }
        
        inIndex++;
    }
    
    return outIndex;
}

bool StorageManager::compressAndWrite(const char* path, const char* data) {
    size_t dataLen = strlen(data);
    size_t compressedSize = compressData(data, dataLen, compressionBuffer, COMPRESSION_BUFFER_SIZE);
    
    if (compressedSize == 0) {
        return writeToSD(path, data, true);
    }
    
    // Verificar si necesitamos rotar el archivo
    if (checkRotation(path)) {
        rotateFile(path);
    }
    
    // Escribir datos comprimidos
    return writeToSD(path, compressionBuffer, true);
}

bool StorageManager::checkRotation(const char* path) {
    if (!SD.exists(path)) return false;
    
    File file = SD.open(path, FILE_READ);
    if (!file) return false;
    
    size_t size = file.size();
    file.close();
    
    return size >= MAX_FILE_SIZE;
}

bool StorageManager::rotateFile(const char* path) {
    // Crear backup antes de rotar
    createBackup(path);
    
    // Rotar archivos (mantener últimos 5)
    for (int i = 4; i >= 0; i--) {
        String oldPath = getRotatedPath(path, i);
        String newPath = getRotatedPath(path, i + 1);
        
        if (SD.exists(oldPath.c_str())) {
            if (i == 4) {
                SD.remove(oldPath.c_str());
            } else {
                SD.rename(oldPath.c_str(), newPath.c_str());
            }
        }
    }
    
    // Renombrar archivo actual
    return SD.rename(path, getRotatedPath(path, 0).c_str());
}

bool StorageManager::createBackup(const char* path) {
    if (!SD.exists(path)) return false;
    
    String backupPath = getBackupPath(path);
    File sourceFile = SD.open(path, FILE_READ);
    File backupFile = SD.open(backupPath.c_str(), FILE_WRITE);
    
    if (!sourceFile || !backupFile) {
        if (sourceFile) sourceFile.close();
        if (backupFile) backupFile.close();
        return false;
    }
    
    // Copiar contenido
    while (sourceFile.available()) {
        backupFile.write(sourceFile.read());
    }
    
    sourceFile.close();
    backupFile.close();
    return true;
}

String StorageManager::getBackupPath(const char* originalPath) {
    String path = String(originalPath);
    int dotIndex = path.lastIndexOf('.');
    if (dotIndex == -1) {
        return path + ".bak";
    }
    return path.substring(0, dotIndex) + ".bak" + path.substring(dotIndex);
}

String StorageManager::getRotatedPath(const char* originalPath, int index) {
    String path = String(originalPath);
    int dotIndex = path.lastIndexOf('.');
    if (dotIndex == -1) {
        return path + "." + String(index);
    }
    return path.substring(0, dotIndex) + "." + String(index) + path.substring(dotIndex);
}

bool StorageManager::writeToSD(const char* path, const char* data, bool append) {
    File file = SD.open(path, append ? FILE_APPEND : FILE_WRITE);
    if (!file) return false;
    
    size_t written = file.print(data);
    file.close();
    
    return written == strlen(data);
}

bool StorageManager::writeToSPIFFS(const char* path, const char* data, bool append) {
    File file = SPIFFS.open(path, append ? "a" : "w");
    if (!file) return false;
    
    size_t written = file.print(data);
    file.close();
    
    return written == strlen(data);
}

bool StorageManager::shouldFlush() {
    return cacheSize > (CACHE_BUFFER_SIZE * 0.75) || 
           (millis() - lastFlushTime) >= MAX_FLUSH_INTERVAL;
}

void StorageManager::clearCache() {
    memset(cacheBuffer, 0, CACHE_BUFFER_SIZE);
    cacheSize = 0;
}

bool StorageManager::flush() {
    return flushCache();
}