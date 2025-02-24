#ifndef STORAGE_MANAGER_H
#define STORAGE_MANAGER_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <SD.h>

// Tamaño del buffer de compresión
#define COMPRESSION_BUFFER_SIZE 1024
// Tamaño del buffer de caché
#define CACHE_BUFFER_SIZE 4096
// Tiempo máximo entre flushes (ms)
#define MAX_FLUSH_INTERVAL 5000
// Tamaño máximo de archivo antes de rotar (bytes)
#define MAX_FILE_SIZE 10485760 // 10MB

class StorageManager {
public:
    static bool initialize();
    static bool writeData(const char* path, const char* data);
    static bool flush();
    
    // Métodos para compresión
    static size_t compressData(const char* input, size_t inputSize, char* output, size_t outputSize);
    static size_t decompressData(const char* input, size_t inputSize, char* output, size_t outputSize);
    
    // Métodos para caché
    static bool cacheData(const char* path, const char* data);
    static bool flushCache();
    
    // Métodos para rotación de archivos
    static bool checkRotation(const char* path);
    static bool rotateFile(const char* path);
    
    // Métodos para backup
    static bool createBackup(const char* path);
    static bool restoreFromBackup(const char* path);

private:
    static char compressionBuffer[COMPRESSION_BUFFER_SIZE];
    static char cacheBuffer[CACHE_BUFFER_SIZE];
    static size_t cacheSize;
    static uint32_t lastFlushTime;
    
    // Métodos internos
    static bool writeToSD(const char* path, const char* data, bool append = true);
    static bool writeToSPIFFS(const char* path, const char* data, bool append = true);
    static bool compressAndWrite(const char* path, const char* data);
    static void initializeBuffers();
    
    // Compresión simple RLE
    static size_t rleCompress(const char* input, size_t inputSize, char* output, size_t outputSize);
    static size_t rleDecompress(const char* input, size_t inputSize, char* output, size_t outputSize);
    
    // Gestión de caché
    static bool isCacheFull();
    static bool shouldFlush();
    static void clearCache();
    
    // Utilidades
    static String getBackupPath(const char* originalPath);
    static String getRotatedPath(const char* originalPath, int index);
    static bool fileExists(const char* path, bool inSPIFFS = false);
    static size_t getFileSize(const char* path, bool inSPIFFS = false);
};

#endif // STORAGE_MANAGER_H