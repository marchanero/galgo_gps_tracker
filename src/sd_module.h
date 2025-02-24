#ifndef SD_MODULE_H
#define SD_MODULE_H

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// Configuración de pines SPI para SD
const int SD_CS = 5;    // Pin CS para SD
const int SD_SCK = 18;  // Pin SCK
const int SD_MISO = 19; // Pin MISO
const int SD_MOSI = 23; // Pin MOSI

// Configuración del sistema de archivos
const size_t BUFFER_SIZE = 4096;        // 4KB buffer
const size_t MAX_FILE_SIZE = 10485760;  // 10MB por archivo
const size_t MIN_FREE_SPACE = 104857600; // 100MB mínimo libre
const uint8_t MAX_ROTATIONS = 5;        // Máximo número de rotaciones
const uint32_t FLUSH_INTERVAL = 5000;   // 5 segundos entre flushes
const uint8_t MAX_BUFFERS = 5;          // Máximo número de buffers

// Estructura para el buffer de escritura
struct WriteBuffer {
    char buffer[BUFFER_SIZE];
    char filePath[32];
    size_t bufferPos;
    uint32_t lastFlush;
    bool used;
};

// Funciones principales
bool initSDModule();
bool writeFileSD(const char* path, const char* data);
String readFileSD(const char* path);
bool createFileSD(const char* path);
bool appendDataSD(const char* path, const char* data);
bool appendDataCSV(const char* path, const char* data);

// Funciones de buffer
bool appendToBuffer(WriteBuffer* buffer, const char* data);
bool flushBuffer(WriteBuffer* buffer);
void initBuffer(WriteBuffer* buffer, const char* path);
WriteBuffer* findBuffer(const char* path);
WriteBuffer* getFreeBuffer();
void flushAllBuffers();

// Funciones de rotación y espacio
bool checkSpace();
bool rotateFile(const char* path);
void cleanOldFiles(const char* directory, uint32_t maxAge);
size_t getFreeSpace();
bool shouldRotate(const char* path);
String getRotatedFileName(const char* path, int rotation);
void removeOldestRotation(const char* path);

// Funciones de control del sistema
void handleKeyboardInterrupt();
bool isSystemPaused();
bool isSystemTerminated();

#endif // SD_MODULE_H
