#include "session_manager.h"
#include "storage_manager.h"
#include <time.h>
#include <stdio.h>
#include <string.h>

// Inicialización de variables estáticas
const char* SessionManager::BASE_PATH = "/sessions";
char SessionManager::currentSessionPath[128] = "";
bool SessionManager::isSessionActive = false;

// Definición de cabeceras CSV
const char* SessionManager::IMU_HEADER = "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,quat_w,quat_x,quat_y,quat_z";
const char* SessionManager::GPS_HEADER = "timestamp,latitude,longitude,altitude,speed,satellites,hdop";
const char* SessionManager::ENV_HEADER = "timestamp,temperature,pressure,humidity";

bool SessionManager::initialize() {
    if (!initSDModule()) return false;
    return StorageManager::initialize();
}

void SessionManager::generateSessionName(char* buffer, size_t size) {
    // Formato: YYYYMMDD_HHMMSS
    time_t now;
    time(&now);
    struct tm* timeinfo = localtime(&now);
    strftime(buffer, size, "%Y%m%d_%H%M%S", timeinfo);
}

bool SessionManager::createSessionDirectory() {
    // Crear directorio base
    if (!SD.mkdir(BASE_PATH)) {
        Serial.println("Error: No se pudo crear el directorio base");
        return false;
    }
    
    // Crear el directorio de la sesión
    char sessionName[32];
    generateSessionName(sessionName, sizeof(sessionName));
    snprintf(currentSessionPath, sizeof(currentSessionPath), "%s/%s", BASE_PATH, sessionName);
    
    if (!SD.mkdir(currentSessionPath)) {
        Serial.println("Error: No se pudo crear el directorio de sesión");
        return false;
    }
    Serial.printf("Sesión creada en: %s\n", currentSessionPath);
    return true;
}

void SessionManager::getIMULogPath(char* buffer, size_t size) {
    snprintf(buffer, size, "%s/imu_data.csv", currentSessionPath);
}

void SessionManager::getGPSLogPath(char* buffer, size_t size) {
    snprintf(buffer, size, "%s/gps_data.csv", currentSessionPath);
}

void SessionManager::getEnvLogPath(char* buffer, size_t size) {
    snprintf(buffer, size, "%s/env_data.csv", currentSessionPath);
}

bool SessionManager::writeHeader(const char* path, const char* header) {
    char headerWithNewline[256];
    snprintf(headerWithNewline, sizeof(headerWithNewline), "%s\n", header);
    return writeFileSD(path, headerWithNewline);
}

bool SessionManager::createLogFiles() {
    char path[128];
    
    getIMULogPath(path, sizeof(path));
    if (!writeHeader(path, IMU_HEADER)) return false;
    
    getGPSLogPath(path, sizeof(path));
    if (!writeHeader(path, GPS_HEADER)) return false;
    
    getEnvLogPath(path, sizeof(path));
    if (!writeHeader(path, ENV_HEADER)) return false;
    
    return true;
}

bool SessionManager::createNewSession() {
    if (isSessionActive) {
        endSession();
    }
    
    if (!createSessionDirectory() || !createLogFiles()) {
        return false;
    }
    
    isSessionActive = true;
    return true;
}

bool SessionManager::endSession() {
    if (!isSessionActive) {
        return true;
    }
    
    // Asegurar que todos los datos en caché se escriban a la SD
    if (!StorageManager::flush()) {
        Serial.println("Error: No se pudo hacer flush de los datos");
        return false;
    }
    
    isSessionActive = false;
    return true;
}

bool SessionManager::logIMUData(const char* data) {
    if (!isSessionActive) return false;
    char path[128];
    getIMULogPath(path, sizeof(path));
    return StorageManager::writeData(path, data);
}

bool SessionManager::logGPSData(const char* data) {
    if (!isSessionActive) return false;
    char path[128];
    getGPSLogPath(path, sizeof(path));
    return StorageManager::writeData(path, data);
}

bool SessionManager::logEnvironmentalData(const char* data) {
    if (!isSessionActive) return false;
    char path[128];
    getEnvLogPath(path, sizeof(path));
    return StorageManager::writeData(path, data);
}