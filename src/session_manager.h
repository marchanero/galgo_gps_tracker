#ifndef SESSION_MANAGER_H
#define SESSION_MANAGER_H

#include <Arduino.h>
#include "sd_module.h"

class SessionManager {
public:
    static bool initialize();
    static bool createNewSession();
    static bool endSession();
    
    // Funciones para logging de datos
    static bool logIMUData(const String& data) { return logIMUData(data.c_str()); }
    static bool logIMUData(const char* data);
    
    static bool logGPSData(const String& data) { return logGPSData(data.c_str()); }
    static bool logGPSData(const char* data);
    
    static bool logEnvironmentalData(const String& data) { return logEnvironmentalData(data.c_str()); }
    static bool logEnvironmentalData(const char* data);
    
    // Getters para rutas de archivos
    static String getCurrentSessionPath() { return String(currentSessionPath); }
    static void getIMULogPath(char* buffer, size_t size);
    static void getGPSLogPath(char* buffer, size_t size);
    static void getEnvLogPath(char* buffer, size_t size);
    
    // Cabeceras CSV
    static const char* IMU_HEADER;
    static const char* GPS_HEADER;
    static const char* ENV_HEADER;

private:
    static const char* BASE_PATH;  // "/sd/sessions"
    static char currentSessionPath[128];
    static bool isSessionActive;
    
    static bool createSessionDirectory();
    static bool createLogFiles();
    static void generateSessionName(char* buffer, size_t size);
    static bool writeHeader(const char* path, const char* header);
};

#endif // SESSION_MANAGER_H