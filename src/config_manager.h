#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "sd_module.h"

// Estructura para la configuración del dispositivo
struct DeviceConfig {
    // Configuración del sistema
    uint16_t sample_rate_ms = 100;
    uint8_t history_size = 10;
    
    // Configuración de calibración
    uint8_t calibration_threshold = 2;
    uint8_t max_init_attempts = 3;
    uint32_t calibration_timeout_ms = 30000;
    
    // Configuración de filtros
    float complementary_alpha = 0.98f;
    float kalman_q = 0.001f;
    float kalman_r = 0.1f;
    
    // Configuración de sensores
    struct {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    } mag_offset;
};

// Estructura para la calibración del BNO055
struct SensorCalibration {
    uint8_t sys_cal;
    uint8_t gyro_cal;
    uint8_t accel_cal;
    uint8_t mag_cal;
    float mag_offset_x;
    float mag_offset_y;
    float mag_offset_z;
};

class ConfigManager {
public:
    static bool initialize();
    static bool loadConfig();
    static bool saveConfig();
    static bool loadCalibration();
    static bool saveCalibration(const SensorCalibration& cal);
    
    static DeviceConfig& getConfig() { return config; }
    static const char* getConfigPath() { return "/config.json"; }
    static const char* getCalibrationPath() { return "/calibration.json"; }

private:
    static DeviceConfig config;
    static bool initialized;
};

#endif // CONFIG_MANAGER_H