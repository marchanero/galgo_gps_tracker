#include "config_manager.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// Inicialización de variables estáticas
DeviceConfig ConfigManager::config;
bool ConfigManager::initialized = false;

bool ConfigManager::initialize() {
    if (!SD.begin()) {
        Serial.println("Error: No se pudo inicializar la SD para la configuración");
        return false;
    }
    
    // Intentar cargar la configuración existente
    if (!loadConfig()) {
        Serial.println("Aviso: No se pudo cargar la configuración, usando valores por defecto");
        // Los valores por defecto ya están en la estructura DeviceConfig
        saveConfig(); // Guardar la configuración por defecto
    }
    
    initialized = true;
    return true;
}

bool ConfigManager::loadConfig() {
    if (!SD.exists(getConfigPath())) {
        Serial.println("Aviso: Archivo de configuración no existe");
        return false;
    }

    File configFile = SD.open(getConfigPath(), FILE_READ);
    if (!configFile) {
        Serial.println("Error: No se pudo abrir el archivo de configuración");
        return false;
    }

    // Crear buffer para el documento JSON
    StaticJsonDocument<512> doc;
    
    // Deserializar el JSON
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();

    if (error) {
        Serial.print("Error al parsear config.json: ");
        Serial.println(error.c_str());
        return false;
    }

    // Cargar valores
    config.sample_rate_ms = doc["sample_rate_ms"] | 100;
    config.history_size = doc["history_size"] | 10;
    config.calibration_threshold = doc["calibration_threshold"] | 2;
    config.max_init_attempts = doc["max_init_attempts"] | 3;
    config.calibration_timeout_ms = doc["calibration_timeout_ms"] | 30000;
    config.complementary_alpha = doc["complementary_alpha"] | 0.98f;
    config.kalman_q = doc["kalman_q"] | 0.001f;
    config.kalman_r = doc["kalman_r"] | 0.1f;
    config.mag_offset.x = doc["mag_offset"]["x"] | 0.0f;
    config.mag_offset.y = doc["mag_offset"]["y"] | 0.0f;
    config.mag_offset.z = doc["mag_offset"]["z"] | 0.0f;

    return true;
}

bool ConfigManager::saveConfig() {
    File configFile = SD.open(getConfigPath(), FILE_WRITE);
    if (!configFile) {
        Serial.println("Error: No se pudo abrir el archivo para guardar la configuración");
        return false;
    }

    StaticJsonDocument<512> doc;

    // Guardar valores actuales
    doc["sample_rate_ms"] = config.sample_rate_ms;
    doc["history_size"] = config.history_size;
    doc["calibration_threshold"] = config.calibration_threshold;
    doc["max_init_attempts"] = config.max_init_attempts;
    doc["calibration_timeout_ms"] = config.calibration_timeout_ms;
    doc["complementary_alpha"] = config.complementary_alpha;
    doc["kalman_q"] = config.kalman_q;
    doc["kalman_r"] = config.kalman_r;
    doc["mag_offset"]["x"] = config.mag_offset.x;
    doc["mag_offset"]["y"] = config.mag_offset.y;
    doc["mag_offset"]["z"] = config.mag_offset.z;

    // Serializar a JSON
    if (serializeJson(doc, configFile) == 0) {
        Serial.println("Error al escribir config.json");
        configFile.close();
        return false;
    }

    configFile.close();
    return true;
}

bool ConfigManager::loadCalibration() {
    if (!SD.exists(getCalibrationPath())) {
        Serial.println("Aviso: Archivo de calibración no existe");
        return false;
    }

    File calFile = SD.open(getCalibrationPath(), FILE_READ);
    if (!calFile) {
        Serial.println("Error: No se pudo abrir el archivo de calibración");
        return false;
    }

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, calFile);
    calFile.close();

    if (error) {
        Serial.print("Error al parsear calibration.json: ");
        Serial.println(error.c_str());
        return false;
    }

    // Cargar en una estructura temporal
    SensorCalibration cal;
    cal.sys_cal = doc["sys_cal"] | 0;
    cal.gyro_cal = doc["gyro_cal"] | 0;
    cal.accel_cal = doc["accel_cal"] | 0;
    cal.mag_cal = doc["mag_cal"] | 0;
    cal.mag_offset_x = doc["mag_offset_x"] | 0.0f;
    cal.mag_offset_y = doc["mag_offset_y"] | 0.0f;
    cal.mag_offset_z = doc["mag_offset_z"] | 0.0f;

    // Actualizar la configuración con los offsets del magnetómetro
    config.mag_offset.x = cal.mag_offset_x;
    config.mag_offset.y = cal.mag_offset_y;
    config.mag_offset.z = cal.mag_offset_z;

    return true;
}

bool ConfigManager::saveCalibration(const SensorCalibration& cal) {
    File calFile = SD.open(getCalibrationPath(), FILE_WRITE);
    if (!calFile) {
        Serial.println("Error: No se pudo abrir el archivo para guardar la calibración");
        return false;
    }

    StaticJsonDocument<256> doc;
    
    doc["sys_cal"] = cal.sys_cal;
    doc["gyro_cal"] = cal.gyro_cal;
    doc["accel_cal"] = cal.accel_cal;
    doc["mag_cal"] = cal.mag_cal;
    doc["mag_offset_x"] = cal.mag_offset_x;
    doc["mag_offset_y"] = cal.mag_offset_y;
    doc["mag_offset_z"] = cal.mag_offset_z;

    if (serializeJson(doc, calFile) == 0) {
        Serial.println("Error al escribir calibration.json");
        calFile.close();
        return false;
    }

    calFile.close();

    // Actualizar también la configuración
    config.mag_offset.x = cal.mag_offset_x;
    config.mag_offset.y = cal.mag_offset_y;
    config.mag_offset.z = cal.mag_offset_z;

    return true;
}