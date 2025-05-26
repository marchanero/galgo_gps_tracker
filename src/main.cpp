#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "utility/imumaths.h"
#include "gps_module.h"
#include "bmp280_module.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include "sd_module.h"
#include "oled_module.h"
#include "config_manager.h"
#include "session_manager.h"
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>

const float GRAVITY_EARTH = 9.80665F; // Constante de gravedad terrestre
bool debug_bno055 = false; // Flag para modo debug. Si es true, se omite la calibración

#include "circular_buffer.h"


IMUHistory imuHistory;

uint8_t gatewayMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Envío broadcast

struct DerivedData {
    float total_acceleration;
    float angular_velocity;
    float tilt_angle;
    uint32_t last_update;
};
DerivedData derivedData = {0.0f, 0.0f, 0.0f, 0};

struct SensorState {
    bool is_calibrated;
    uint8_t system_status;
    uint8_t error_count;
    bool is_initialized;
};
SensorState sensorState = {false, 0, 0, false};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

struct LinearVelocity {
    float x;
    float y;
    float z;
};
LinearVelocity linearVelocity = {0.0f, 0.0f, 0.0f};

struct Position {
    float x;
    float y;
    float z;
};
Position position = {0.0f, 0.0f, 0.0f};

struct KalmanFilter {
    float Q; // Variancia del proceso
    float R; // Variancia de la medición
    float x; // Valor estimado
    float P; // Incertidumbre de la estimación
};

float kalmanUpdate(KalmanFilter *kf, float measurement) {
    kf->P += kf->Q;
    float K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + K * (measurement - kf->x);
    kf->P *= (1 - K);
    return kf->x;
}

static float kalman_velocity_x = 0.0f, kalman_velocity_y = 0.0f, kalman_velocity_z = 0.0f;
static float kalman_position_x = 0.0f, kalman_position_y = 0.0f, kalman_position_z = 0.0f;
static float P_vel = 1.0f, P_pos = 1.0f;

void printCentered(const char* text) {
    Serial.println(text);
}

void printSection(const char* title) {
    Serial.println(title);
}

float calculateMagnitude(const imu::Vector<3>& vec) {
    return sqrt(vec.x() * vec.x() + vec.y() * vec.y() + vec.z() * vec.z());
}

void updateHistory(float accel_mag, float gyro_mag) {
    imuHistory.update(accel_mag, gyro_mag);
}

bool checkCalibration() {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    DeviceConfig& config = ConfigManager::getConfig();
    return (sys >= config.calibration_threshold &&
            gyro >= config.calibration_threshold &&
            accel >= config.calibration_threshold &&
            mag >= config.calibration_threshold);
}

bool initializeSensor() {
    DeviceConfig& config = ConfigManager::getConfig();
    uint8_t attempts = 0;
    while (attempts < config.max_init_attempts) {
        if (bno.begin()) {
            bno.setExtCrystalUse(true);
            sensorState.is_initialized = true;
            
            // Cargar calibración previa si existe
            if (ConfigManager::loadCalibration()) {
                Serial.println("Calibración previa cargada");
            }
            
            return true;
        }
        Serial.println(F("Error al inicializar BNO055. Reintentando..."));
        delay(1000);
        attempts++;
    }
    return false;
}

void updateSystemState() {
    uint8_t system_status, self_test, system_error;
    bno.getSystemStatus(&system_status, &self_test, &system_error);
    sensorState.system_status = system_status;
    
    if (system_error) {
        sensorState.error_count++;
    }
    
    // Verificar calibración y guardarla si es válida
    bool was_calibrated = sensorState.is_calibrated;
    sensorState.is_calibrated = checkCalibration();
    
    if (!was_calibrated && sensorState.is_calibrated) {
        // Si acabamos de calibrar, guardar los datos
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        
        SensorCalibration cal;
        cal.sys_cal = sys;
        cal.gyro_cal = gyro;
        cal.accel_cal = accel;
        cal.mag_cal = mag;
        
        // Obtener offsets del magnetómetro
        imu::Vector<3> mag_data = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        cal.mag_offset_x = mag_data.x();
        cal.mag_offset_y = mag_data.y();
        cal.mag_offset_z = mag_data.z();
        
        ConfigManager::saveCalibration(cal);
        Serial.println("Nueva calibración guardada");
    }
}

void displayTemperature() {
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");
}

void calibrateSensor() {
    if (debug_bno055) {
        Serial.println("Modo debug activado: se omite la calibración.");
        return;
    }
    
    DeviceConfig& config = ConfigManager::getConfig();
    
    Serial.println("Calibrando sensor...");
    uint32_t calibrationStart = millis();
    while (!checkCalibration() && (millis() - calibrationStart < config.calibration_timeout_ms)) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
        Serial.print(".");
    }
    uint32_t elapsed = millis() - calibrationStart;
    uint32_t hours   = elapsed / 3600000;
    uint32_t minutes = (elapsed % 3600000) / 60000;
    uint32_t seconds = (elapsed % 60000) / 1000;
    
    if (!checkCalibration()) {
        Serial.println("\nERROR: Tiempo de calibración agotado.");
    } else {
        Serial.println("\nSensor calibrado.");
        updateSystemState(); // Esto guardará la calibración
    }
    Serial.print("Tiempo de calibración: ");
    Serial.print(hours);   Serial.print("h ");
    Serial.print(minutes); Serial.print("m ");
    Serial.print(seconds); Serial.println("s");
    
    digitalWrite(LED_BUILTIN, HIGH);
}

const char* getCompassDirection(float heading) {
    if (heading < 0) heading += 360;
    if (heading < 22.5 || heading >= 337.5) return "Norte";
    else if (heading < 67.5) return "Noreste";
    else if (heading < 112.5) return "Este";
    else if (heading < 157.5) return "Sureste";
    else if (heading < 202.5) return "Sur";
    else if (heading < 247.5) return "Suroeste";
    else if (heading < 292.5) return "Oeste";
    else return "Noroeste";
}

void applyComplementaryFilter(float dt, const imu::Vector<3>& accel, const imu::Vector<3>& gyro) {
    static float filteredTilt = 0.0f;
    float accelTilt = atan2(accel.z(), sqrt(accel.x()*accel.x() + accel.y()*accel.y())) * 180.0 / PI;
    float alpha = ConfigManager::getConfig().complementary_alpha;
    filteredTilt = alpha * (filteredTilt + gyro.x() * dt) + (1.0f - alpha) * accelTilt;
    Serial.print("📏 Inclinación Complementaria: ");
    Serial.print(filteredTilt, 4);
    Serial.println("°");
}

String getDeviceID() {
    String mac = WiFi.macAddress();  
    mac.replace(":", "");            
    // Tomamos los últimos 8 caracteres
    return mac.substring(mac.length() - 8);
}

String buildSensorPayload() {
    unsigned long timestamp = millis();
    String deviceID = getDeviceID();
    
    // Se asume que ya se han actualizado las variables globales y llamados a gpsProcess(), readBmpAhtData(), etc.
    String imuData = formatIMUData(timestamp, 
                                   bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL),
                                   bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE),
                                   bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER),
                                   bno.getQuat());
    
    String gpsData = "";
    gpsProcess();
    if (gps.location.isValid()) {
        gpsData = formatGPSData();
    }
    
    readBmpAhtData();
    String envData = formatEnvironmentalData((bmp_temperature + aht_temperature) / 2.0f,
                                               bmp_pressure,
                                               aht_humidity);
    
    String payload = "{";
    payload += "\"device_id\":\"" + deviceID + "\",";
    payload += "\"timestamp\":" + String(timestamp) + ",";
    payload += "\"imu\":{\"data\":\"" + imuData + "\"},";
    payload += "\"gps\":{\"data\":\"" + gpsData + "\"},";
    payload += "\"env\":{\"data\":\"" + envData + "\"}";
    payload += "}";
    return payload;
}

void sendDataToGateway(const String &data) {
    int result = esp_now_send(gatewayMAC, (uint8_t*)data.c_str(), data.length());
    Serial.print("Enviando datos: ");
    Serial.println(data);
    Serial.print("Resultado del envío: ");
    Serial.println(result == ESP_OK ? "OK" : "Error");
}
void setup() {
    Serial.begin(115200);
    Serial.println("\n\n");
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error al inicializar ESP-NOW");
        while (1) delay(10);
    }
    Serial.println("ESP-NOW iniciado");

    pinMode(LED_BUILTIN, OUTPUT);
    printCentered("Sistema de Orientación BNO055");
    Serial.println("\n");
    
    // Inicializar ConfigManager antes que nada
    if (!ConfigManager::initialize()) {
        Serial.println("ERROR CRÍTICO: No se pudo inicializar la configuración");
        while (1) delay(10);
    }
    Serial.println("Configuración cargada correctamente");
    
    // Inicializar SessionManager
    if (!SessionManager::initialize()) {
        Serial.println("ERROR CRÍTICO: No se pudo inicializar el gestor de sesiones");
        while (1) delay(10);
    }
    
    // Crear nueva sesión
    if (!SessionManager::createNewSession()) {
        Serial.println("ERROR CRÍTICO: No se pudo crear una nueva sesión");
        while (1) delay(10);
    }
    Serial.printf("Nueva sesión creada en: %s\n", SessionManager::getCurrentSessionPath());
    
    Wire.begin(26, 22);
    delay(100);
    
    if (!initializeSensor()) {
        Serial.println("ERROR CRÍTICO: No se pudo inicializar el sensor");
        while (1) delay(10);
    }
    printCentered("Sensor Inicializado Correctamente");
    
    calibrateSensor();
    initBmpAht();
    gpsInit();
    
    // Inicializar OLED y mostrar pantalla de inicio
    initOled();  // Ahora incluye la pantalla de inicio con logo
}

String formatIMUData(uint32_t timestamp, const imu::Vector<3>& accel, const imu::Vector<3>& gyro,
                    const imu::Vector<3>& mag, const imu::Quaternion& quat) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
             timestamp,
             accel.x(), accel.y(), accel.z(),
             gyro.x(), gyro.y(), gyro.z(),
             mag.x(), mag.y(), mag.z(),
             quat.w(), quat.x(), quat.y(), quat.z());
    return String(buffer);
}

String formatGPSData() {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%lu,%f,%f,%f,%f,%d,%f",
             millis(),
             gps.location.lat(), gps.location.lng(),
             gps.altitude.meters(),
             gps.speed.kmph(),
             gps.satellites.value(),
             gps.hdop.hdop());
    return String(buffer);
}

String formatEnvironmentalData(float temp, float pressure, float humidity) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%lu,%f,%f,%f",
             millis(),
             temp, pressure, humidity);
    return String(buffer);
}

void loop() {
    handleKeyboardInterrupt();
    if (isSystemTerminated()) {
        Serial.println("Programa finalizado.");
        SessionManager::endSession();
        while (true) {
            delay(100);
        }
    }
    while (isSystemPaused()) {  // Si se ha detenido, espera a reanudar
        handleKeyboardInterrupt();
        delay(100);
    }
    
    static uint32_t last_update = millis();
    uint32_t current_time = millis();
    uint32_t dt_ms = current_time - last_update;
    
    DeviceConfig& config = ConfigManager::getConfig();
    if (dt_ms >= config.sample_rate_ms) {
        float dt = dt_ms / 1000.0f;
        last_update = current_time;
        updateSystemState();
        
        // Inicializar filtros Kalman
        static bool kalmanInitialized = false;
        static KalmanFilter kfTotalAccel, kfAngularVel, kfTilt;
        if (!kalmanInitialized) {
            DeviceConfig& config = ConfigManager::getConfig();
            kfTotalAccel.x = derivedData.total_acceleration;
            kfTotalAccel.P = 1.0f; kfTotalAccel.Q = config.kalman_q; kfTotalAccel.R = config.kalman_r;
            kfAngularVel.x = derivedData.angular_velocity;
            kfAngularVel.P = 1.0f; kfAngularVel.Q = config.kalman_q; kfAngularVel.R = config.kalman_r;
            kfTilt.x = derivedData.tilt_angle;
            kfTilt.P = 1.0f; kfTilt.Q = config.kalman_q; kfTilt.R = config.kalman_r;
            kalmanInitialized = true;
        }
        
        sensors_event_t event;
        bno.getEvent(&event);
        if (sensorState.system_status == 0) {
            Serial.println(F("ERROR: Sensor no responde."));
            return;
        }
        imu::Vector<3> euler      = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> accel      = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> gyro       = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu::Vector<3> mag        = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        imu::Vector<3> accelFull  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> gravity    = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        imu::Quaternion quat      = bno.getQuat();
        derivedData.total_acceleration = calculateMagnitude(accel);
        derivedData.angular_velocity   = calculateMagnitude(gyro);
        derivedData.tilt_angle         = atan2(accel.z(), sqrt(accel.x() * accel.x() + accel.y() * accel.y())) * 180.0 / PI;
        updateHistory(derivedData.total_acceleration, derivedData.angular_velocity);
        linearVelocity.x += accel.x() * dt;
        linearVelocity.y += accel.y() * dt;
        linearVelocity.z += accel.z() * dt;
        position.x += linearVelocity.x * dt;
        position.y += linearVelocity.y * dt;
        position.z += linearVelocity.z * dt;
        applyComplementaryFilter(dt, accel, gyro);
        {
            static float dynamicTilt = 0.0f;
            static bool initDynamic = true;
            float accelTilt = derivedData.tilt_angle;
            if (initDynamic) {
                dynamicTilt = accelTilt;
                initDynamic = false;
            } else {
                float alpha = ConfigManager::getConfig().complementary_alpha;
                dynamicTilt = alpha * (dynamicTilt + gyro.x() * dt) + (1.0f - alpha) * accelTilt;
            }
            Serial.print("📏 Inclinación Dinámica: "); Serial.print(dynamicTilt, 4); Serial.println("°");
        }
        // Calcular y actualizar valores Kalman
        float kalmanTotalAccel = kalmanUpdate(&kfTotalAccel, derivedData.total_acceleration);
        float kalmanAngularVel = kalmanUpdate(&kfAngularVel, derivedData.angular_velocity);
        float kalmanTilt = kalmanUpdate(&kfTilt, derivedData.tilt_angle);

        // Imprimir datos Kalman
        Serial.print("🔎 Kalman Total Accel: "); Serial.print(kalmanTotalAccel, 4); Serial.println(" m/s²");
        Serial.print("🔎 Kalman Angular Vel: "); Serial.print(kalmanAngularVel, 4); Serial.println(" rad/s");
        Serial.print("🔎 Kalman Tilt: "); Serial.print(kalmanTilt, 4); Serial.println("°");
        {
            DeviceConfig& config = ConfigManager::getConfig();
            static KalmanFilter kfVelX = {config.kalman_q, config.kalman_r, kalman_velocity_x, P_vel};
            static KalmanFilter kfVelY = {config.kalman_q, config.kalman_r, kalman_velocity_y, P_vel};
            static KalmanFilter kfVelZ = {config.kalman_q, config.kalman_r, kalman_velocity_z, P_vel};
            static KalmanFilter kfPosX = {config.kalman_q, config.kalman_r, kalman_position_x, P_pos};
            static KalmanFilter kfPosY = {config.kalman_q, config.kalman_r, kalman_position_y, P_pos};
            static KalmanFilter kfPosZ = {config.kalman_q, config.kalman_r, kalman_position_z, P_pos};
            kalmanUpdate(&kfVelX, accel.x() * dt);
            kalman_velocity_x = kfVelX.x;
            kalmanUpdate(&kfVelY, accel.y() * dt);
            kalman_velocity_y = kfVelY.x;
            kalmanUpdate(&kfVelZ, accel.z() * dt);
            kalman_velocity_z = kfVelZ.x;
            kalmanUpdate(&kfPosX, kalman_velocity_x * dt);
            kalman_position_x = kfPosX.x;
            kalmanUpdate(&kfPosY, kalman_velocity_y * dt);
            kalman_position_y = kfPosY.x;
            kalmanUpdate(&kfPosZ, kalman_velocity_z * dt);
            kalman_position_z = kfPosZ.x;
            Serial.print("🔎 Kalman Vel X: "); Serial.print(kalman_velocity_x, 4); Serial.println(" m/s");
            Serial.print("🔎 Kalman Vel Y: "); Serial.print(kalman_velocity_y, 4); Serial.println(" m/s");
            Serial.print("🔎 Kalman Vel Z: "); Serial.print(kalman_velocity_z, 4); Serial.println(" m/s");
            Serial.print("🔎 Kalman Pos X: "); Serial.print(kalman_position_x, 4); Serial.println(" m");
            Serial.print("🔎 Kalman Pos Y: "); Serial.print(kalman_position_y, 4); Serial.println(" m");
            Serial.print("🔎 Kalman Pos Z: "); Serial.print(kalman_position_z, 4); Serial.println(" m");
        }
        Serial.println("============== SENSOR DATA ==============");
        Serial.print("⏱️  Time: ");            Serial.println(current_time);
        Serial.print("🔧 Calibration: ");      Serial.println(sensorState.is_calibrated ? "YES" : "NO");
        Serial.print("⚠️  Error Count: ");      Serial.println(sensorState.error_count);
        Serial.print("🌡️  Temperature: ");      Serial.println(bno.getTemp());
        displayTemperature();
        Serial.println("-----------------------------------------");
        {
            float heading = euler.x();
            const char* compassDir = getCompassDirection(heading);
            Serial.print("🧭 Dirección de la Brújula: ");
            Serial.println(compassDir);
        }
        Serial.print("📈 Linear Accel X: "); Serial.print(accel.x(), 4); Serial.print(" m/s²");
        Serial.print(", Y: ");              Serial.print(accel.y(), 4); Serial.print(" m/s²");
        Serial.print(", Z: ");              Serial.print(accel.z(), 4); Serial.println(" m/s²");
        Serial.print("🚀 Velocidad Lineal X: "); Serial.print(linearVelocity.x, 4); Serial.print(" m/s");
        Serial.print(" | Y: "); Serial.print(linearVelocity.y, 4); Serial.print(" m/s");
        Serial.print(" | Z: "); Serial.print(linearVelocity.z, 4); Serial.println(" m/s");
        Serial.print("📍 Posición Estimada X: "); Serial.print(position.x, 4); Serial.print(" m");
        Serial.print(" | Y: "); Serial.print(position.y, 4); Serial.print(" m");
        Serial.print(" | Z: "); Serial.print(position.z, 4); Serial.println(" m");
        Serial.print("🌀 Gyroscope X: "); Serial.print(gyro.x(), 4); Serial.print(" rps");
        Serial.print(", Y: ");            Serial.print(gyro.y(), 4); Serial.print(" rps");
        Serial.print(", Z: ");            Serial.print(gyro.z(), 4); Serial.println(" rps");
        {
            float headingRate = gyro.z();
            Serial.print("📏 Heading Rate: "); Serial.print(headingRate, 4); Serial.println(" deg/s");
        }
        {
            static float driftSumX = 0.0f, driftSumY = 0.0f, driftSumZ = 0.0f;
            static uint32_t driftCount = 0;
            if (fabs(gyro.x()) < 0.1f && fabs(gyro.y()) < 0.1f && fabs(gyro.z()) < 0.1f) {
                driftSumX += gyro.x();
                driftSumY += gyro.y();
                driftSumZ += gyro.z();
                driftCount++;
            }
            float avgDriftX = driftCount > 0 ? driftSumX / driftCount : 0.0f;
            float avgDriftY = driftCount > 0 ? driftSumY / driftCount : 0.0f;
            float avgDriftZ = driftCount > 0 ? driftSumZ / driftCount : 0.0f;
            Serial.print("🧭 Gyro Drift X: "); Serial.print(avgDriftX, 4); Serial.print(" rps");
            Serial.print(" | Y: "); Serial.print(avgDriftY, 4); Serial.print(" rps");
            Serial.print(" | Z: "); Serial.print(avgDriftZ, 4); Serial.println(" rps");
        }
        Serial.print("📡 Magnetometer X: "); Serial.print(mag.x(), 4); Serial.print(" uT");
        Serial.print(", Y: ");               Serial.print(mag.y(), 4); Serial.print(" uT");
        Serial.print(", Z: ");               Serial.print(mag.z(), 4); Serial.println(" uT");
        {
            DeviceConfig& config = ConfigManager::getConfig();
            float corrMagX = mag.x() - config.mag_offset.x;
            float corrMagY = mag.y() - config.mag_offset.y;
            float corrMagZ = mag.z() - config.mag_offset.z;
            float corrMagStrength = sqrt(corrMagX*corrMagX + corrMagY*corrMagY + corrMagZ*corrMagZ);
            Serial.print("📡 Magnetómetro compensado X: "); Serial.print(corrMagX, 4); Serial.print(" uT");
            Serial.print(", Y: "); Serial.print(corrMagY, 4); Serial.print(" uT");
            Serial.print(", Z: "); Serial.print(corrMagZ, 4); Serial.println(" uT");
            Serial.print("📡 Fuerza del Campo Magnético (Compensado): "); Serial.print(corrMagStrength, 4); Serial.println(" uT");
        }
        {
            float magFieldStrength = calculateMagnitude(mag);
            Serial.print("📡 Fuerza del Campo Magnético: ");
            Serial.print(magFieldStrength, 4);
            Serial.println(" uT");
        }
        Serial.print("📊 Accelerometer X: "); Serial.print(accelFull.x(), 4); Serial.print(" m/s²");
        Serial.print(", Y: ");                Serial.print(accelFull.y(), 4); Serial.print(" m/s²");
        Serial.print(", Z: ");                Serial.print(accelFull.z(), 4); Serial.println(" m/s²");
        Serial.print("🌐 Gravity X: "); Serial.print(gravity.x(), 4); Serial.print(" m/s²");
        Serial.print(", Y: ");           Serial.print(gravity.y(), 4); Serial.print(" m/s²");
        Serial.print(", Z: ");           Serial.print(gravity.z(), 4); Serial.println(" m/s²");
        Serial.print("📊 Total Accel: ");      Serial.print(derivedData.total_acceleration, 4);
        Serial.print(" m/s² (Avg: ");           Serial.print(imuHistory.getAccelAvg(), 4);
        Serial.println(" m/s²)");
        Serial.print("🌀 Angular Vel: ");      Serial.print(derivedData.angular_velocity, 4);
        Serial.print(" rad/s (Avg: ");          Serial.print(imuHistory.getGyroAvg(), 4);
        Serial.println(" rad/s)");
        Serial.print("📐 Tilt Angle: ");       Serial.print(derivedData.tilt_angle, 4); Serial.println("°");
        Serial.print("🔀 Quaternion: W: ");    Serial.print(quat.w(), 4);
        Serial.print(" X: ");                Serial.print(quat.x(), 4);
        Serial.print(" Y: ");                Serial.print(quat.y(), 4);
        Serial.print(" Z: ");                Serial.println(quat.z(), 4);
        Serial.println("=========================================\n");
        // Guardar datos del IMU
        String imuData = formatIMUData(current_time, accel, gyro, mag, quat);
        SessionManager::logIMUData(imuData);

        // Guardar datos GPS
        gpsProcess();
        if (gps.location.isValid()) {
            String gpsData = formatGPSData();
            SessionManager::logGPSData(gpsData);
        }

        // Guardar datos ambientales
        readBmpAhtData();
        String envData = formatEnvironmentalData(
            (bmp_temperature + aht_temperature) / 2.0f, // Promedio de ambos sensores
            bmp_pressure,
            aht_humidity
        );
        SessionManager::logEnvironmentalData(envData);

        // Actualizar datos en pantalla OLED
        DisplayData displayData;
        displayData.accel_total = kalmanTotalAccel;
        displayData.gyro_total = kalmanAngularVel;
        displayData.tilt_angle = kalmanTilt;
        
        displayData.latitude = gps.location.lat();
        displayData.longitude = gps.location.lng();
        displayData.speed = gps.speed.kmph();
        displayData.satellites = gps.satellites.value();
        
        displayData.temperature = (bmp_temperature + aht_temperature) / 2.0f;
        displayData.pressure = bmp_pressure;
        displayData.humidity = aht_humidity;
        
        displayData.sd_ok = true;  // Ya verificado en SessionManager
        displayData.imu_ok = sensorState.is_calibrated;
        displayData.gps_ok = gps.location.isValid();
        displayData.env_ok = true;  // Asumimos que está bien si llegamos aquí
        
        displayData.battery_level = 100;  // TODO: Implementar medición de batería
        displayData.signal_strength = gps.satellites.value() > 8 ? 4 :
                                    gps.satellites.value() > 6 ? 3 :
                                    gps.satellites.value() > 4 ? 2 : 1;
        
        updateDisplayData(displayData);
        
        // Cambiar página cada 3 segundos
        static uint32_t last_page_change = 0;
        if (current_time - last_page_change > 3000) {
            nextPage();
            last_page_change = current_time;
        }

        // Construir payload JSON y enviarlo vía ESP-NOW
        String payload = buildSensorPayload();
        sendDataToGateway(payload);
    }
}