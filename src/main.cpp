#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "utility/imumaths.h"
#include "gps_module.h"

// Constantes para la configuración del sistema// Constantes para la configuración del sistema
const uint16_t SAMPLE_RATE_MS = 100;  // Tasa de muestreo en milisegundose muestreo en milisegundos
const uint8_t HISTORY_SIZE = 10;      // Tamaño del histórico para promediosios
const float GRAVITY_EARTH = 9.80665F; // Constante de gravedad terrestre
const uint8_t CALIBRATION_THRESHOLD = 2; // Umbral mínimo de calibración
const uint8_t MAX_INIT_ATTEMPTS = 3;    // Máximo número de intentos de inicializacióninicialización

// Nuevas variables globales para control de calibración
const uint32_t CALIBRATION_TIMEOUT_MS = 30000; // 30 segundos de tiempo máximo de calibración
bool debug_bno055 = false; // Flag para modo debug. Si es true, se omite la calibración

const float COMPLEMENTARY_ALPHA = 0.98f;

// Declaración correcta de estructuras
struct SensorHistory {
    float accel_magnitude[HISTORY_SIZE];
    float gyro_magnitude[HISTORY_SIZE];
    uint8_t index;
    float accel_avg;
    float gyro_avg;
};
SensorHistory sensorHistory = {{0}, {0}, 0, 0.0f, 0.0f};

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

// Instanciar el sensor BNO055 en el bus I2Cbus I2C
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Nueva estructura y variable global para la velocidad lineal
struct LinearVelocity {
    float x;
    float y;
    float z;
};
LinearVelocity linearVelocity = {0.0f, 0.0f, 0.0f};

// Nueva estructura y variable global para la posición estimada (desplazamiento)
struct Position {
    float x;
    float y;
    float z;
};
Position position = {0.0f, 0.0f, 0.0f};

// Nueva estructura global para los offsets de calibración del magnetómetro
struct MagCalibrationOffset {
    float x;
    float y;
    float z;
};
MagCalibrationOffset magOffset = {0.0f, 0.0f, 0.0f}; // Ajustar con valores de calibración

// Añadir estructura y función de filtro Kalman
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

// Añadir variables globales para Kalman de velocidad y posición
static float kalman_velocity_x = 0.0f, kalman_velocity_y = 0.0f, kalman_velocity_z = 0.0f;
static float kalman_position_x = 0.0f, kalman_position_y = 0.0f, kalman_position_z = 0.0f;
static float P_vel = 1.0f, P_pos = 1.0f;
const float kalman_R = 0.1f, kalman_Q = 0.001f;

// Función para imprimir una línea centrada
void printCentered(const char* text) {
    // Versión simplificada sin bordes
    Serial.println(text);
}

// Función para imprimir una sección/ Función para imprimir una sección
void printSection(const char* title) {
    // Versión simplificada de una secciónección
    Serial.println(title);
}

// Función para calcular la magnitud de un vector/ Función para calcular la magnitud de un vector
float calculateMagnitude(const imu::Vector<3>& vec) {
    return sqrt(vec.x() * vec.x() + vec.y() * vec.y() + vec.z() * vec.z());
}

// Función para actualizar el histórico y calcular promedios/ Función para actualizar el histórico y calcular promedios
void updateHistory(float accel_mag, float gyro_mag) {
    sensorHistory.accel_magnitude[sensorHistory.index] = accel_mag;
    sensorHistory.gyro_magnitude[sensorHistory.index] = gyro_mag;
    
    float accel_sum = 0, gyro_sum = 0;
    for(int i = 0; i < HISTORY_SIZE; i++) {
        accel_sum += sensorHistory.accel_magnitude[i];
        gyro_sum += sensorHistory.gyro_magnitude[i];
    }
    
    sensorHistory.accel_avg = accel_sum / HISTORY_SIZE;
    sensorHistory.gyro_avg = gyro_sum / HISTORY_SIZE;
    sensorHistory.index = (sensorHistory.index + 1) % HISTORY_SIZE;
}

// Función para verificar la calibración/ Función para verificar la calibración
bool checkCalibration() {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    return (sys >= CALIBRATION_THRESHOLD && 
            gyro >= CALIBRATION_THRESHOLD && 
            accel >= CALIBRATION_THRESHOLD && 
            mag >= CALIBRATION_THRESHOLD);
}

// Función para inicializar el sensor con reintentos/ Función para inicializar el sensor con reintentos
bool initializeSensor() {
    uint8_t attempts = 0;
    while (attempts < MAX_INIT_ATTEMPTS) {
        if (bno.begin()) {
            bno.setExtCrystalUse(true);
            sensorState.is_initialized = true;
            return true;
        }
        Serial.println(F("Error al inicializar BNO055. Reintentando..."));
        delay(1000);
        attempts++;
    }
    return false;
}

// Función para actualizar el estado del sistema/ Función para actualizar el estado del sistema
void updateSystemState() {
    uint8_t system_status, self_test, system_error;
    bno.getSystemStatus(&system_status, &self_test, &system_error);
    sensorState.system_status = system_status;
    
    if (system_error) {
        sensorState.error_count++;
    }
    
    sensorState.is_calibrated = checkCalibration();
}

// Se agrega nueva función para mostrar la temperatura/ Se agrega nueva función para mostrar la temperatura
void displayTemperature() {
    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");
}

// Actualizar la función de calibración sin persistencia/ Función para calibrar el sensor esperando hasta que alcance la calibración mínima, utilizando LED_BUILTIN para indicar estado
void calibrateSensor() {
    // Si se encuentra en modo debug, se omite la calibración
    if (debug_bno055) {
        Serial.println("Modo debug activado: se omite la calibración.");
        return;
    }
    
    Serial.println("Calibrando sensor...");
    uint32_t calibrationStart = millis();
    while (!checkCalibration() && (millis() - calibrationStart < CALIBRATION_TIMEOUT_MS)) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
        Serial.print(".");
    }
    uint32_t elapsed = millis() - calibrationStart; // Tiempo transcurrido en ms
    
    // Convertir a horas, minutos y segundos
    uint32_t hours   = elapsed / 3600000;               // 3600000 ms en 1 hora
    uint32_t minutes = (elapsed % 3600000) / 60000;       // 60000 ms en 1 minuto
    uint32_t seconds = (elapsed % 60000) / 1000;          // 1000 ms en 1 segundo
    
    if (!checkCalibration()) {
        Serial.println("\nERROR: Tiempo de calibración agotado.");
    } else {
        Serial.println("\nSensor calibrado.");
    }
    Serial.print("Tiempo de calibración: ");
    Serial.print(hours);   Serial.print("h ");
    Serial.print(minutes); Serial.print("m ");
    Serial.print(seconds); Serial.println("s");
    
    digitalWrite(LED_BUILTIN, HIGH); // LED fijo en encendido para indicar toma de datos
}

// Nueva función para convertir el ángulo de la brújula a dirección cardinal
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

// Filtro Complementario para inclinación
void applyComplementaryFilter(float dt, const imu::Vector<3>& accel, const imu::Vector<3>& gyro) {
    static float filteredTilt = 0.0f;
    float accelTilt = atan2(accel.z(), sqrt(accel.x()*accel.x() + accel.y()*accel.y())) * 180.0 / PI;
    filteredTilt = COMPLEMENTARY_ALPHA * (filteredTilt + gyro.x() * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accelTilt;
    Serial.print("📏 Inclinación Complementaria: ");
    Serial.print(filteredTilt, 4); // antes: (filteredTilt, 2)
    Serial.println("°");
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n");
    Serial.println("\n\n");
    // Configurar LED_BUILTIN como salida
    pinMode(LED_BUILTIN, OUTPUT);
    // Se reemplaza la salida con bordes por una simple impresión
    printCentered("Sistema de Orientación BNO055");
    Serial.println("\n");
    
    // Inicializar I2C
    Wire.begin(26, 22);
    delay(100);
    
    // Inicializar sensor
    if (!initializeSensor()) {
        Serial.println("ERROR CRÍTICO: No se pudo inicializar el sensor");
        while (1) delay(10);
    }
    
    printCentered("Sensor Inicializado Correctamente");
    // Llamada a la función de calibración para establecer el estado inicial
    calibrateSensor();
    
    // Inicialización del módulo GPS
    gpsInit();
}

void loop() {
    static uint32_t last_update = millis(); // Inicializar con millis()
    uint32_t current_time = millis();
    uint32_t dt_ms = current_time - last_update;
    
    if (dt_ms >= SAMPLE_RATE_MS) {
        float dt = dt_ms / 1000.0f; // Conversión a segundos
        last_update = current_time;
        updateSystemState();

        if (sensorState.system_status == 0) {
            Serial.println(F("ERROR: Sensor no responde."));
            return;
        }

        // Obtener datos del sensor
        sensors_event_t event;
        bno.getEvent(&event);

        // Datos ya existentes
        imu::Vector<3> euler      = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> accel      = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> gyro       = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        // Nuevos datos solicitados
        imu::Vector<3> mag        = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        imu::Vector<3> accelFull  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> gravity    = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
        imu::Quaternion quat      = bno.getQuat();

        // Calcular datos derivados
        derivedData.total_acceleration = calculateMagnitude(accel);
        derivedData.angular_velocity   = calculateMagnitude(gyro);
        derivedData.tilt_angle         = atan2(accel.z(), sqrt(accel.x() * accel.x() + accel.y() * accel.y())) * 180.0 / PI;

        updateHistory(derivedData.total_acceleration, derivedData.angular_velocity);

        // Integrar aceleración para obtener la velocidad lineal
        linearVelocity.x += accel.x() * dt;
        linearVelocity.y += accel.y() * dt;
        linearVelocity.z += accel.z() * dt;
        
        // Nueva integración: integrar la velocidad para obtener la posición (desplazamiento)
        position.x += linearVelocity.x * dt;
        position.y += linearVelocity.y * dt;
        position.z += linearVelocity.z * dt;
        
        // Aplicar el filtro complementario para inclinación
        applyComplementaryFilter(dt, accel, gyro);

        // Complementary filter para la estimación de la inclinación dinámica
        {
            static float dynamicTilt = 0.0f;
            static bool initDynamic = true;
            float accelTilt = derivedData.tilt_angle; // Inclinación derivada del acelerómetro
            if (initDynamic) {
                dynamicTilt = accelTilt;
                initDynamic = false;
            } else {
                // Se utiliza un factor alfa (ej. 0.98) para integrar el rate del giroscopio (asumido en gyro.x())
                dynamicTilt = 0.98f * (dynamicTilt + gyro.x() * dt) + 0.02f * accelTilt;
            }
            Serial.print("📏 Inclinación Dinámica: "); Serial.print(dynamicTilt, 4); Serial.println("°");
        }

        // --- Se integra el filtro Kalman para magnitudes derivadas ---
        {
            static bool kalmanInitialized = false;
            static KalmanFilter kfTotalAccel, kfAngularVel, kfTilt;
            if (!kalmanInitialized) {
                kfTotalAccel.x = derivedData.total_acceleration;
                kfTotalAccel.P = 1.0f; kfTotalAccel.Q = 0.01f; kfTotalAccel.R = 0.1f;
                kfAngularVel.x = derivedData.angular_velocity;
                kfAngularVel.P = 1.0f; kfAngularVel.Q = 0.01f; kfAngularVel.R = 0.1f;
                kfTilt.x = derivedData.tilt_angle;
                kfTilt.P = 1.0f; kfTilt.Q = 0.01f; kfTilt.R = 0.1f;
                kalmanInitialized = true;
            }
            float kalmanTotalAccel = kalmanUpdate(&kfTotalAccel, derivedData.total_acceleration);
            float kalmanAngularVel = kalmanUpdate(&kfAngularVel, derivedData.angular_velocity);
            float kalmanTilt       = kalmanUpdate(&kfTilt, derivedData.tilt_angle);
            Serial.print("🔎 Kalman Total Accel: "); Serial.print(kalmanTotalAccel, 4); Serial.println(" m/s²");
            Serial.print("🔎 Kalman Angular Vel: "); Serial.print(kalmanAngularVel, 4); Serial.println(" rad/s");
            Serial.print("🔎 Kalman Tilt: "); Serial.print(kalmanTilt, 4); Serial.println("°");
        }
        // --- Fin de integración del filtro Kalman ---

        // --- NUEVA INTEGRACIÓN: Aplicación de filtros Kalman para velocidad y posición ---
        {
            static KalmanFilter kfVelX = {kalman_Q, kalman_R, kalman_velocity_x, P_vel};
            static KalmanFilter kfVelY = {kalman_Q, kalman_R, kalman_velocity_y, P_vel};
            static KalmanFilter kfVelZ = {kalman_Q, kalman_R, kalman_velocity_z, P_vel};
            static KalmanFilter kfPosX = {kalman_Q, kalman_R, kalman_position_x, P_pos};
            static KalmanFilter kfPosY = {kalman_Q, kalman_R, kalman_position_y, P_pos};
            static KalmanFilter kfPosZ = {kalman_Q, kalman_R, kalman_position_z, P_pos};
            
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
        // --- FIN del bloque Kalman ---

        // Nueva salida por consola
        Serial.println("============== SENSOR DATA ==============");
        Serial.print("⏱️  Time: ");            Serial.println(current_time);
        Serial.print("🔧 Calibration: ");      Serial.println(sensorState.is_calibrated ? "YES" : "NO");
        Serial.print("⚠️  Error Count: ");      Serial.println(sensorState.error_count);
        Serial.print("🌡️  Temperature: ");      Serial.println(bno.getTemp());
        displayTemperature();
        Serial.println("-----------------------------------------");
        
        // Euler con unidades en todas las componentes (degrees, de 0 a 359)
        Serial.print("🧭 Euler X: "); Serial.print(euler.x(), 4); Serial.print("°");
        Serial.print(", Y: ");       Serial.print(euler.y(), 4); Serial.print("°");
        Serial.print(", Z: ");       Serial.print(euler.z(), 4); Serial.println("°");
        
        // Nueva línea para mostrar la dirección de la brújula basada en el ángulo Euler X
        {
            float heading = euler.x(); // asumiendo que Euler X corresponde al heading
            const char* compassDir = getCompassDirection(heading);
            Serial.print("🧭 Dirección de la Brújula: ");
            Serial.println(compassDir);
        }
        
        // Aceleración lineal (VECTOR_LINEARACCEL, m/s^2)
        Serial.print("📈 Linear Accel X: "); Serial.print(accel.x(), 4); Serial.print(" m/s²");
        Serial.print(", Y: ");              Serial.print(accel.y(), 4); Serial.print(" m/s²");
        Serial.print(", Z: ");              Serial.print(accel.z(), 4); Serial.println(" m/s²");
        
        // Nueva salida por consola para la velocidad lineal
        Serial.print("🚀 Velocidad Lineal X: "); Serial.print(linearVelocity.x, 4); Serial.print(" m/s");
        Serial.print(" | Y: "); Serial.print(linearVelocity.y, 4); Serial.print(" m/s");
        Serial.print(" | Z: "); Serial.print(linearVelocity.z, 4); Serial.println(" m/s");

        // Nueva salida por consola para la posición estimada
        Serial.print("📍 Posición Estimada X: "); Serial.print(position.x, 4); Serial.print(" m");
        Serial.print(" | Y: "); Serial.print(position.y, 4); Serial.print(" m");
        Serial.print(" | Z: "); Serial.print(position.z, 4); Serial.println(" m");

        // Giroscopio (VECTOR_GYROSCOPE, rps)
        Serial.print("🌀 Gyroscope X: "); Serial.print(gyro.x(), 4); Serial.print(" rps");
        Serial.print(", Y: ");            Serial.print(gyro.y(), 4); Serial.print(" rps");
        Serial.print(", Z: ");            Serial.print(gyro.z(), 4); Serial.println(" rps");
        
        // Nueva sección para calcular y mostrar el Heading Rate (derivado del giroscopio Z)
        {
            float headingRate = gyro.z();
            Serial.print("📏 Heading Rate: "); Serial.print(headingRate, 4); Serial.println(" deg/s");
        }
        
        // Nueva sección para calcular y mostrar el Drift del Giroscopio
        {
            static float driftSumX = 0.0f, driftSumY = 0.0f, driftSumZ = 0.0f;
            static uint32_t driftCount = 0;
            // Si el giroscopio está en reposo (lecturas pequeñas)
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
        
        // Magnetómetro (VECTOR_MAGNETOMETER, uT)
        Serial.print("📡 Magnetometer X: "); Serial.print(mag.x(), 4); Serial.print(" uT");
        Serial.print(", Y: ");               Serial.print(mag.y(), 4); Serial.print(" uT");
        Serial.print(", Z: ");               Serial.print(mag.z(), 4); Serial.println(" uT");
        
        // Se aplica compensación al magnetómetro
        {
            float corrMagX = mag.x() - magOffset.x;
            float corrMagY = mag.y() - magOffset.y;
            float corrMagZ = mag.z() - magOffset.z;
            float corrMagStrength = sqrt(corrMagX*corrMagX + corrMagY*corrMagY + corrMagZ*corrMagZ);
            Serial.print("📡 Magnetómetro compensado X: "); Serial.print(corrMagX, 4); Serial.print(" uT");
            Serial.print(", Y: "); Serial.print(corrMagY, 4); Serial.print(" uT");
            Serial.print(", Z: "); Serial.print(corrMagZ, 4); Serial.println(" uT");
            Serial.print("📡 Fuerza del Campo Magnético (Compensado): "); Serial.print(corrMagStrength, 4); Serial.println(" uT");
        }
        
        // Nueva sección para calcular y mostrar la fuerza del campo magnético
        {
            float magFieldStrength = calculateMagnitude(mag);
            Serial.print("📡 Fuerza del Campo Magnético: ");
            Serial.print(magFieldStrength, 4);
            Serial.println(" uT");
        }
        
        // Acelerómetro completo (VECTOR_ACCELEROMETER, m/s^2)
        Serial.print("📊 Accelerometer X: "); Serial.print(accelFull.x(), 4); Serial.print(" m/s²");
        Serial.print(", Y: ");                Serial.print(accelFull.y(), 4); Serial.print(" m/s²");
        Serial.print(", Z: ");                Serial.print(accelFull.z(), 4); Serial.println(" m/s²");
        
        // Gravedad (VECTOR_GRAVITY, m/s^2)
        Serial.print("🌐 Gravity X: "); Serial.print(gravity.x(), 4); Serial.print(" m/s²");
        Serial.print(", Y: ");           Serial.print(gravity.y(), 4); Serial.print(" m/s²");
        Serial.print(", Z: ");           Serial.print(gravity.z(), 4); Serial.println(" m/s²");
        
        // Total Acceleration con medias
        Serial.print("📊 Total Accel: ");      Serial.print(derivedData.total_acceleration, 4);
        Serial.print(" m/s² (Avg: ");           Serial.print(sensorHistory.accel_avg, 4);
        Serial.println(" m/s²)");
        
        // Angular Velocity con medias
        Serial.print("🌀 Angular Vel: ");      Serial.print(derivedData.angular_velocity, 4);
        Serial.print(" rad/s (Avg: ");          Serial.print(sensorHistory.gyro_avg, 4);
        Serial.println(" rad/s)");
        
        // Tilt Angle
        Serial.print("📐 Tilt Angle: ");       Serial.print(derivedData.tilt_angle, 4); Serial.println("°");
        
        // Quaternion (se asume que no requiere unidades)
        Serial.print("🔀 Quaternion: W: ");    Serial.print(quat.w(), 4);
        Serial.print(" X: ");                Serial.print(quat.x(), 4);
        Serial.print(" Y: ");                Serial.print(quat.y(), 4);
        Serial.print(" Z: ");                Serial.println(quat.z(), 4);
        Serial.println("=========================================\n");
    }
    
    // Llamada al procesamiento del módulo GPS
    gpsProcess();
}
