#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "utility/imumaths.h"

// Constantes para la configuración del sistema// Constantes para la configuración del sistema
const uint16_t SAMPLE_RATE_MS = 100;  // Tasa de muestreo en milisegundose muestreo en milisegundos
const uint8_t HISTORY_SIZE = 10;      // Tamaño del histórico para promediosios
const float GRAVITY_EARTH = 9.80665F; // Constante de gravedad terrestre
const uint8_t CALIBRATION_THRESHOLD = 2; // Umbral mínimo de calibración
const uint8_t MAX_INIT_ATTEMPTS = 3;    // Máximo número de intentos de inicializacióninicialización

// Nuevas variables globales para control de calibración
const uint32_t CALIBRATION_TIMEOUT_MS = 30000; // 30 segundos de tiempo máximo de calibración
bool debug_bno055 = false; // Flag para modo debug. Si es true, se omite la calibración

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
            Serial.print("📏 Inclinación Dinámica: "); Serial.print(dynamicTilt, 2); Serial.println("°");
        }

        // Nueva salida por consola
        Serial.println("============== SENSOR DATA ==============");
        Serial.print("⏱️  Time: ");            Serial.println(current_time);
        Serial.print("🔧 Calibration: ");      Serial.println(sensorState.is_calibrated ? "YES" : "NO");
        Serial.print("⚠️  Error Count: ");      Serial.println(sensorState.error_count);
        Serial.print("🌡️  Temperature: ");      Serial.println(bno.getTemp());
        displayTemperature();
        Serial.println("-----------------------------------------");
        
        // Euler con unidades en todas las componentes (degrees, de 0 a 359)
        Serial.print("🧭 Euler X: "); Serial.print(euler.x(), 2); Serial.print("°");
        Serial.print(", Y: ");       Serial.print(euler.y(), 2); Serial.print("°");
        Serial.print(", Z: ");       Serial.print(euler.z(), 2); Serial.println("°");
        
        // Nueva línea para mostrar la dirección de la brújula basada en el ángulo Euler X
        {
            float heading = euler.x(); // asumiendo que Euler X corresponde al heading
            const char* compassDir = getCompassDirection(heading);
            Serial.print("🧭 Dirección de la Brújula: ");
            Serial.println(compassDir);
        }
        
        // Aceleración lineal (VECTOR_LINEARACCEL, m/s^2)
        Serial.print("📈 Linear Accel X: "); Serial.print(accel.x(), 2); Serial.print(" m/s²");
        Serial.print(", Y: ");              Serial.print(accel.y(), 2); Serial.print(" m/s²");
        Serial.print(", Z: ");              Serial.print(accel.z(), 2); Serial.println(" m/s²");
        
        // Nueva salida por consola para la velocidad lineal
        Serial.print("🚀 Velocidad Lineal X: "); Serial.print(linearVelocity.x, 2); Serial.print(" m/s");
        Serial.print(" | Y: "); Serial.print(linearVelocity.y, 2); Serial.print(" m/s");
        Serial.print(" | Z: "); Serial.print(linearVelocity.z, 2); Serial.println(" m/s");

        // Nueva salida por consola para la posición estimada
        Serial.print("📍 Posición Estimada X: "); Serial.print(position.x, 2); Serial.print(" m");
        Serial.print(" | Y: "); Serial.print(position.y, 2); Serial.print(" m");
        Serial.print(" | Z: "); Serial.print(position.z, 2); Serial.println(" m");

        // Giroscopio (VECTOR_GYROSCOPE, rps)
        Serial.print("🌀 Gyroscope X: "); Serial.print(gyro.x(), 2); Serial.print(" rps");
        Serial.print(", Y: ");            Serial.print(gyro.y(), 2); Serial.print(" rps");
        Serial.print(", Z: ");            Serial.print(gyro.z(), 2); Serial.println(" rps");
        
        // Nueva sección para calcular y mostrar el Heading Rate (derivado del giroscopio Z)
        {
            float headingRate = gyro.z();
            Serial.print("📏 Heading Rate: "); Serial.print(headingRate, 2); Serial.println(" deg/s");
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
            Serial.print("🧭 Gyro Drift X: "); Serial.print(avgDriftX, 2); Serial.print(" rps");
            Serial.print(" | Y: "); Serial.print(avgDriftY, 2); Serial.print(" rps");
            Serial.print(" | Z: "); Serial.print(avgDriftZ, 2); Serial.println(" rps");
        }
        
        // Magnetómetro (VECTOR_MAGNETOMETER, uT)
        Serial.print("📡 Magnetometer X: "); Serial.print(mag.x(), 2); Serial.print(" uT");
        Serial.print(", Y: ");               Serial.print(mag.y(), 2); Serial.print(" uT");
        Serial.print(", Z: ");               Serial.print(mag.z(), 2); Serial.println(" uT");
        
        // Nueva sección para calcular y mostrar la fuerza del campo magnético
        {
            float magFieldStrength = calculateMagnitude(mag);
            Serial.print("📡 Fuerza del Campo Magnético: ");
            Serial.print(magFieldStrength, 2);
            Serial.println(" uT");
        }
        
        // Acelerómetro completo (VECTOR_ACCELEROMETER, m/s^2)
        Serial.print("📊 Accelerometer X: "); Serial.print(accelFull.x(), 2); Serial.print(" m/s²");
        Serial.print(", Y: ");                Serial.print(accelFull.y(), 2); Serial.print(" m/s²");
        Serial.print(", Z: ");                Serial.print(accelFull.z(), 2); Serial.println(" m/s²");
        
        // Gravedad (VECTOR_GRAVITY, m/s^2)
        Serial.print("🌐 Gravity X: "); Serial.print(gravity.x(), 2); Serial.print(" m/s²");
        Serial.print(", Y: ");           Serial.print(gravity.y(), 2); Serial.print(" m/s²");
        Serial.print(", Z: ");           Serial.print(gravity.z(), 2); Serial.println(" m/s²");
        
        // Total Acceleration con medias
        Serial.print("📊 Total Accel: ");      Serial.print(derivedData.total_acceleration, 2);
        Serial.print(" m/s² (Avg: ");           Serial.print(sensorHistory.accel_avg, 2);
        Serial.println(" m/s²)");
        
        // Angular Velocity con medias
        Serial.print("🌀 Angular Vel: ");      Serial.print(derivedData.angular_velocity, 2);
        Serial.print(" rad/s (Avg: ");          Serial.print(sensorHistory.gyro_avg, 2);
        Serial.println(" rad/s)");
        
        // Tilt Angle
        Serial.print("📐 Tilt Angle: ");       Serial.print(derivedData.tilt_angle, 2); Serial.println("°");
        
        // Quaternion (se asume que no requiere unidades)
        Serial.print("🔀 Quaternion: W: ");    Serial.print(quat.w(), 4);
        Serial.print(" X: ");                Serial.print(quat.x(), 4);
        Serial.print(" Y: ");                Serial.print(quat.y(), 4);
        Serial.print(" Z: ");                Serial.println(quat.z(), 4);
        Serial.println("=========================================\n");
    }
}
