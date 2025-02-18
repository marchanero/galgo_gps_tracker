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
    static uint32_t last_update = 0;
    uint32_t current_time = millis();

    if (current_time - last_update >= SAMPLE_RATE_MS) {
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
        
        // Aceleración lineal (VECTOR_LINEARACCEL, m/s^2)
        Serial.print("📈 Linear Accel X: "); Serial.print(accel.x(), 2); Serial.print(" m/s²");
        Serial.print(", Y: ");              Serial.print(accel.y(), 2); Serial.print(" m/s²");
        Serial.print(", Z: ");              Serial.print(accel.z(), 2); Serial.println(" m/s²");
        
        // Giroscopio (VECTOR_GYROSCOPE, rps)
        Serial.print("🌀 Gyroscope X: "); Serial.print(gyro.x(), 2); Serial.print(" rps");
        Serial.print(", Y: ");            Serial.print(gyro.y(), 2); Serial.print(" rps");
        Serial.print(", Z: ");            Serial.print(gyro.z(), 2); Serial.println(" rps");
        
        // Magnetómetro (VECTOR_MAGNETOMETER, uT)
        Serial.print("📡 Magnetometer X: "); Serial.print(mag.x(), 2); Serial.print(" uT");
        Serial.print(", Y: ");               Serial.print(mag.y(), 2); Serial.print(" uT");
        Serial.print(", Z: ");               Serial.print(mag.z(), 2); Serial.println(" uT");
        
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
