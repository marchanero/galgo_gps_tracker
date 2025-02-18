#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "utility/imumaths.h"

// Constantes para la configuración del sistema
const uint16_t SAMPLE_RATE_MS = 100;  // Tasa de muestreo en milisegundos
const uint8_t HISTORY_SIZE = 10;      // Tamaño del histórico para promedios
const float GRAVITY_EARTH = 9.80665F; // Constante de gravedad terrestre
const uint8_t CALIBRATION_THRESHOLD = 2; // Umbral mínimo de calibración
const uint8_t MAX_INIT_ATTEMPTS = 3;    // Máximo número de intentos de inicialización

// Estructura para almacenar el histórico de datos del sensor
struct SensorHistory {
    float accel_magnitude[HISTORY_SIZE];
    float gyro_magnitude[HISTORY_SIZE];
    uint8_t index;
    float accel_avg;
    float gyro_avg;
} sensorHistory = {{0}, {0}, 0, 0.0f, 0.0f};

// Estructura para almacenar datos derivados de las mediciones
struct DerivedData {
    float total_acceleration;
    float angular_velocity;
    float tilt_angle;
    uint32_t last_update;
} derivedData = {0.0f, 0.0f, 0.0f, 0};

// Estructura para monitorear el estado del sensor
struct SensorState {
    bool is_calibrated;
    uint8_t system_status;
    uint8_t error_count;
    bool is_initialized;
} sensorState = {false, 0, 0, false};

// Instanciar el sensor BNO055 en el bus I2C
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Función para imprimir una línea centrada
void printCentered(const char* text) {
    // Versión simplificada sin bordes
    Serial.println(text);
}

// Función para imprimir una sección
void printSection(const char* title) {
    // Versión simplificada de una sección
    Serial.println(title);
}

// Función para calcular la magnitud de un vector
float calculateMagnitude(const imu::Vector<3>& vec) {
    return sqrt(vec.x() * vec.x() + vec.y() * vec.y() + vec.z() * vec.z());
}

// Función para actualizar el histórico y calcular promedios
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

// Función para verificar la calibración
bool checkCalibration() {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    return (sys >= CALIBRATION_THRESHOLD && 
            gyro >= CALIBRATION_THRESHOLD && 
            accel >= CALIBRATION_THRESHOLD && 
            mag >= CALIBRATION_THRESHOLD);
}

// Función para inicializar el sensor con reintentos
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

// Función para actualizar el estado del sistema
void updateSystemState() {
    uint8_t system_status, self_test, system_error;
    bno.getSystemStatus(&system_status, &self_test, &system_error);
    sensorState.system_status = system_status;
    
    if (system_error) {
        sensorState.error_count++;
    }
    
    sensorState.is_calibrated = checkCalibration();
}

// Se agrega nueva función para mostrar la temperatura
void displayTemperature() {
    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n");
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
        // Llamada a la nueva función para mostrar la temperatura
        displayTemperature();
        Serial.println("-----------------------------------------");
        Serial.print("🧭 Euler X: ");           Serial.print(euler.x(), 2);
        Serial.print("°, Y: ");                Serial.print(euler.y(), 2);
        Serial.print("°, Z: ");                Serial.println(euler.z(), 2);
        Serial.print("📈 Linear Accel X: ");   Serial.print(accel.x(), 2);
        Serial.print(" m/s², Y: ");            Serial.print(accel.y(), 2);
        Serial.print(" m/s², Z: ");            Serial.println(accel.z(), 2);
        Serial.print("🌀 Gyroscope X: ");      Serial.print(gyro.x(), 2);
        Serial.print(" rad/s, Y: ");           Serial.print(gyro.y(), 2);
        Serial.print(" rad/s, Z: ");           Serial.println(gyro.z(), 2);
        Serial.print("📡 Magnetometer X: ");  Serial.print(mag.x(), 2);
        Serial.print(" uT, Y: ");             Serial.print(mag.y(), 2);
        Serial.print(" uT, Z: ");             Serial.println(mag.z(), 2);
        Serial.print("📊 Accelerometer X: "); Serial.print(accelFull.x(), 2);
        Serial.print(" m/s², Y: ");           Serial.print(accelFull.y(), 2);
        Serial.print(" m/s², Z: ");           Serial.println(accelFull.z(), 2);
        Serial.print("🌐 Gravity X: ");       Serial.print(gravity.x(), 2);
        Serial.print(" m/s², Y: ");           Serial.print(gravity.y(), 2);
        Serial.print(" m/s², Z: ");           Serial.println(gravity.z(), 2);
        Serial.print("📊 Total Accel: ");      Serial.print(derivedData.total_acceleration, 2);
        Serial.print(" m/s² (Avg: ");          Serial.print(sensorHistory.accel_avg, 2);
        Serial.println(")");
        Serial.print("🌀 Angular Vel: ");      Serial.print(derivedData.angular_velocity, 2);
        Serial.print(" rad/s (Avg: ");         Serial.print(sensorHistory.gyro_avg, 2);
        Serial.println(")");
        Serial.print("📐 Tilt Angle: ");       Serial.print(derivedData.tilt_angle, 2);
        Serial.println("°");
        Serial.print("🔀 Quaternion: W: ");    Serial.print(quat.w(), 4);
        Serial.print(" X: ");                Serial.print(quat.x(), 4);
        Serial.print(" Y: ");                Serial.print(quat.y(), 4);
        Serial.print(" Z: ");                Serial.println(quat.z(), 4);
        Serial.println("=========================================\n");
    }
}
