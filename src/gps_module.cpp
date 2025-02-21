#include "gps_module.h"
#include <math.h>  // Agregado para la fórmula de Haversine
#include <algorithm>  // Añadido para std::sort
#include <cstring>    // Añadido para memcpy
#include "sd_module.h"  // Añadido para usar funciones del módulo SD

#define RXD2 16  // Pin RX del GPS
#define TXD2 17  // Pin TX del GPS

static const int updateInterval = 1000;         
static unsigned long start_time;                
static unsigned long lastUpdate = 0;            
static unsigned long lastGoodSignal = 0;        
static const unsigned long SIGNAL_TIMEOUT = 30000;  
static const int MIN_SATELLITES = 4;            
static const float MAX_ACCEPTABLE_HDOP = 5.0;     
static int maxSatellites = 0;                    
static float bestHDOP = 999;

// Variables para calcular distancia recorrida
static double lastLat = 0.0, lastLon = 0.0;
static double totalDistance = 0.0;  // en km

// Variables para aceleración
static float previousSpeed = 0.0;
static unsigned long lastTime = 0;

// Variable para cambio de rumbo
static float previousCourse = 0.0;

// Variables para tiempo en movimiento/reposo
static unsigned long movingTime = 0;
static unsigned long stoppedTime = 0;
static unsigned long lastStateChange = 0;
static bool isMoving = false;

// Variables para velocidad máxima y media
static float maxSpeed = 0.0;
static float totalSpeed = 0.0;
static int speedCount = 0;
static float avgSpeed = 0.0;

// Variable para aceleración máxima
static float maxAcceleration = 0.0;

// Variable para carga de entrenamiento (TRIMP)
static float trainingLoad = 0.0;

// NUEVA CONSTANTE: Offset geoid para corrección de altitud (ejemplo, ajuste según datos EGM96)
static const float GEOID_OFFSET = 27.0; // Valor de ejemplo en metros

// NUEVAS DEFINICIONES: Ventana temporal para suavizar aceleración
#define WINDOW_SIZE 10
float speedWindow[WINDOW_SIZE] = {0};
int speedWindowIndex = 0;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
static int DEBUG = 0;
static bool gpsConfigured = false;

static void debugNmeaPrint(char c) {
    if (DEBUG == 1) {
        Serial.write(c);
    }
}

static void sendCommand(const char* command) {
    gpsSerial.println(command);
    Serial.print("📡 Enviando comando: ");
    Serial.println(command);
}

// NUEVA FUNCIÓN: Calibración de variables del GPS
// Reinicia todas las variables internas para iniciar una nueva sesión
void calibrateGPSVariables() {
    // Reiniciar variables de posición y distancia
    lastLat = 0.0;
    lastLon = 0.0;
    totalDistance = 0.0;
    
    // Reiniciar variables de velocidad y aceleración
    previousSpeed = 0.0;
    lastTime = millis();
    previousCourse = 0.0;
    maxSpeed = 0.0;
    totalSpeed = 0.0;
    speedCount = 0;
    avgSpeed = 0.0;
    maxAcceleration = 0.0;
    
    // Reiniciar carga de entrenamiento
    trainingLoad = 0.0;
    
    // Reiniciar tiempos en movimiento y en reposo
    movingTime = 0;
    stoppedTime = 0;
    lastStateChange = millis();
    isMoving = false;
    
    Serial.println("✅ Variables del GPS calibradas.");
}

// NUEVA FUNCIÓN: Configurar el GPS para una fijación rápida y mayor precisión
void configureGPS() {
    // Aumenta la tasa de actualización (por ejemplo, a 200 ms -> 5Hz)
    gpsSerial.println("$PMTK220,200*2C"); // Configura actualización cada 200ms (5Hz)
    
    // Limita las sentencias NMEA a solo las necesarias (por ejemplo, GPRMC y GPGGA)
    gpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
    
    // (Opcional) Habilitar modo de alta precisión si el receptor lo permite
    // gpsSerial.println("$PMTK301,2*2E");
    
    Serial.println("✅ GPS configurado para fijación rápida y mayor precisión");
}

void gpsInit() {
    Serial.println("\n🛰️ Configurando GPS para máxima recepción...");
    gpsSerial.setRxBufferSize(1024);
    gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(100);
    gpsConfigured = true;
    Serial.println("✅ Configuración GPS completada");
    start_time = millis();
    lastGoodSignal = millis();
    
    // Calibrar variables del GPS
    calibrateGPSVariables();
    
    // Configurar el GPS para acelerar la fijación y mejorar precisión
    configureGPS();
}

static bool checkSignalQuality() {
    if (!gps.satellites.isValid() || !gps.hdop.isValid()) return false;
    int sats = gps.satellites.value();
    float hdop = gps.hdop.hdop();
    maxSatellites = (sats > maxSatellites) ? sats : maxSatellites;
    bestHDOP = (hdop < bestHDOP) ? hdop : bestHDOP;
    if (sats >= MIN_SATELLITES && hdop <= MAX_ACCEPTABLE_HDOP) {
        lastGoodSignal = millis();
        return true;
    }
    return false;
}

static void checkGPSReset() {
    if ((millis() - lastGoodSignal) > SIGNAL_TIMEOUT && gpsConfigured) {
        Serial.println("\n⚠️ Señal GPS pobre durante mucho tiempo, reconfigurando...");
        gpsConfigured = false;
        gpsInit();
    }
}

// Calcula el cambio de rumbo comparando la dirección actual con la previa
void checkCourseChange() {
    if (gps.course.isValid()) {
        float currentCourse = gps.course.deg();
        float courseChange = currentCourse - previousCourse;
        if (abs(courseChange) > 10.0) {
            Serial.print("🔄 Cambio de rumbo: ");
            Serial.print(courseChange, 2);
            Serial.println("°");
        }
        previousCourse = currentCourse;
    }
}

// Calcula y muestra la aceleración
float getSmoothedAcceleration(float newSpeed, unsigned long deltaTime) {
    speedWindow[speedWindowIndex] = newSpeed;
    speedWindowIndex = (speedWindowIndex + 1) % WINDOW_SIZE;
    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += speedWindow[i];
    }
    float avgSpeed = sum / WINDOW_SIZE;
    return (newSpeed - avgSpeed) / (deltaTime / 1000.0);
}

// Modificar calculateAcceleration() para usar el filtro de aceleración suavizada
void calculateAcceleration() {
    if (gps.speed.isValid()) {
        float currentSpeed = gps.speed.kmph();
        unsigned long currentTime = millis();
        unsigned long dt = currentTime - lastTime;
        if (dt > 0) {
            float smoothedAcc = getSmoothedAcceleration(currentSpeed, dt);
            Serial.print("📈 Aceleración (Suavizada): ");
            Serial.print(smoothedAcc, 2);
            Serial.println(" m/s²");
        }
        lastTime = currentTime;
    }
}

// Actualiza y muestra la aceleración máxima registrada
void updateAcceleration() {
    if (gps.speed.isValid()) {
        float currentSpeed = gps.speed.kmph();
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0;
        if (deltaTime > 0) {
            float acceleration = (currentSpeed - previousSpeed) / deltaTime;
            if (abs(acceleration) > maxAcceleration) {
                maxAcceleration = abs(acceleration);
            }
        }
        previousSpeed = currentSpeed;
        lastTime = currentTime;
    }
    Serial.print("⚡ Aceleración Máxima: ");
    Serial.print(maxAcceleration, 2);
    Serial.println(" m/s²");
}

// Calcula la distancia usando la fórmula de Haversine
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371.0;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

// MODIFICACIÓN DE updateDistance(): Se muestra la distancia en metros (totalDistance en km * 1000)
void updateDistance() {
    if (gps.location.isValid()) {
        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();
        if (lastLat != 0.0 && lastLon != 0.0) {
            totalDistance += calculateDistance(lastLat, lastLon, currentLat, currentLon);
        }
        lastLat = currentLat;
        lastLon = currentLon;
    }
    // Convertir kilómetros a metros
    double totalDistanceMeters = totalDistance * 1000;
    Serial.print("🏃 Distancia total: ");
    Serial.print(totalDistanceMeters, 0);
    Serial.println(" m");
}

// NUEVAS DEFINICIONES: Filtros para suavizar la velocidad
#define FILTER_SIZE 5  // Número de muestras para el promedio móvil
float speedBuffer[FILTER_SIZE] = {0};
int speedIndex = 0;

float getFilteredSpeed(float newSpeed) {
    speedBuffer[speedIndex] = newSpeed;
    speedIndex = (speedIndex + 1) % FILTER_SIZE;
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += speedBuffer[i];
    }
    return sum / FILTER_SIZE;
}

#define MEDIAN_FILTER_SIZE 5  // Número de muestras para el filtro de mediana
float speedValues[MEDIAN_FILTER_SIZE] = {0};

float getMedianSpeed(float newSpeed) {
    // Desplazar los valores previos
    for (int i = MEDIAN_FILTER_SIZE - 1; i > 0; i--) {
        speedValues[i] = speedValues[i - 1];
    }
    speedValues[0] = newSpeed;
    float sorted[MEDIAN_FILTER_SIZE];
    memcpy(sorted, speedValues, sizeof(speedValues));
    std::sort(sorted, speedValues + MEDIAN_FILTER_SIZE);
    return sorted[MEDIAN_FILTER_SIZE / 2];  // Devuelve la mediana
}

// Modificar updateSpeedMetrics para usar los filtros de velocidad
void updateSpeedMetrics() {
    if (gps.speed.isValid()) {
        float rawSpeed = gps.speed.kmph();
        float filteredSpeed = getFilteredSpeed(rawSpeed);
        float medianSpeed = getMedianSpeed(rawSpeed);
        // Usar el valor mediano como velocidad "limpia"
        float currentSpeed = medianSpeed;
        
        if (currentSpeed > maxSpeed) {
            maxSpeed = currentSpeed;
        }
        totalSpeed += currentSpeed;
        speedCount++;
        avgSpeed = totalSpeed / speedCount;
    }
    Serial.print("🚀 Velocidad Máxima: ");
    Serial.print(maxSpeed, 2);
    Serial.println(" km/h");
    Serial.print("📊 Velocidad Media: ");
    Serial.print(avgSpeed, 2);
    Serial.println(" km/h");
}

// Estima la cadencia en pasos por minuto (SPM)
void estimateCadence() {
    if (gps.speed.isValid()) {
        float speed = gps.speed.kmph();
        int estimatedCadence = (int)(speed * 15);
        Serial.print("🏃‍♂️ Cadencia estimada: ");
        Serial.print(estimatedCadence);
        Serial.println(" SPM");
    }
}

// Calcula la carga de entrenamiento (TRIMP) combinando velocidad y distancia
void calculateTrainingLoad() {
    if (gps.speed.isValid()) {
        float speed = gps.speed.kmph();
        float distance = calculateDistance(lastLat, lastLon, gps.location.lat(), gps.location.lng());
        trainingLoad += (speed * distance);
    }
    Serial.print("💪 Índice de Esfuerzo (TRIMP): ");
    Serial.print(trainingLoad, 2);
    Serial.println(" u");
}

// Seguimiento del tiempo en movimiento y en reposo
void trackMovementTime() {
    float speed = gps.speed.kmph();
    unsigned long currentTime = millis();
    if (speed > 1.0 && !isMoving) {
        isMoving = true;
        lastStateChange = currentTime;
    } else if (speed <= 1.0 && isMoving) {
        isMoving = false;
        lastStateChange = currentTime;
    }
    if (isMoving) {
        movingTime += (currentTime - lastStateChange);
    } else {
        stoppedTime += (currentTime - lastStateChange);
    }
    lastStateChange = currentTime;
    Serial.print("⏱️ Tiempo en movimiento: ");
    Serial.print(movingTime / 1000);
    Serial.print(" s | Tiempo detenido: ");
    Serial.print(stoppedTime / 1000);
    Serial.println(" s");
}

// Detecta giros bruscos comparando la velocidad y el cambio de rumbo
void detectTurns() {
    if (gps.speed.isValid() && gps.course.isValid()) {
        float speed = gps.speed.kmph();
        float courseChange = abs(gps.course.deg() - previousCourse);
        if (speed > 5.0 && courseChange > 15.0) {
            Serial.println("🚗⚠️ Giro detectado!");
        }
        previousCourse = gps.course.deg();
    }
}

// NUEVA FUNCIÓN: Mostrar fecha y hora del GPS
void displayGPSTime() {
    if (gps.time.isValid() && gps.date.isValid()) {
        Serial.print("🕒 Fecha: ");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.print(gps.date.year());
        Serial.print(" - Hora: ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());
    } else {
        Serial.println("⌛ Sin hora GPS");
    }
}

// NUEVA FUNCIÓN: Ajusta la altitud usando el modelo EGM96
float adjustAltitude(float gpsAltitude, float geoidOffset) {
    return gpsAltitude - geoidOffset;
}

// NUEVA FUNCIÓN: Validación de Datos GPS (Para Evitar Saltos de Posición)
bool isValidGPSFix() {
    if (!gps.location.isValid()) return false;
    if (!gps.speed.isValid()) return false;
    if (!gps.satellites.isValid() || gps.satellites.value() < 4) return false;
    if (!gps.hdop.isValid() || gps.hdop.hdop() > 5.0) return false;
    return true;
}

void gpsProcess() {
    // Leer datos del GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        debugNmeaPrint(c);
        gps.encode(c);
    }
    
    checkGPSReset();
    
    // Variables estáticas para detectar transición de FIX perdido a FIX obtenido
    static bool fixActive = false;
    static unsigned long fixLostTime = start_time;  // Se usa start_time para el primer FIX
    
    if (millis() - lastUpdate >= updateInterval) {
        lastUpdate = millis();
        Serial.println("\n===== 🛰️ DIAGNÓSTICO GPS =====\n");
        if (gps.satellites.isValid()) {
            int sats = gps.satellites.value();
            Serial.print("📡 Satélites: ");
            Serial.println(sats);
            if (gps.hdop.isValid()) {
                float hdop = gps.hdop.hdop();
                Serial.print("🔧 HDOP: ");
                Serial.println(String(hdop, 2));
            }
        }
        unsigned long uptime = (millis() - start_time) / 1000;
        Serial.print("⏱️ Tiempo activo: ");
        Serial.print(uptime);
        Serial.println(" s\n");
        
        if (isValidGPSFix()) {  // Se usa la validación de datos GPS
            // Guardar configuración GPS en SD solo una vez
            static bool configSaved = false;
            if (!configSaved) {
                String config;
                config += "Fecha: " + String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year()) + "\n";
                config += "Hora: " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\n";
                config += "Satélites: " + String(gps.satellites.value()) + "\n";
                config += "HDOP: " + String(gps.hdop.hdop(), 2) + "\n";
                if (createFileSD("/gps_config.txt")) {
                    appendDataSD("/gps_config.txt", config.c_str());
                    Serial.println("✅ Configuración GPS guardada en SD.");
                    configSaved = true;
                } else {
                    Serial.println("⚠️ Error al guardar la configuración GPS en SD.");
                }
            }
            
            // Detectar transición: si recibiendo FIX después de no tenerlo, se guardan métricas adicionales
            if (!fixActive) {
                fixActive = true;
                unsigned long fixAcquisitionTime = millis() - fixLostTime;
                String fixData;
                fixData += "Hora: " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\n";
                fixData += "Latitud: " + String(gps.location.lat(), 6) + "\n";
                fixData += "Longitud: " + String(gps.location.lng(), 6) + "\n";
                fixData += "Satélites: " + String(gps.satellites.value()) + "\n";
                fixData += "HDOP: " + String(gps.hdop.hdop(), 2) + "\n";
                fixData += "Tiempo hasta FIX: " + String(fixAcquisitionTime) + " ms\n";
                fixData += "--------------------------------\n";
                static bool fileCreated = false;
                if (!fileCreated) {
                    if (createFileSD("/gps_fixes.txt")) {
                        fileCreated = true;
                    } else {
                        Serial.println("⚠️ Error al crear el fichero de FIX GPS.");
                    }
                }
                if (fileCreated) {
                    appendDataSD("/gps_fixes.txt", fixData.c_str());
                    Serial.println("✅ Fijación GPS guardada en SD para debug:");
                    Serial.println(fixData);
                }
            }
            // Continuación de la impresión del FIX actual
            double curLat = gps.location.lat();
            double curLon = gps.location.lng();
            Serial.println("✅ GPS FIX:");
            Serial.print("   📍 Latitud:  ");
            Serial.println(curLat, 6);
            Serial.print("   📍 Longitud: ");
            Serial.println(curLon, 6);
            Serial.print("   📡 Satélites: ");
            Serial.println(gps.satellites.value());
            Serial.print("   🔧 HDOP: ");
            Serial.println(String(gps.hdop.hdop(), 2));
            // Usar la corrección de altitud
            float rawAltitude = gps.altitude.meters();
            float correctedAltitude = adjustAltitude(rawAltitude, GEOID_OFFSET);
            Serial.print("   🏔️ Altitud :  ");
            Serial.print(correctedAltitude);
            Serial.println(" m");
            Serial.print("   🚀 Velocidad: ");
            Serial.print(gps.speed.kmph(), 2);
            Serial.println(" km/h");
            Serial.print("   🧭 Dirección: ");
            Serial.print(gps.course.deg(), 2);
            Serial.println("°\n");

            // Mostrar la fecha y hora del GPS
            displayGPSTime();
            
            // Actualización de métricas
            // NUEVA LLAMADA: Calcular carga de entrenamiento
            calculateTrainingLoad();
            // Separator added after TRIMP
            Serial.println("-----------------------------------------\n");
            
            // Continuación de la actualización de métricas
            updateDistance();
            calculateAcceleration();
            checkCourseChange();
            trackMovementTime();
            detectTurns();
            updateSpeedMetrics();
            updateAcceleration();
            estimateCadence();
        } else {
            // Si el FIX no es válido, reiniciamos la bandera y actualizamos el tiempo de pérdida
            if (fixActive) {
                fixActive = false;
            }
            fixLostTime = millis();
            Serial.println("❌ Datos GPS no válidos\n");
        }
    }
    
    delay(100);
}
