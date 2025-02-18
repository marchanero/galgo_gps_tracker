#include "gps_module.h"
#include <math.h>  // Agregado para la fórmula de Haversine

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
void calculateAcceleration() {
    if (gps.speed.isValid()) {
        float currentSpeed = gps.speed.kmph();
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0;
        if (deltaTime > 0) {
            float acceleration = (currentSpeed - previousSpeed) / deltaTime;
            Serial.print("📈 Aceleración: ");
            Serial.print(acceleration, 2);
            Serial.println(" m/s²");
        }
        previousSpeed = currentSpeed;
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

// Actualiza y muestra la distancia total recorrida
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
    Serial.print("🏃 Distancia total: ");
    Serial.print(totalDistance, 2);
    Serial.println(" km");
}

// Actualiza y muestra la velocidad máxima y media
void updateSpeedMetrics() {
    if (gps.speed.isValid()) {
        float currentSpeed = gps.speed.kmph();
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

void gpsProcess() {
    // Leer datos del GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        debugNmeaPrint(c);
        gps.encode(c);
    }
    
    checkGPSReset();
    
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
        
        if (gps.location.isValid()) {
            double curLat = gps.location.lat();
            double curLon = gps.location.lng();
            Serial.println("✅ GPS FIX:");
            Serial.print("   📍 Latitud:  ");
            Serial.println(curLat, 6);
            Serial.print("   📍 Longitud: ");
            Serial.println(curLon, 6);
            Serial.print("   🏔️ Altitud:  ");
            Serial.print(gps.altitude.meters());
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
            updateDistance();
            calculateAcceleration();
            checkCourseChange();
            trackMovementTime();
            detectTurns();
            updateSpeedMetrics();
            updateAcceleration();
            estimateCadence();
            calculateTrainingLoad();
        } else {
            Serial.println("❌ Sin fix GPS\n");
        }
    }
    
    delay(100);
}
