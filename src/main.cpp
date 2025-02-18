#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ===== CONFIGURACIÓN GPS =====
#define RXD2 16  // RX del GPS
#define TXD2 17  // TX del GPS
int DEBUG = 0;
bool debugNMEA = true;
unsigned long start_time;                
unsigned long lastUpdate = 0;            
const int updateInterval = 1000;         
unsigned long lastGoodSignal = 0;        
const unsigned long SIGNAL_TIMEOUT = 30000;  
const int MIN_SATELLITES = 4;            
const float MAX_ACCEPTABLE_HDOP = 5.0;     
bool gpsConfigured = false;              
float lastSpeed = 0;                     
unsigned long lastSpeedTime = 0;
int maxSatellites = 0;                    
float bestHDOP = 999;                    
TinyGPSPlus gps;                         
HardwareSerial gpsSerial(1);

// ===== FUNCIONES GPS =====
void sendCommand(const char* command) {
    gpsSerial.println(command);
    Serial.print("📡 Enviando comando: ");
    Serial.println(command);
}

void debugNmeaPrint(char c) {
    if (DEBUG == 1) {
        Serial.write(c);
    }
}

// La función configureGPS() se simplifica para centrarse en GPS
void configureGPS() {
    Serial.println("\n🛰️ Configurando GPS para máxima recepción...");
    gpsSerial.setRxBufferSize(1024);
    gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(100);
    gpsConfigured = true;
    Serial.println("✅ Configuración GPS completada");
    start_time = millis();
    lastGoodSignal = millis();
}

bool checkSignalQuality() {
    if (!gps.satellites.isValid() || !gps.hdop.isValid()) return false;
    int sats = gps.satellites.value();
    float hdop = gps.hdop.hdop();
    maxSatellites = max(maxSatellites, sats);
    bestHDOP = min(bestHDOP, hdop);
    if (sats >= MIN_SATELLITES && hdop <= MAX_ACCEPTABLE_HDOP) {
        lastGoodSignal = millis();
        return true;
    }
    return false;
}

void checkGPSReset() {
    if (millis() - lastGoodSignal > SIGNAL_TIMEOUT && gpsConfigured) {
        Serial.println("\n⚠️ Señal GPS pobre durante mucho tiempo, reconfigurando...");
        gpsConfigured = false;
        configureGPS();
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial); // Esperar conexión del monitor serie

    // Inicialización exclusiva del GPS
    configureGPS();
}

void loop() {
    // Procesar datos del GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        debugNmeaPrint(c);
        gps.encode(c);
    }
    
    checkGPSReset();
    
    // Actualización de diagnóstico GPS cada segundo
    if (millis() - lastUpdate >= updateInterval) {
        lastUpdate = millis();
        Serial.println("\n===== 🛰️ DIAGNÓSTICO GPS =====");
        if (gps.satellites.isValid()) {
            int sats = gps.satellites.value();
            Serial.print("- Satélites actuales: ");
            Serial.println(sats);
            if (gps.hdop.isValid()) {
                float hdop = gps.hdop.hdop();
                Serial.print("- HDOP actual: ");
                Serial.println(hdop);
            }
        }
        unsigned long current_time = (millis() - start_time) / 1000;
        Serial.print("- Tiempo desde inicio: ");
        Serial.print(current_time);
        Serial.println(" segundos");
        // Mostrar datos de posición si están disponibles
        if (gps.location.isValid()) {
            Serial.println("\n✅ POSICIÓN FIJADA:");
            Serial.print("Latitud: ");
            Serial.println(gps.location.lat(), 6);
            Serial.print("Longitud: ");
            Serial.println(gps.location.lng(), 6);
            Serial.print("Altitud GPS: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" m");
            Serial.print("Velocidad: ");
            Serial.println(gps.speed.kmph());
            Serial.println(" km/h");
            Serial.print("Dirección: ");
            Serial.print(gps.course.deg());
            Serial.println("°");
        } else {
            Serial.println("Sin fix GPS");
        }
    }
    
    delay(100);
}
