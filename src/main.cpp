#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Actualización para Wemos Lolin32 Lite: usar UART2 con pines 16 (RX) y 17 (TX)
#define RXD2 16  // RX del GPS (conectar al TX del GPS)
#define TXD2 17  // TX del GPS (conectar al RX del GPS)

// Declaración para controlar la impresión NMEA: 1 = debug activo, 0 = debug inactivo
int DEBUG = 0;

// Activar debug: cambiar a false para deshabilitar impresión NMEA
bool debugNMEA = true;

// Variables de control y tiempo
unsigned long start_time;                // Para medir el tiempo de espera
unsigned long lastUpdate = 0;            // Control de actualizaciones
const int updateInterval = 1000;         // Actualizar cada 1 segundo
unsigned long lastGoodSignal = 0;        // Último momento con buena señal
const unsigned long SIGNAL_TIMEOUT = 30000;  // 30 segundos sin señal para reiniciar
const int MIN_SATELLITES = 4;            // Mínimo de satélites para buena señal
const float MAX_ACCEPTABLE_HDOP = 5.0;     // HDOP máximo aceptable
bool gpsConfigured = false;              // Control de configuración

// Variables para estadísticas
float lastSpeed = 0;                     // Para calcular la aceleración
unsigned long lastSpeedTime = 0;
int maxSatellites = 0;                   // Máximo de satélites vistos
float bestHDOP = 999;                    // Mejor HDOP registrado

TinyGPSPlus gps;                         // Instancia de TinyGPS++
HardwareSerial gpsSerial(1);              // UART1 para GPS

// Función para enviar comandos NMEA al GPS
void sendCommand(const char* command) {
    gpsSerial.println(command);
    Serial.print("📡 Enviando comando: ");
    Serial.println(command);
}

// Función para imprimir datos NMEA solo cuando DEBUG es 1
void debugNmeaPrint(char c) {
    if (DEBUG == 1) {
        Serial.write(c);
    }
}

// Función para configurar el GPS
void configureGPS() {
    Serial.println("\n🛰️ Configurando GPS para máxima recepción...");
    
    // Optimizar UART para ESP32: aumentar tamaño del buffer y estabilizar
    gpsSerial.setRxBufferSize(1024);
    gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(100); // Esperar estabilización

    // Configuración de antena y señal
    sendCommand("$PCAS06,1,1*22"); // Control de antena activa
    sendCommand("$PCAS07,1*21");   // Detección de antena
    sendCommand("$PCAS08,1*20");   // Alimentación de antena

    // Preconfigurar ubicación aproximada (Albacete: 39.0°N, -1.8°E, ~686m)
    sendCommand("$PCAS01,39.0,-1.8,686*XX"); // Nota: actualizar checksum si es necesario

    // Configuración de ultra sensibilidad
    sendCommand("$PCAS09,4*2E"); // Modo de máxima sensibilidad posible
    sendCommand("$PCAS16,2,2,2,2*27"); // Tracking agresivo de señales débiles
    
    // Optimizar para satélites visibles en Albacete
    sendCommand("$PCAS10,3,3,3,3,2,2,2,2,1,1,2,2,1,1,0,2,2,1,1,0,0,0,0*1E");

    // Ajustar filtros para máxima sensibilidad
    sendCommand("$PCAS11,0,1,2,3*18"); // Filtros mínimos para mejor sensibilidad

    // Configuración SBAS optimizada para EGNOS en España
    sendCommand("$PCAS13,2,1,1*2A"); // SBAS modo agresivo, solo EGNOS
    sendCommand("$PCAS18,1,1*2D");   // Correcciones ionosféricas regionales

    // Ajustes de adquisición para señales débiles
    sendCommand("$PCAS04,10*1D"); // HDOP muy permisivo para señales débiles
    sendCommand("$PCAS05,1*18");  // Mínimo absoluto de satélites
    
    // Deshabilitar filtros de ruido para máxima sensibilidad
    sendCommand("$PCAS19,0*2F"); // Desactivar filtrado de ruido
    sendCommand("$PCAS20,1*2E"); // Modo de seguimiento continuo

    // Mensajes NMEA completos para diagnóstico
    sendCommand("$PCAS14,1,1,1,1,1,1,0,0,0*22");

    // Modo estático adaptativo
    sendCommand("$PCAS12,2*29");

    gpsConfigured = true;
    Serial.println("✅ Configuración GPS completada");
    start_time = millis();
    lastGoodSignal = millis();
}

// Función para verificar la calidad de señal
bool checkSignalQuality() {
    if (!gps.satellites.isValid() || !gps.hdop.isValid()) {
        return false;
    }
    
    int sats = gps.satellites.value();
    float hdop = gps.hdop.hdop();
    
    // Actualizar estadísticas
    maxSatellites = max(maxSatellites, sats);
    bestHDOP = min(bestHDOP, hdop);
    
    if (sats >= MIN_SATELLITES && hdop <= MAX_ACCEPTABLE_HDOP) {
        lastGoodSignal = millis();
        return true;
    }
    return false;
}

// Función para reiniciar el GPS si es necesario
void checkGPSReset() {
    if (millis() - lastGoodSignal > SIGNAL_TIMEOUT && gpsConfigured) {
        Serial.println("\n⚠️ Señal GPS pobre durante mucho tiempo, reconfigurando...");
        gpsConfigured = false;
        configureGPS();
    }
}

void setup() {
    Serial.begin(115200);
    configureGPS();
}

void loop() {
    // Procesar datos del GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        // Usar función debug para mostrar datos NMEA solo si está activado
        debugNmeaPrint(c);
        gps.encode(c);
    }

    // Verificar calidad de señal y reiniciar si es necesario
    checkGPSReset();

    // Mostrar estadísticas cada segundo
    if (millis() - lastUpdate >= updateInterval) {
        lastUpdate = millis();

        Serial.println("\n===== 🛰️ DIAGNÓSTICO GPS =====");

        // Estadísticas de señal
        Serial.println("\n📊 Calidad de señal:");
        if (gps.satellites.isValid()) {
            int sats = gps.satellites.value();
            Serial.print("- Satélites actuales: ");
            Serial.print(sats);
            Serial.print(" (Máximo: ");
            Serial.print(maxSatellites);
            Serial.println(")");
            
            if (gps.hdop.isValid()) {
                float hdop = gps.hdop.hdop();
                Serial.print("- HDOP actual: ");
                Serial.print(hdop);
                Serial.print(" (Mejor: ");
                Serial.print(bestHDOP);
                Serial.println(")");
                
                Serial.print("- Calidad de señal: ");
                if (hdop < 1.5) {
                    Serial.println("Excelente 🟢");
                } else if (hdop < 2.5) {
                    Serial.println("Buena 🟡");
                } else if (hdop < 5.0) {
                    Serial.println("Regular 🟠");
                } else {
                    Serial.println("Pobre 🔴");
                }
            }
        }

        // Estado del sistema
        Serial.println("\n⚙️ Estado del sistema:");
        unsigned long current_time = (millis() - start_time) / 1000;
        Serial.print("- Tiempo desde inicio: ");
        Serial.print(current_time);
        Serial.println(" segundos");
        
        Serial.print("- Último fix válido hace: ");
        Serial.print((millis() - lastGoodSignal) / 1000);
        Serial.println(" segundos");

        // Datos de posición si están disponibles
        if (gps.location.isValid()) {
            Serial.println("\n✅ POSICIÓN FIJADA:");
            Serial.print("🌍 Latitud: ");
            Serial.print(gps.location.lat(), 6);
            Serial.print(" | 🌍 Longitud: ");
            Serial.println(gps.location.lng(), 6);

            Serial.print("📏 Altitud GPS: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" m");

            Serial.print("🚗 Velocidad: ");
            Serial.print(gps.speed.kmph());
            Serial.println(" km/h");

            Serial.print("🧭 Dirección: ");
            Serial.print(gps.course.deg());
            Serial.println("°");
            
            float CEP = gps.hdop.hdop() * 3.04;
            Serial.print("📍 Precisión estimada: ");
            Serial.print(CEP);
            Serial.println(" metros");

            int confidence = gps.satellites.value() * 10 - gps.hdop.hdop() * 5;
            confidence = max(0, min(100, confidence));
            Serial.print("🔹 Índice de confianza en la posición: ");
            Serial.print(confidence);
            Serial.println("/100");

            Serial.print("⏱️ Edad de los datos: ");
            Serial.print(gps.location.age() / 1000.0, 1);
            Serial.println(" segundos");

            Serial.print("🎯 Tipo de fix: ");
            if (gps.altitude.isValid() && confidence > 60) {
                Serial.println("3D Fix ⭐");
            } else if (gps.location.isValid()) {
                Serial.println("2D Fix ✨");
            } else {
                Serial.println("Sin fix ❌");
            }

            if (gps.date.isValid() && gps.time.isValid()) {
                Serial.print("📅 Fecha/Hora GPS: ");
                Serial.printf("%02d/%02d/%d %02d:%02d:%02d UTC\n",
                    gps.date.day(), gps.date.month(), gps.date.year(),
                    gps.time.hour(), gps.time.minute(), gps.time.second());
            }
            
            // Aceleración calculada
            if (gps.speed.isValid() && lastSpeedTime > 0) {
                float currentSpeed = gps.speed.kmph();
                unsigned long currentTime = millis();
                float acceleration = (currentSpeed - lastSpeed) / ((currentTime - lastSpeedTime) / 1000.0);
                Serial.print("⚡ Aceleración: ");
                Serial.print(acceleration);
                Serial.println(" m/s2");
                lastSpeed = currentSpeed;
                lastSpeedTime = currentTime;
            }

            // Estadísticas de recepción
            Serial.println("\n📊 Diagnóstico de señal para Albacete:");
            Serial.print("- Mensajes procesados: ✅ ");
            Serial.print(gps.passedChecksum());
            Serial.print(" | ❌ ");
            Serial.print(gps.failedChecksum());
            Serial.print(" (");
            float successRate = 100.0 * gps.passedChecksum() / (gps.passedChecksum() + gps.failedChecksum());
            Serial.print(successRate, 1);
            Serial.println("% éxito)");
    
            // Análisis de señal regional
            if (gps.satellites.isValid() && gps.hdop.isValid()) {
                float hdop = gps.hdop.hdop();
                int sats = gps.satellites.value();
                
                Serial.println("\n📡 Calidad de señal regional:");
                Serial.print("- Cobertura EGNOS: ");
                if (hdop < 2.0) {
                    Serial.println("Excelente ⭐⭐⭐");
                } else if (hdop < 5.0) {
                    Serial.println("Buena ⭐⭐");
                } else {
                    Serial.println("Limitada ⭐");
                }
    
                Serial.print("- Sensibilidad: ");
                if (sats >= 8) {
                    Serial.println("Óptima 📶📶📶");
                } else if (sats >= 5) {
                    Serial.println("Buena 📶📶");
                } else {
                    Serial.println("Baja 📶");
                }
    
                float altitudeError = gps.hdop.hdop() * 2.5; // Factor típico para Albacete
                Serial.print("- Error estimado altitud: ±");
                Serial.print(altitudeError, 1);
                Serial.println(" metros");
            }
        }
        Serial.println("=============================");
    }
    
    delay(100);
}

