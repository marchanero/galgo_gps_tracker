#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== CONFIGURACIÓN I2C =====
#define SDA_PIN 26  // Pin SDA definido
#define SCL_PIN 22  // Pin SCL definido
TwoWire I2C = TwoWire(1); // Se utiliza el segundo canal I2C

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
unsigned long lastI2Cscan = 0;  // Para programar el escaneo I2C

// ===== CONFIGURACIÓN OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

void configureGPS() {
    Serial.println("\n🛰️ Configurando GPS para máxima recepción...");
    gpsSerial.setRxBufferSize(1024);
    gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
    delay(100);
    sendCommand("$PCAS06,1,1*22");
    sendCommand("$PCAS07,1*21");
    sendCommand("$PCAS08,1*20");
    sendCommand("$PCAS01,39.0,-1.8,686*XX"); // Actualizar checksum si es necesario
    sendCommand("$PCAS09,4*2E");
    sendCommand("$PCAS16,2,2,2,2*27");
    sendCommand("$PCAS10,3,3,3,3,2,2,2,2,1,1,2,2,1,1,0,2,2,1,1,0,0,0,0*1E");
    sendCommand("$PCAS11,0,1,2,3*18");
    sendCommand("$PCAS13,2,1,1*2A");
    sendCommand("$PCAS18,1,1*2D");
    sendCommand("$PCAS04,10*1D");
    sendCommand("$PCAS05,1*18");
    sendCommand("$PCAS19,0*2F");
    sendCommand("$PCAS20,1*2E");
    sendCommand("$PCAS14,1,1,1,1,1,1,0,0,0*22");
    sendCommand("$PCAS12,2*29");
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

// ===== FUNCION PARA ESCANEAR DISPOSITIVOS I2C =====
void scanI2CDevices() {
    uint8_t count = 0;
    Serial.println("\nBuscando dispositivos I2C...");
    for (uint8_t address = 1; address < 127; address++) {
        I2C.beginTransmission(address);
        uint8_t error = I2C.endTransmission();
        if (error == 0) {
            Serial.print("Dispositivo encontrado en la dirección 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            count++;
        } else if (error == 4) {
            Serial.print("Error desconocido en la dirección 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (count == 0)
        Serial.println("No se encontraron dispositivos I2C.");
    else {
        Serial.print("Total de dispositivos encontrados: ");
        Serial.println(count);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial); // Para esperar conexión del monitor serie si corresponde
    // Inicialización I2C
    I2C.begin(SDA_PIN, SCL_PIN);
    // Inicialización del display OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Error al inicializar el OLED");
        while (true);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Iniciando...");
    display.display();
    // Configuración GPS
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
                if (hdop < 1.5) Serial.println("Excelente 🟢");
                else if (hdop < 2.5) Serial.println("Buena 🟡");
                else if (hdop < 5.0) Serial.println("Regular 🟠");
                else Serial.println("Pobre 🔴");
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
            if (gps.altitude.isValid() && confidence > 60)
                Serial.println("3D Fix ⭐");
            else if (gps.location.isValid())
                Serial.println("2D Fix ✨");
            else
                Serial.println("Sin fix ❌");
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
            float successRate = 100.0 * gps.passedChecksum() / (gps.passedChecksum() + gps.failedChecksum());
            Serial.print(" (");
            Serial.print(successRate, 1);
            Serial.println("% éxito)");
            // Análisis de señal regional
            if (gps.satellites.isValid() && gps.hdop.isValid()) {
                float hdop = gps.hdop.hdop();
                int sats = gps.satellites.value();
                Serial.println("\n📡 Calidad de señal regional:");
                Serial.print("- Cobertura EGNOS: ");
                if (hdop < 2.0) Serial.println("Excelente ⭐⭐⭐");
                else if (hdop < 5.0) Serial.println("Buena ⭐⭐");
                else Serial.println("Limitada ⭐");
                Serial.print("- Sensibilidad: ");
                if (sats >= 8) Serial.println("Óptima 📶📶📶");
                else if (sats >= 5) Serial.println("Buena 📶📶");
                else Serial.println("Baja 📶");
                float altitudeError = gps.hdop.hdop() * 2.5;
                Serial.print("- Error estimado altitud: ±");
                Serial.print(altitudeError, 1);
                Serial.println(" metros");
            }
        }
        Serial.println("=============================");
    }
    
    // Actualizar pantalla OLED con datos GPS
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    if(gps.location.isValid()){
        display.print("Lat: ");
        display.println(gps.location.lat(), 6);
        display.print("Lng: ");
        display.println(gps.location.lng(), 6);
        display.print("Fix: Fijado");
    } else {
        display.println("Sin fix");
    }
    display.display();
    
    // Escanear dispositivos I2C cada 5000 ms
    if(millis() - lastI2Cscan >= 5000) {
        scanI2CDevices();
        lastI2Cscan = millis();
    }
    
    delay(100);
}
