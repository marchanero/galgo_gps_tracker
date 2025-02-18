#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

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
#define SCREEN_HEIGHT 32       // Actualizado para pantalla de 128x32
#define OLED_RESET    -1
// Usamos el bus I2C configurado
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C, OLED_RESET);

// Nuevo icono GPS (16x16) almacenado en PROGMEM
const unsigned char PROGMEM gpsIcon[] = {
    0x00, 0x00, 0x07, 0xE0, 0x18, 0x18, 0x20, 0x04,
    0x40, 0x02, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
    0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01,
    0x40, 0x02, 0x20, 0x04, 0x18, 0x18, 0x07, 0xE0, 0x00, 0x00
};

// Objetos para los sensores ambientales (BMP280 y AHT20)
Adafruit_BMP280 bmp;  // BMP280
Adafruit_AHTX0 aht;   // AHT20
// Instanciar el sensor BNO055 (usando la dirección 0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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
    // Inicialización I2C con frecuencia más baja y timeout
    I2C.begin(SDA_PIN, SCL_PIN, 100000); // 100kHz
    I2C.setTimeOut(1000); // 1 segundo de timeout
    // Inicialización del display OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Error al inicializar el OLED");
        while (true);
    }
    display.clearDisplay();
    
    delay(100); // Esperar después de la inicialización I2C
    
    // Inicializar BMP280 con reintentos
    bool bmpInitialized = false;
    for(int i = 0; i < 3 && !bmpInitialized; i++) {
        if (bmp.begin(0x77, BMP280_CHIPID)) {
            bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     // Modo forzado
                           Adafruit_BMP280::SAMPLING_X2,       // Temperatura
                           Adafruit_BMP280::SAMPLING_X16,      // Presión
                           Adafruit_BMP280::FILTER_X16,        // Filtrado
                           Adafruit_BMP280::STANDBY_MS_500);   // Tiempo de espera
            Serial.println("BMP280 inicializado correctamente");
            bmpInitialized = true;
        } else {
            Serial.print("Intento "); Serial.print(i + 1);
            Serial.println(" de inicialización BMP280 fallido");
            delay(1000);
        }
    }
    
    delay(100); // Esperar entre inicializaciones
    
    // Inicializar AHT20 con dirección específica
    if (!aht.begin(&I2C, 0x38)) {
        Serial.println("Error: AHT20 no encontrado!");
    } else {
        Serial.println("AHT20 inicializado correctamente");
    }

    delay(100); // Esperar entre inicializaciones

    // Inicializar BNO055 con reintentos y reset
    bool bnoInitialized = false;
    for(int i = 0; i < 3 && !bnoInitialized; i++) {
        if (bno.begin()) {
            delay(100);
            // Configurar modo de operación
            bno.setMode((adafruit_bno055_opmode_t)0x0C); // NDOF mode
            delay(50);
            // Activar cristal externo
            bno.setExtCrystalUse(true);
            delay(50);
            // Verificar que el sensor responde
            uint8_t system_status, self_test_result, system_error;
            bno.getSystemStatus(&system_status, &self_test_result, &system_error);
            if (system_status == 5) { // 5 = sistema ejecutándose
                Serial.println("BNO055 inicializado correctamente");
                bnoInitialized = true;
            }
        }
        if (!bnoInitialized) {
            Serial.print("Intento "); Serial.print(i + 1);
            Serial.println(" de inicialización BNO055 fallido");
            delay(1000);
        }
    }
    
    // Mostrar pantalla de bienvenida con mayor tamaño dividido en dos líneas
    display.setTextSize(2); // Tamaño mayor
    display.setTextColor(SSD1306_WHITE);
    // Calcular centrado: "galgo" (5 letras * 12 px = 60px) y "Sport" (5 letras * 12 px = 60px)
    display.setCursor((SCREEN_WIDTH - 60) / 2, 0);
    display.println("galgo");
    display.setCursor((SCREEN_WIDTH - 60) / 2, 16); // segunda línea (altura 16)
    display.println("Sport");
    display.display();
    delay(2000); // Mostrar bienvenida 2 segundos
    
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
    
    // Variables estáticas para almacenar las últimas lecturas válidas
    static struct {
        float temperature;
        float pressure;
        unsigned long lastRead;
        bool isValid;
    } bmpData = {0, 0, 0, false};

    static struct {
        float humidity;
        float temperature;
        unsigned long lastRead;
        bool isValid;
    } ahtData = {0, 0, 0, false};

    static struct {
        float heading;
        float roll;
        float pitch;
        uint8_t calibration;
        unsigned long lastRead;
        bool isValid;
    } bnoData = {0, 0, 0, 0, 0, false};

    // Intervalos de lectura optimizados para cada sensor
    const unsigned long BMP_INTERVAL = 2000;  // 2 segundos para BMP280
    const unsigned long AHT_INTERVAL = 1000;  // 1 segundo para AHT20
    const unsigned long BNO_INTERVAL = 100;   // 100ms para BNO055
    
    unsigned long currentMillis = millis();

    // Leer BMP280 cada 2 segundos
    if (currentMillis - bmpData.lastRead >= BMP_INTERVAL) {
        bmpData.isValid = false;
        if (bmp.takeForcedMeasurement()) {
            float temp = bmp.readTemperature();
            float pres = bmp.readPressure() / 100.0F;
            if (!isnan(temp) && !isnan(pres)) {
                bmpData.temperature = temp;
                bmpData.pressure = pres;
                bmpData.isValid = true;
                bmpData.lastRead = currentMillis;
                Serial.printf("BMP280: %.1f°C, %.1f hPa\n", temp, pres);
            }
        }
        if (!bmpData.isValid) {
            Serial.println("Error BMP280");
        }
    }

    // Leer AHT20 cada segundo
    if (currentMillis - ahtData.lastRead >= AHT_INTERVAL) {
        ahtData.isValid = false;
        sensors_event_t humidity, temp;
        if (aht.getEvent(&humidity, &temp)) {
            if (!isnan(humidity.relative_humidity) && !isnan(temp.temperature)) {
                ahtData.humidity = humidity.relative_humidity;
                ahtData.temperature = temp.temperature;
                ahtData.isValid = true;
                ahtData.lastRead = currentMillis;
                Serial.printf("AHT20: %.1f%%, %.1f°C\n",
                    humidity.relative_humidity, temp.temperature);
            }
        }
        if (!ahtData.isValid) {
            Serial.println("Error AHT20");
        }
    }

    // Leer BNO055 cada 100ms
    if (currentMillis - bnoData.lastRead >= BNO_INTERVAL) {
        bnoData.isValid = false;
        sensors_event_t event;
        uint8_t system, gyro, accel, mag;
        
        if (bno.getEvent(&event) && bno.getCalibration(&system, &gyro, &accel, &mag)) {
            if (system >= 1) {  // Sistema mínimamente calibrado
                bnoData.heading = event.orientation.x;
                bnoData.roll = event.orientation.y;
                bnoData.pitch = event.orientation.z;
                bnoData.calibration = system;
                bnoData.isValid = true;
                bnoData.lastRead = currentMillis;
                
                // Reducir mensajes Serial a solo cuando hay cambios significativos
                static float lastHeading = 0;
                if (abs(bnoData.heading - lastHeading) > 1.0) {
                    Serial.printf("BNO055: H=%.1f° R=%.1f° P=%.1f° Cal=%d\n",
                        bnoData.heading, bnoData.roll, bnoData.pitch, system);
                    lastHeading = bnoData.heading;
                }
            }
        }
        if (!bnoData.isValid) {
            Serial.println("Error BNO055");
        }
    }
    
    // Extraer todas las variables del BNO055 usando imu::Vector<3>
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyroVec   = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> magVec    = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    // Obtener estados de calibración
    uint8_t sysCal, gyroCal, accelCal, magCal;
    bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
    
    // Publicar los valores en el puerto Serial
    Serial.println("----- BNO055 Variables -----");
    Serial.print("Euler -> Heading: "); Serial.print(heading, 1);
    Serial.print(" Roll: "); Serial.print(roll, 1);
    Serial.print(" Pitch: "); Serial.println(pitch, 1);
    
    Serial.print("Aceleración lineal -> X: "); Serial.print(linAccel.x(), 1);
    Serial.print(" Y: "); Serial.print(linAccel.y(), 1);
    Serial.print(" Z: "); Serial.println(linAccel.z(), 1);
    
    Serial.print("Giroscopio -> X: "); Serial.print(gyroVec.x(), 1);
    Serial.print(" Y: "); Serial.print(gyroVec.y(), 1);
    Serial.print(" Z: "); Serial.println(gyroVec.z(), 1);
    
    Serial.print("Magnetómetro -> X: "); Serial.print(magVec.x(), 1);
    Serial.print(" Y: "); Serial.print(magVec.y(), 1);
    Serial.print(" Z: "); Serial.println(magVec.z(), 1);
    
    Serial.print("Calibración -> System: "); Serial.print(sysCal);
    Serial.print(" Gyro: "); Serial.print(gyroCal);
    Serial.print(" Accel: "); Serial.print(accelCal);
    Serial.print(" Mag: "); Serial.println(magCal);
    Serial.println("-----------------------------");
    
    // Actualizar pantalla OLED con datos GPS y sensores ambientales
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
        // Líneas sensores ambientales (4 líneas en total en pantalla 128x32)
        display.print("T: ");
        display.print(bmpTemp, 1);
        display.print("C ");
        display.print("H: ");
        display.print(ahtHum, 1);
        display.println("%");
        display.print("P: ");
        display.print(pressure, 1);
        display.println("hPa");
        // Datos BNO055 (por ejemplo, heading)
        display.print("Hd: ");
        display.print(heading, 1);
        display.println("°");
        // Mostrar icono GPS en esquina superior derecha
        display.drawBitmap(SCREEN_WIDTH - 16, 0, gpsIcon, 16, 16, SSD1306_WHITE);
    } else {
        display.println("Sin fix");
        // También se pueden mostrar lecturas ambientales aunque no haya fix
        display.print("T: ");
        display.print(bmpTemp, 1);
        display.print("C H: ");
        display.print(ahtHum, 1);
        display.println("%");
        display.print("P: ");
        display.print(pressure, 1);
        display.println("hPa");
        display.print("Hd: ");
        display.print(heading, 1);
        display.println("°");
    }
    display.display();
    
    // Escanear dispositivos I2C cada 5000 ms
    if(millis() - lastI2Cscan >= 5000) {
        scanI2CDevices();
        lastI2Cscan = millis();
    }
    
    delay(100);
}
