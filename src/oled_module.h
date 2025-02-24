#ifndef OLED_MODULE_H
#define OLED_MODULE_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // Ancho, en píxeles
#define SCREEN_HEIGHT 32 // Alto, en píxeles
#define OLED_RESET    -1 // Pin de reset (no usado)
#define OLED_ADDRESS  0x3C // Dirección I2C típica

// Páginas de visualización
enum DisplayPage {
    PAGE_IMU,      // Datos IMU (aceleración, orientación)
    PAGE_GPS,      // Datos GPS (posición, velocidad)
    PAGE_ENV,      // Datos ambientales (temp, presión, humedad)
    PAGE_STATUS,   // Estado del sistema (SD, sensores)
    PAGE_COUNT     // Número total de páginas
};

extern Adafruit_SSD1306 display;

// Funciones básicas
void initOled();
void clearDisplay();
void updateDisplay();

// Funciones de visualización por página
void showIMUPage(float accel, float gyro, float tilt);
void showGPSPage(float lat, float lon, float speed, int sats);
void showEnvPage(float temp, float pressure, float humidity);
void showStatusPage(bool sdOk, bool imuOk, bool gpsOk, bool envOk);

// Funciones de utilidad
void drawProgressBar(int x, int y, int width, int height, int progress);
void drawBattery(int x, int y, int percentage);
void drawSignalStrength(int x, int y, int strength);
void drawHeader(const char* title);
void drawFooter(const char* status);

// Control de páginas
void nextPage();
void previousPage();
DisplayPage getCurrentPage();

// Estructura para datos del display
struct DisplayData {
    // Datos IMU
    float accel_total;
    float gyro_total;
    float tilt_angle;
    
    // Datos GPS
    float latitude;
    float longitude;
    float speed;
    int satellites;
    
    // Datos ambientales
    float temperature;
    float pressure;
    float humidity;
    
    // Estado del sistema
    bool sd_ok;
    bool imu_ok;
    bool gps_ok;
    bool env_ok;
    
    // Batería y señal
    int battery_level;
    int signal_strength;
};

// Función para actualizar todos los datos
void updateDisplayData(const DisplayData& data);

#endif // OLED_MODULE_H
