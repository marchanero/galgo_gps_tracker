#include "oled_module.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static DisplayPage currentPage = PAGE_IMU;
static DisplayData currentData = {0};

void initOled() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        return;
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
}

void clearDisplay() {
    display.clearDisplay();
    display.display();
}

void drawHeader(const char* title) {
    display.drawLine(0, 8, SCREEN_WIDTH, 8, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(2, 0);
    display.print(title);
    
    // Dibujar batería y señal en la esquina superior derecha
    drawBattery(SCREEN_WIDTH - 20, 0, currentData.battery_level);
    drawSignalStrength(SCREEN_WIDTH - 35, 0, currentData.signal_strength);
}

void drawFooter(const char* status) {
    display.drawLine(0, SCREEN_HEIGHT-9, SCREEN_WIDTH, SCREEN_HEIGHT-9, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(2, SCREEN_HEIGHT-8);
    display.print(status);
}

void drawProgressBar(int x, int y, int width, int height, int progress) {
    display.drawRect(x, y, width, height, SSD1306_WHITE);
    int fillWidth = (width - 2) * progress / 100;
    display.fillRect(x + 1, y + 1, fillWidth, height - 2, SSD1306_WHITE);
}

void drawBattery(int x, int y, int percentage) {
    display.drawRect(x, y, 10, 6, SSD1306_WHITE);
    display.drawRect(x + 10, y + 1, 2, 4, SSD1306_WHITE);
    display.fillRect(x + 1, y + 1, 8 * percentage / 100, 4, SSD1306_WHITE);
}

void drawSignalStrength(int x, int y, int strength) {
    for(int i = 0; i < 4; i++) {
        if(i < strength) {
            display.fillRect(x + (i*3), y + (6-i*2), 2, i*2+2, SSD1306_WHITE);
        } else {
            display.drawRect(x + (i*3), y + (6-i*2), 2, i*2+2, SSD1306_WHITE);
        }
    }
}

void showIMUPage(float accel, float gyro, float tilt) {
    display.clearDisplay();
    drawHeader("IMU Data");
    
    display.setCursor(2, 10);
    display.print("Acc: ");
    display.print(accel, 1);
    display.print(" m/s2");
    
    display.setCursor(2, 19);
    display.print("Gyr: ");
    display.print(gyro, 1);
    display.print(" rad/s");
    
    // Barra de inclinación
    int tiltBar = map(constrain(tilt, -90, 90), -90, 90, 0, 100);
    drawProgressBar(64, 12, 60, 8, tiltBar);
    
    char footer[32];
    snprintf(footer, sizeof(footer), "Tilt: %.1f deg", tilt);
    drawFooter(footer);
    display.display();
}

void showGPSPage(float lat, float lon, float speed, int sats) {
    display.clearDisplay();
    drawHeader("GPS");
    
    display.setCursor(2, 10);
    display.print(lat, 6);
    display.print(", ");
    display.print(lon, 6);
    
    display.setCursor(2, 19);
    display.print(speed, 1);
    display.print(" km/h  Sats:");
    display.print(sats);
    
    // Barra de precisión basada en satélites
    int satBar = map(constrain(sats, 0, 12), 0, 12, 0, 100);
    drawProgressBar(64, 12, 60, 8, satBar);
    
    drawFooter(sats >= 4 ? "Fix 3D" : "No Fix");
    display.display();
}

void showEnvPage(float temp, float pressure, float humidity) {
    display.clearDisplay();
    drawHeader("Environment");
    
    display.setCursor(2, 10);
    display.print(temp, 1);
    display.print("C ");
    display.print(pressure, 0);
    display.print("hPa");
    
    display.setCursor(2, 19);
    display.print("Hum: ");
    display.print(humidity, 1);
    display.print("%");
    
    // Barra de humedad
    drawProgressBar(64, 12, 60, 8, humidity);
    
    char footer[32];
    snprintf(footer, sizeof(footer), "Alt: %.0fm", pressure/100*8.43);
    drawFooter(footer);
    display.display();
}

void showStatusPage(bool sdOk, bool imuOk, bool gpsOk, bool envOk) {
    display.clearDisplay();
    drawHeader("System Status");
    
    display.setCursor(2, 10);
    display.print("SD:");
    display.print(sdOk ? "OK " : "ERR");
    display.print(" IMU:");
    display.print(imuOk ? "OK" : "ERR");
    
    display.setCursor(2, 19);
    display.print("GPS:");
    display.print(gpsOk ? "OK " : "ERR");
    display.print(" ENV:");
    display.print(envOk ? "OK" : "ERR");
    
    String status = String("Err:") + String(!sdOk + !imuOk + !gpsOk + !envOk);
    drawFooter(status.c_str());
    display.display();
}

void nextPage() {
    currentPage = static_cast<DisplayPage>((currentPage + 1) % PAGE_COUNT);
    updateDisplay();
}

void previousPage() {
    currentPage = static_cast<DisplayPage>((currentPage - 1 + PAGE_COUNT) % PAGE_COUNT);
    updateDisplay();
}

DisplayPage getCurrentPage() {
    return currentPage;
}

void updateDisplayData(const DisplayData& data) {
    currentData = data;
    updateDisplay();
}

void updateDisplay() {
    switch(currentPage) {
        case PAGE_IMU:
            showIMUPage(currentData.accel_total, 
                       currentData.gyro_total, 
                       currentData.tilt_angle);
            break;
            
        case PAGE_GPS:
            showGPSPage(currentData.latitude, 
                       currentData.longitude, 
                       currentData.speed, 
                       currentData.satellites);
            break;
            
        case PAGE_ENV:
            showEnvPage(currentData.temperature, 
                       currentData.pressure, 
                       currentData.humidity);
            break;
            
        case PAGE_STATUS:
            showStatusPage(currentData.sd_ok, 
                         currentData.imu_ok, 
                         currentData.gps_ok, 
                         currentData.env_ok);
            break;
            
        default:
            break;
    }
}
