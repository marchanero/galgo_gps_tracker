#include "oled_module.h"
#include <Wire.h>
#include <math.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static DisplayPage currentPage = PAGE_IMU;
static DisplayData lastData;
static bool initScreenComplete = false;

void drawGPSLogo(int x, int y, int size) {
    int radius = size / 2;
    // Dibujar círculo exterior
    display.drawCircle(x + radius, y + radius, radius, WHITE);
    // Dibujar cruz interior
    display.drawLine(x + radius, y, x + radius, y + size, WHITE);
    display.drawLine(x, y + radius, x + size, y + radius, WHITE);
    // Dibujar punto central
    display.fillCircle(x + radius, y + radius, 2, WHITE);
}

void drawWaveAnimation(int x, int y, int width, int height, uint8_t frame) {
    for (int i = 0; i < width; i++) {
        float angle = (i + frame) * 0.2;
        int amplitude = height / 2;
        int yPos = y + amplitude + (sin(angle) * amplitude);
        display.drawPixel(x + i, yPos, WHITE);
    }
}

void showSplashLogo() {
    // Efecto de fade-in para el texto
    for(int i = 0; i < 16; i++) {
        display.clearDisplay();
        
        // Dibujar título con efecto de deslizamiento
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(20 - (16-i), 0);
        display.println("GPS");
        display.setCursor(20 + (16-i), 16);
        display.println("TRACK");
        
        // Dibujar logo GPS creciendo desde el centro
        int logoSize = i + 1;
        drawGPSLogo(85 - logoSize/2, 8 - logoSize/2, logoSize * 2);
        
        display.display();
        delay(50);
    }
    
    // Parpadeo suave del logo
    for(int i = 0; i < 3; i++) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(20, 0);
        display.println("GPS");
        display.setCursor(20, 16);
        display.println("TRACK");
        display.display();
        delay(100);
        
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(20, 0);
        display.println("GPS");
        display.setCursor(20, 16);
        display.println("TRACK");
        drawGPSLogo(85, 8, 16);
        display.display();
        delay(200);
    }
    
    delay(500);
}

void showInitProgress(const char* status, int progress) {
    static uint8_t animFrame = 0;
    static int lastProgress = 0;
    static const int progressBarY = 24;
    
    // Suavizar la transición del progreso
    while (lastProgress < progress) {
        display.clearDisplay();
        
        // Título con efecto pulsante
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("GPS TRACKER v1.0");
        
        // Línea separadora animada
        for(int i = 0; i < SCREEN_WIDTH; i += 4) {
            if((i + animFrame) % 8 < 4) {
                display.drawPixel(i, 9, WHITE);
            }
        }
        
        // Logo GPS con rotación
        float angle = (animFrame * 10) * PI / 180.0;
        drawGPSLogo(2, 12, 8);
        display.drawLine(6, 16,
                        6 + (int)(cos(angle) * 4),
                        16 + (int)(sin(angle) * 4),
                        WHITE);
        
        // Mensaje de estado
        display.setTextSize(1);
        display.setCursor(16, 12);
        display.println(status);
        
        // Barra de progreso con doble animación
        display.drawRect(0, progressBarY, SCREEN_WIDTH, 8, WHITE);
        
        // Relleno principal
        int fillWidth = (lastProgress * (SCREEN_WIDTH-4)) / 100;
        display.fillRect(2, progressBarY+2, fillWidth, 4, WHITE);
        
        // Onda animada sobre el relleno
        drawWaveAnimation(2, progressBarY+2, fillWidth, 4, animFrame);
        
        display.display();
        lastProgress++;
        animFrame++;
        delay(20);
    }
    
    if (progress >= 100) {
        // Efecto de completado
        for(int i = 0; i < 3; i++) {
            display.invertDisplay(true);
            delay(50);
            display.invertDisplay(false);
            delay(50);
        }
        initScreenComplete = true;
    }
}

bool isInitScreenDone() {
    return initScreenComplete;
}

void initOled() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        return;
    }
    
    display.clearDisplay();
    display.display();
    delay(100);
    
    // Efecto de fade-in para el logo y texto
    for(int i = 0; i < 16; i++) {
        display.clearDisplay();
        
        // Dibujar título con efecto de deslizamiento
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(20 - (16-i), 0);
        display.println("GPS");
        display.setCursor(20 + (16-i), 16);
        display.println("TRACK");
        
        // Logo GPS creciendo desde el centro
        int logoSize = i + 1;
        drawGPSLogo(85 - logoSize/2, 8 - logoSize/2, logoSize * 2);
        
        display.display();
        delay(50);
    }
    
    // Mantener el logo un momento
    delay(1000);
    
    // Efecto de transición
    for(int i = 0; i < SCREEN_HEIGHT; i += 2) {
        display.clearDisplay();
        display.fillRect(0, 0, SCREEN_WIDTH, i, BLACK);
        display.display();
        delay(20);
    }
    
    initScreenComplete = false;
}

void showSplashScreen() {
    display.clearDisplay();
    
    // Título con efecto de aparición
    for(int i = 0; i < 16; i += 2) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor((SCREEN_WIDTH - 15*6)/2, 8);
        display.println("GPS TRACKER");
        
        // Línea animada
        display.drawLine(0, 20, i * 8, 20, WHITE);
        display.display();
        delay(50);
    }
    
    delay(500);
}

void drawStatusBar() {
    // Dibuja indicadores de estado en la línea superior
    display.drawRect(0, 0, 128, 8, WHITE);
    
    // Batería
    display.drawRect(2, 1, 10, 6, WHITE);
    display.drawRect(12, 3, 2, 2, WHITE);
    display.fillRect(2, 1, lastData.battery_level / 10, 6, WHITE);
    
    // Señal GPS
    for(int i = 0; i < lastData.signal_strength; i++) {
        display.fillRect(20 + (i*3), 6 - (i*2), 2, 1 + (i*2), WHITE);
    }
    
    // Estado de sensores
    display.drawCircle(40, 3, 2, WHITE);
    if(lastData.imu_ok) display.fillCircle(40, 3, 2, WHITE);
    
    display.drawCircle(48, 3, 2, WHITE);
    if(lastData.gps_ok) display.fillCircle(48, 3, 2, WHITE);
    
    display.drawCircle(56, 3, 2, WHITE);
    if(lastData.env_ok) display.fillCircle(56, 3, 2, WHITE);
    
    display.drawCircle(64, 3, 2, WHITE);
    if(lastData.sd_ok) display.fillCircle(64, 3, 2, WHITE);
    
    // Indicador de página actual (4 puntos a la derecha)
    const int dotSpacing = 4;
    const int dotRadius = 1;
    const int startX = 120;
    const int centerY = 4;
    
    for(int i = 0; i < 4; i++) {  // 4 páginas: IMU, GPS, COMPASS, ENV
        if(i == currentPage) {
            display.fillCircle(startX - (i * dotSpacing), centerY, dotRadius, WHITE);
        } else {
            display.drawCircle(startX - (i * dotSpacing), centerY, dotRadius, WHITE);
        }
    }
}

void drawTiltIndicator(float angle) {
    const int centerX = 64;
    const int centerY = 24;
    const int radius = 10;
    
    // Convertir ángulo a radianes
    float angleRad = angle * PI / 180.0;
    
    // Calcular punto final de la línea indicadora
    int endX = centerX + (int)(sin(angleRad) * radius);
    int endY = centerY - (int)(cos(angleRad) * radius);
    
    // Dibujar círculo base
    display.drawCircle(centerX, centerY, radius, WHITE);
    
    // Dibujar línea indicadora
    display.drawLine(centerX, centerY, endX, endY, WHITE);
    
    // Dibujar marcas de referencia
    display.drawPixel(centerX, centerY - radius, WHITE); // 0°
    display.drawPixel(centerX + radius, centerY, WHITE); // 90°
    display.drawPixel(centerX, centerY + radius, WHITE); // 180°
    display.drawPixel(centerX - radius, centerY, WHITE); // 270°
}

void drawAccelBar(float accel) {
    const int barWidth = 40;
    const int barHeight = 8;
    const int barX = 80;
    const int barY = 12;
    const float maxAccel = 20.0; // m/s²
    
    // Dibujar marco de la barra
    display.drawRect(barX, barY, barWidth, barHeight, WHITE);
    
    // Calcular el ancho de la barra basado en la aceleración
    int fillWidth = constrain((int)((accel / maxAccel) * barWidth), 0, barWidth);
    display.fillRect(barX, barY, fillWidth, barHeight, WHITE);
}

void displayIMUPage() {
    display.setTextSize(1);
    
    // Mostrar valores numéricos
    display.setCursor(0, 10);
    display.print("ACC:");
    display.print(lastData.accel_total, 1);
    
    display.setCursor(0, 20);
    display.print("GYR:");
    display.print(lastData.gyro_total, 1);
    
    // Dibujar indicador de inclinación
    drawTiltIndicator(lastData.tilt_angle);
    
    // Dibujar barra de aceleración
    drawAccelBar(lastData.accel_total);
}

void drawCompass(float heading) {
    const int centerX = 96;
    const int centerY = 20;
    const int radius = 8;
    
    // Convertir heading a radianes
    float headingRad = heading * PI / 180.0;
    
    // Dibujar círculo exterior
    display.drawCircle(centerX, centerY, radius, WHITE);
    
    // Dibujar puntos cardinales
    display.drawPixel(centerX, centerY - radius, WHITE); // N
    display.drawPixel(centerX + radius, centerY, WHITE); // E
    display.drawPixel(centerX, centerY + radius, WHITE); // S
    display.drawPixel(centerX - radius, centerY, WHITE); // W
    
    // Dibujar flecha de dirección
    int arrowX = centerX + (int)(sin(headingRad) * (radius - 2));
    int arrowY = centerY - (int)(cos(headingRad) * (radius - 2));
    display.drawLine(centerX, centerY, arrowX, arrowY, WHITE);
}

void displayGPSPage() {
    display.setTextSize(1);
    
    // Mostrar coordenadas en la parte superior
    display.setCursor(0, 10);
    display.print("LAT:");
    display.println(lastData.latitude, 6);
    display.print("LON:");
    display.println(lastData.longitude, 6);
    
    // Mostrar velocidad y satélites
    display.setCursor(0, 25);
    display.print("SPD:");
    display.print(lastData.speed, 1);
    display.print("km/h");
    
    // Mostrar satélites con icono
    display.setCursor(70, 25);
    display.print("SAT:");
    display.print(lastData.satellites);
}

void displayCompassPage() {
    display.setTextSize(1);
    
    // Dibujar brújula grande en el centro
    const int centerX = SCREEN_WIDTH / 2;
    const int centerY = SCREEN_HEIGHT / 2;
    const int radius = 12;
    
    // Dibujar círculo exterior
    display.drawCircle(centerX, centerY, radius, WHITE);
    
    // Dibujar puntos cardinales
    display.setCursor(centerX - 2, centerY - radius - 8);
    display.print("N");
    display.setCursor(centerX + radius + 4, centerY - 2);
    display.print("E");
    display.setCursor(centerX - 2, centerY + radius + 2);
    display.print("S");
    display.setCursor(centerX - radius - 8, centerY - 2);
    display.print("W");
    
    // Obtener el heading y convertirlo a radianes
    float heading = 0.0; // TODO: Obtener el heading real del GPS
    float headingRad = heading * PI / 180.0;
    
    // Dibujar flecha de dirección
    int arrowX = centerX + (int)(sin(headingRad) * (radius - 2));
    int arrowY = centerY - (int)(cos(headingRad) * (radius - 2));
    display.drawLine(centerX, centerY, arrowX, arrowY, WHITE);
    
    // Mostrar dirección actual
    display.setCursor(0, 0);
    display.print("Dir: ");
    display.print(getCompassDirection(heading));
    
    // Mostrar ángulo
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print(heading, 1);
    display.print("°");
}

void drawEnvBar(int x, int y, float value, float maxValue, const char* label) {
    const int barWidth = 50;
    const int barHeight = 6;
    
    // Dibujar etiqueta
    display.setCursor(x, y);
    display.print(label);
    
    // Dibujar valor
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%0.1f", value);
    int valueWidth = strlen(buffer) * 6; // 6 píxeles por carácter aprox.
    display.setCursor(x + barWidth + 5, y);
    display.print(buffer);
    
    // Dibujar barra
    display.drawRect(x, y + 2, barWidth, barHeight, WHITE);
    int fillWidth = constrain((int)((value / maxValue) * barWidth), 0, barWidth);
    display.fillRect(x, y + 2, fillWidth, barHeight, WHITE);
}

void displayEnvPage() {
    display.setTextSize(1);
    
    // Temperatura (rango típico: -10 a 50°C)
    drawEnvBar(5, 10, lastData.temperature, 50.0, "T:");
    display.print("C");
    
    // Humedad (rango: 0-100%)
    drawEnvBar(5, 20, lastData.humidity, 100.0, "H:");
    display.print("%");
    
    // Presión (rango típico: 900-1100 hPa, normalizado)
    float normPressure = lastData.pressure - 900.0; // Normalizar para mejor visualización
    drawEnvBar(5, 30, normPressure, 200.0, "P:");
    display.print("hPa");
}

void updateDisplayData(const DisplayData& data) {
    lastData = data;
    display.clearDisplay();
    
    drawStatusBar();
    
    switch(currentPage) {
        case PAGE_IMU:
            displayIMUPage();
            break;
        case PAGE_GPS:
            displayGPSPage();
            break;
        case PAGE_COMPASS:
            displayCompassPage();
            break;
        case PAGE_ENV:
            displayEnvPage();
            break;
        default:
            break;
    }
    
    display.display();
}

void nextPage() {
    // Navegar entre las páginas principales (IMU, GPS, COMPASS, ENV)
    currentPage = static_cast<DisplayPage>((currentPage + 1) % 4);
}

void previousPage() {
    // Navegar entre las páginas principales (IMU, GPS, COMPASS, ENV)
    int prevPage = (currentPage - 1);
    if (prevPage < 0) {
        prevPage = 3;  // Volver a la última página (ENV)
    }
    currentPage = static_cast<DisplayPage>(prevPage);
}

DisplayPage getCurrentPage() {
    return currentPage;
}

void clearDisplay() {
    display.clearDisplay();
    display.display();
}
