#include "oled_module.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void initOled() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("OLED: Fallo al inicializar el display");
        for(;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("OLED inicializado");
    display.display();
    Serial.println("OLED inicializado correctamente.");
}

void displayStatus(const String &msg) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(msg);
    display.display();
}
