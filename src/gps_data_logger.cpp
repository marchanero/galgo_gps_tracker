#include "gps_data_logger.h"
#include "sd_module.h"

// Se utiliza un nombre de archivo compatible en formato 8.3 con barra inicial
static const char* FILE_PATH = "/GPSDATA.CSV";

void logGPSData(unsigned long timestamp, double latitude, double longitude, float speed, float acceleration, double distance) {
    String csvLine = String(timestamp) + "," + 
                     String(latitude, 6) + "," +
                     String(longitude, 6) + "," +
                     String(speed, 2) + "," +
                     String(acceleration, 2) + "," +
                     String(distance, 2);
    if (!appendDataCSV(FILE_PATH, csvLine.c_str())) {
        Serial.println("❌ Error escribiendo en " + String(FILE_PATH));
    } else {
        Serial.println("✅ Datos GPS guardados en CSV");
    }
}
