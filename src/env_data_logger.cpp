#include "env_data_logger.h"
#include "sd_module.h"

// Se cambia el nombre de archivo para cumplir con 8.3 y se incluye la barra inicial.
static const char* FILE_PATH = "/ENVDATA.CSV";

void logEnvData(const String &gpsTime, unsigned long elapsedSec, float bmpTemp, float bmpPressure, float ahtTemp, float ahtHumidity) {
    // Si el fichero no existe, crearlo y añadir la cabecera
    if (!SD.exists(FILE_PATH)) {
        if (createFileSD(FILE_PATH)) {
            // Añadir la cabecera con los títulos de las variables
            appendDataCSV(FILE_PATH, "GPS_Hora,Tiempo_Transcurrido_seg,BMP_Temperatura_C,Presion_hPa,AHT_Temperatura_C,Humedad_%");
        } else {
            Serial.println("❌ Error creando el fichero de datos ambientales");
            return;
        }
    }
    // Formatear la línea CSV
    String csvLine = gpsTime + "," + String(elapsedSec) + "," +
                      String(bmpTemp, 2) + "," +
                      String(bmpPressure, 2) + "," +
                      String(ahtTemp, 2) + "," +
                      String(ahtHumidity, 2);
    // Añadir la línea de datos al fichero en formato CSV
    if (!appendDataCSV(FILE_PATH, csvLine.c_str())) {
        Serial.println("❌ Error al escribir datos ambientales en la SD");
    } else {
        Serial.println("✅ Datos ambientales guardados en CSV");
    }
}
