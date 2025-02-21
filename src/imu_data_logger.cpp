#include "imu_data_logger.h"
#include "sd_module.h"

// Se cambia a formato 8.3 con barra inicial
static const char* FILE_PATH = "/IMUDATA.CSV";

void logImuData(unsigned long timestamp, float totalAccel, float angularVel, float tiltAngle, int temperature) {
    // Si el fichero no existe, crearlo y añadir la cabecera
    if (!SD.exists(FILE_PATH)) {
        if (createFileSD(FILE_PATH)) {
            appendDataCSV(FILE_PATH, "Timestamp,TotalAccel(m/s^2),AngularVel(rad/s),TiltAngle(deg),Temperature(C)");
        } else {
            Serial.println("❌ Error creando el fichero de datos IMU");
            return;
        }
    }
    // Formatear la línea CSV
    String csvLine = String(timestamp) + "," +
                     String(totalAccel, 2) + "," +
                     String(angularVel, 2) + "," +
                     String(tiltAngle, 2) + "," +
                     String(temperature);
    if (!appendDataCSV(FILE_PATH, csvLine.c_str())) {
        Serial.println("❌ Error al escribir datos IMU en la SD");
    } else {
        Serial.println("✅ Datos IMU guardados en CSV");
    }
}
