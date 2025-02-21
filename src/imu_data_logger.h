#ifndef IMU_DATA_LOGGER_H
#define IMU_DATA_LOGGER_H

#include <Arduino.h>

// Registra magnitudes del BNO055 en formato CSV.
// Parámetros: timestamp, aceleración total (m/s²), velocidad angular (rad/s), ángulo de inclinación (°) y temperatura (°C).
void logImuData(unsigned long timestamp, float totalAccel, float angularVel, float tiltAngle, int temperature);

#endif // IMU_DATA_LOGGER_H
