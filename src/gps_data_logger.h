#ifndef GPS_DATA_LOGGER_H
#define GPS_DATA_LOGGER_H

#include <Arduino.h>

// Registra datos calculados del GPS en formato CSV.
// Parámetros: tiempo (ms), latitud, longitud, velocidad (km/h), aceleración (m/s²) y distancia (m)
void logGPSData(unsigned long timestamp, double latitude, double longitude, float speed, float acceleration, double distance);

#endif // GPS_DATA_LOGGER_H
