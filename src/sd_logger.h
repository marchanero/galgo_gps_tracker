#ifndef SD_LOGGER_H
#define SD_LOGGER_H

bool initSD(); // Inicializa la tarjeta SD.
void logData(const char* data); // Registra una línea de datos en el archivo de log.
void sdDiagnostic(); // Realiza una rutina de diagnóstico de la SD.

#endif // SD_LOGGER_H
