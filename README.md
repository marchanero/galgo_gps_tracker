# GPS Tracker

Este proyecto implementa un sistema de seguimiento GPS utilizando Arduino. El sistema configura y procesa datos desde un módulo GPS, además de escanear dispositivos I2C y mostrar los datos en una pantalla OLED.

## Funcionalidades

- **Configuración GPS:**  
  Se inicializa y configura el módulo GPS para optimizar la recepción de señales. Se envían múltiples comandos de configuración y se monitorean estadísticas de señal (satélites, HDOP, velocidad, etc.).

- **Diagnóstico y análisis de la señal:**  
  Se evalúa la calidad de la señal en tiempo real, mostrando estadísticas como número de satélites, precisión (HDOP) y datos de posicionamiento (latitud, longitud, altitud). Se calcula además la aceleración.

- **Escaneo de dispositivos I2C:**  
  Periódicamente se escanean los dispositivos conectados al bus I2C, mostrando las direcciones encontradas para facilitar la depuración.

- **Visualización en pantalla OLED:**  
  Los datos principales del GPS (latitud, longitud y estado de fix) se muestran en una pantalla OLED, actualizando la información continuamente.

## Estructura del proyecto

- **src/main.cpp:**  
  Contiene la inicialización de hardware, configuración del GPS, procesamiento de datos y actualización de la pantalla OLED. También se gestiona el escaneo del bus I2C.

- **README.md:**  
  Este documento proporciona una descripción general y funcional del proyecto.

## Cómo utilizar

1. Conecta el módulo GPS y la pantalla OLED siguiendo el esquema especificado.
2. Sube el código a la placa Arduino.
3. Monitorea la salida en el monitor serie para ver el diagnóstico y la configuración en tiempo real.
4. Verifica la información visualizada en la pantalla OLED.

## Notas adicionales

- Se recomienda revisar la configuración de pines y parámetros de comunicación (baudrate, pines SDA/SCL del I2C) para adaptarlos a la configuración de hardware utilizada.
- El código incluye comandos específicos para la configuración del GPS, los cuales pueden necesitar ajuste en caso de utilizar un módulo diferente o para actualizar checksums.

// ...existing código del proyecto...
