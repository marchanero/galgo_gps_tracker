# 🚀 GPS Tracker

Un sistema de seguimiento GPS integrado con sensores múltiples y funcionalidades avanzadas.

---

## 📌 Descripción del Proyecto

Este proyecto implementa un sistema de seguimiento GPS utilizando Arduino. Entre sus funcionalidades se destacan:

- **Configuración GPS:**  
  Inicializa y configura el módulo GPS para optimizar la recepción de señales mediante comandos específicos.
- **Diagnóstico y análisis:**  
  Monitorea la calidad de la señal (satélites, HDOP) y procesa datos de posicionamiento, aceleración y altitud.
- **Visualización interactiva:**  
  Muestra datos en tiempo real en el monitor serie y en una pantalla OLED.
- **Escaneo I2C:**  
  Detecta y lista los dispositivos conectados en el bus I2C.

---

## 🗂️ Estructura del Proyecto

| Componente          | Descripción                                                                 | Archivo(s)                                                           |
|---------------------|-----------------------------------------------------------------------------|----------------------------------------------------------------------|
| **Main**            | Inicializa hardware, procesa datos y gestiona la comunicación serial        | `/src/main.cpp`                                                      |
| **GPS Module**      | Configura y procesa datos del módulo GPS                                    | `/src/gps_module.h`, `/src/gps_module.cpp`                             |
| **Data Logging**    | Registra y guarda datos en SD en formato CSV                                | `/src/gps_data_logger.h`, `/src/gps_data_logger.cpp`                   |
| **Ambiental Sensor**| Lee y muestra datos de sensores BMP280 y AHT20                              | `/src/bmp280_module.h`, `/src/bmp280_module.cpp`                       |
| **Env Data Logger** | Guarda datos ambientales en CSV                                             | `/src/env_data_logger.h`, `/src/env_data_logger.cpp`                   |

---

## 🔧 Funcionalidades Avanzadas

- **📡 Configuración Automática del GPS:**  
  La función `configureGPS` ajusta la tasa de actualización y limita las sentencias NMEA para mejorar la precisión.
  
- **📊 Análisis Visual de Datos:**  
  Uso de filtros (promedio móvil y mediana) para suavizar la velocidad, y la implementación de filtros de Kalman para obtener estimaciones más precisas.

- **📝 Registro de Datos:**  
  Se usa el módulo SD para almacenar la configuración y los datos fijos del GPS, además de los registros ambientales que son guardados en formato CSV siguiendo el estándar 8.3.

- **🎛️ Interfaz de Usuario Mejorada:**  
  A través del monitor serie se despliegan mensajes informativos y diagnósticos en tiempo real, apoyados por iconos y mensajes claros.

---

## 📏 Métricas Utilizadas

El sistema registra diversas métricas para monitorear el estado y el rendimiento tanto del módulo GPS como de los sensores ambientales y de movimiento. Estas métricas se utilizan para diagnosticar el estado del sensor, validar la calidad de la señal y optimizar el procesamiento de los datos.

| Métrica                    | Descripción                                                     | Unidad       |
|----------------------------|-----------------------------------------------------------------|--------------|
| **Calibración**            | Estado de calibración del sensor (umbral superado u omitido)     | Sí/No        |
| **HDOP**                   | Precisión horizontal de la señal GPS                             | Valor numérico (menor es mejor) |
| **Satélites**              | Número de satélites conectados para la medición                  | Número       |
| **Aceleración Total**      | Magnitud de la aceleración calculada a partir de sensores         | m/s²         |
| **Velocidad Angular**      | Magnitud de la velocidad angular                                | rad/s        |
| **Inclinación**            | Ángulo de inclinación obtenido con filtros complementarios        | °            |
| **Distancia Total**        | Distancia acumulada calculada con fórmula de Haversine            | m            |
| **Velocidad Filtrada**     | Velocidad suavizada utilizando filtros (promedio móvil/mediana)     | km/h         |
| **Aceleración Suavizada**  | Variación de velocidad suavizada para eliminar saltos de medición  | m/s²         |

---

## 📊 Métricas Calculadas y Procesamiento de Señales

El sistema no solo registra métricas brutas, sino que también realiza cálculos avanzados y aplican filtros para mejorar la calidad de los datos. Se utilizan técnicas de filtrado como el filtro Complementario para inclinación, filtros de Kalman para la estimación de aceleraciones, velocidades y posiciones, y métodos de mediana y promedio móvil para suavizar la velocidad capturada.

| Procesamiento/Filtro       | Función                                                            | Descripción                                                                            |
|----------------------------|---------------------------------------------------------------------|----------------------------------------------------------------------------------------|
| **Complementary Filter**   | Estima la inclinación combinando acelerómetro y giroscopio            | Combina datos de ambos sensores para reducir errores de medición                       |
| **Filtro de Kalman**       | Suaviza mediciones de aceleración, velocidad y posición               | Reduce el ruido y optimiza la estimación mediante un modelo estadístico                   |
| **Promedio Móvil**         | Suavizado de velocidad                                              | Calcula la media de muestras recientes para eliminar fluctuaciones bruscas              |
| **Filtro de Mediana**      | Filtrado para eliminar outliers en la velocidad                      | Ordena muestras y toma el valor central para minimizar el impacto de valores atípicos      |

---

## 🛠️ Modificaciones Recientes

Se han realizado las siguientes mejoras al proyecto:

- **Estilos y Formato Mejorado:**  
  Se han agregado títulos, iconos y tablas para una mejor comprensión y visualización de la información.
- **Optimización del Código:**  
  Se refactorizó la lectura y el registro de datos del GPS, así como la integración de sensores ambientales.
- **Documentación Ampliada:**  
  Se añadió una sección de "Modificaciones" y se actualizó la tabla de componentes para reflejar la estructura actual del proyecto.

---

## ⚙️ Cómo Utilizar

1. **Conexiones:**  
   Conecta el módulo GPS, sensores I2C y la pantalla OLED conforme al esquema del hardware.
2. **Carga del Código:**  
   Sube el código a la placa Arduino.
3. **Monitoreo:**  
   Verifica la salida en el monitor serie y en la pantalla OLED para validar el diagnóstico y la configuración.
4. **Revisión de Registros:**  
   Consulta los archivos CSV en la tarjeta SD para analizar los datos registrados.

---

## ℹ️ Notas Adicionales

- Revisa y adapta la configuración de pines y parámetros (baudrate, I2C, etc.) a tu hardware específico.
- Se recomienda validar el checksum de los comandos enviados al GPS para asegurar su correcto funcionamiento.
- El proyecto usa formatos compatibles con sistemas embebidos y el almacenamiento SD siguiendo el estándar 8.3.

---

¡Disfruta del proyecto y sigue experimentando!
