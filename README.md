# 🚀 GPS Tracker y Sistema de Análisis de Movimiento

## 📌 Resumen del Sistema
Sistema integrado de seguimiento GPS y análisis de movimiento con sensores múltiples (IMU, GPS, ambientales) y procesamiento avanzado de señales.

---

## 🎯 Objetivos del Sistema
1. Captura precisa de movimiento y posición
2. Análisis ambiental completo
3. Procesamiento en tiempo real de señales
4. Almacenamiento eficiente de datos
5. Organización automática de sesiones

---

## 🔧 Arquitectura del Sistema

### 1. Módulos Hardware
- **IMU (BNO055)**
  - Acelerómetro: ±2g, ±4g, ±8g, ±16g
  - Giroscopio: ±250, ±500, ±1000, ±2000 °/s
  - Magnetómetro: ±1300µT (x,y), ±2500µT (z)
  
- **GPS (NEO-6M)**
  - Precisión posición: 2.5m CEP
  - Velocidad: 0.1 m/s
  - TTFF: 27s cold start
  
- **Sensores Ambientales**
  - BMP280: -40 a +85°C, 300-1100 hPa
  - AHT20: 0-100% RH, ±2% precisión

### 2. Sistema de Almacenamiento
```
/sd/sessions/
└── YYYYMMDD_HHMMSS/
    ├── imu_data.csv
    ├── gps_data.csv
    └── env_data.csv
```

### 3. Configuración de Pines
- **SD Card**
  - CS: 5
  - SCK: 18
  - MISO: 19
  - MOSI: 23
  
- **I2C Bus**
  - SDA: 26
  - SCL: 22

---

## 📊 Procesamiento y Análisis de Datos

### 1. Datos IMU
#### Datos Brutos (imu_data.csv)
```csv
timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,quat_w,quat_x,quat_y,quat_z
```

#### Datos Procesados
- **Aceleración Total**: √(ax² + ay² + az²)
- **Velocidad Angular**: √(gx² + gy² + gz²)
- **Fuerza Magnética**: √(mx² + my² + mz²)
- **Orientación**: Quaternion → Ángulos de Euler

#### Filtros Aplicados
1. **Kalman para Aceleración**
   - Q = 0.1 (varianza proceso)
   - R = 0.1 (varianza medición)
   - Reduce ruido y suaviza señal

2. **Complementario para Inclinación**
   - α = 0.96 (peso giroscopio)
   - 1-α = 0.04 (peso acelerómetro)
   - Fusiona datos para mejor precisión

### 2. Datos GPS
#### Datos Brutos (gps_data.csv)
```csv
timestamp,latitude,longitude,altitude,speed,satellites,hdop
```

#### Datos Procesados
- **Velocidad Filtrada**: Media móvil 5 muestras
- **Distancia**: Fórmula Haversine
- **Precisión**: Factor HDOP × 2.5m

#### Filtros Aplicados
1. **Media Móvil para Velocidad**
   - Ventana: 5 muestras
   - Elimina picos y suaviza cambios

2. **Filtro de Mediana**
   - Ventana: 3 muestras
   - Elimina outliers

### 3. Datos Ambientales
#### Datos Brutos (env_data.csv)
```csv
timestamp,temperature,pressure,humidity
```

#### Datos Procesados
- **Temperatura**: Promedio BMP280/AHT20
- **Altitud Barométrica**: f(presión)
- **Punto de Rocío**: f(temp, humidity)

---

## 💾 Sistema de Almacenamiento

### 1. Buffer y Optimización
- **Tamaño Buffer**: 4KB
- **Flush Automático**: 5 segundos
- **Rotación**: 10MB por archivo
- **Backups**: 5 rotaciones máximo

### 2. Estructura de Sesiones
- Directorio por timestamp
- Archivos CSV independientes
- Headers automáticos
- Comprobación de espacio

### 3. Gestión de Errores
- Verificación de escrituras
- Recuperación automática
- Log de errores
- Backup en fallo

---

## 📈 Métricas y Calibración

### 1. IMU
- **Calibración**
  - Sistema: 3/3
  - Giroscopio: 3/3
  - Acelerómetro: 3/3
  - Magnetómetro: 3/3

- **Deriva**
  - Giroscopio: <0.1°/s
  - Magnetómetro: Compensación offset

### 2. GPS
- **Calidad Señal**
  - HDOP < 2.0
  - Satélites > 6
  - Fix 3D

### 3. Sensores Ambientales
- **Precisión**
  - Temperatura: ±0.5°C
  - Presión: ±1 hPa
  - Humedad: ±2% RH

---

## 🛠️ Configuración y Uso

### 1. Inicialización
```cpp
void setup() {
    ConfigManager::initialize();  // Carga config
    SessionManager::initialize(); // Inicia sesión
    initSensors();              // Configura sensores
}
```

### 2. Captura de Datos
```cpp
void loop() {
    if (newDataAvailable()) {
        processIMUData();    // Procesa IMU
        processGPSData();    // Procesa GPS
        processEnvData();    // Procesa ambientales
        logData();          // Guarda datos
    }
}
```

### 3. Análisis de Datos
- Exportación CSV
- Compatibilidad Excel/Python
- Formato timestamp Unix
- Headers descriptivos

---

## 📝 Notas de Desarrollo
- Verificar calibración antes de uso
- Mantener buffer SD < 75%
- Revisar logs periódicamente
- Actualizar offsets magnéticos

---

## 🔄 Actualizaciones Futuras
- Interfaz web tiempo real
- Compresión de datos
- Machine Learning local
- Batería con backup

---

¡Disfruta del proyecto y sigue experimentando!
